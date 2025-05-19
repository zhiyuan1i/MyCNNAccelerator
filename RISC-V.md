**核心思路：**

- **简化数据通路：** 避免复杂的 `im2col` 硬件转换。我们将假设数据以适合直接卷积的方式被 DMA 加载和处理。
- **简化控制：** 由于尺寸固定 (输入 32x32, 卷积核最大 5x5)，很多控制信号和循环边界可以简化。
- **DMA 核心：** DMA 负责将输入特征图 (IFM)、卷积核 (Kernel/Weights) 从主存加载到加速器内部的专用缓冲区，并将输出特征图 (OFM) 从加速器缓冲区写回主存。
- **配置与执行分离：** 使用不同的 RoCC 指令来配置地址、参数，并启动运算。

### RoCC 指令设计 (针对简单卷积)

为了保持简单，我们可以设计几条核心的 RoCC 指令。这里假设使用 R-Format 指令，其中 `funct` 字段用于区分不同的加速器操作。`rs1` 和 `rs2` 用于传递参数，`rd` 用于接收操作的状态或结果（如果需要）。

1. **`CONFIG_IFM_ADDR` (配置输入特征图地址)**
   - `funct = 0`
   - `rs1`: 输入特征图在主存中的基地址。
   - `rs2`: (可选) 加速器内部 IFM 缓冲区的目标地址/偏移量 (如果需要CPU指定)。如果只有一个固定缓冲区，此参数可能不需要。
   - `rd`: (可选) 返回状态码 (例如，配置成功)。
2. **`CONFIG_KERNEL_ADDR_SIZE` (配置卷积核地址和尺寸)**
   - `funct = 1`
   - `rs1`: 卷积核在主存中的基地址。
   - `rs2`:
     - 低位 (例如 `rs2[4:0]`): 卷积核尺寸 K (例如，3 表示 3x3，5 表示 5x5)。
     - 高位: (可选) 加速器内部卷积核缓冲区的目标地址/偏移量。
   - `rd`: (可选) 返回状态码。
3. **`CONFIG_OFM_ADDR_PARAMS` (配置输出特征图地址和卷积参数)**
   - `funct = 2`
   - `rs1`: 输出特征图在主存中的基地址。
   - `rs2`
     - 低位 (例如 `rs2[7:0]`): 卷积步长 (stride)。由于输出与输入同尺寸 (32x32)，且卷积核最大 5x5，为保持尺寸不变，如果 K=3, stride=1, padding=1; 如果 K=5, stride=1, padding=2。Padding 可以由 K 和 stride 推断，或者在此处明确指定。为了简单，我们假设 stride=1，padding 根据 K 自动推断以保持输出尺寸。
     - (可选) 其他参数，如激活函数类型等，如果您的设计支持。
   - `rd`: (可选) 返回状态码。
4. **`START_CONVOLUTION` (启动卷积运算)**
   - `funct = 3`
   - `rs1`, `rs2`: (可选) 可以不使用，或者用于传递一些运行时的小参数。
   - `rd`: (可选) 可以不立即返回值，CPU 通过后续的 `GET_STATUS` 指令查询。或者，如果 RoCC 支持阻塞，CPU 会等待直到操作完成。对于非阻塞设计，`rd` 可以先返回一个任务 ID 或“已启动”状态。
5. **`GET_STATUS` (获取加速器状态/结果)**
   - `funct = 4`
   - `rs1`: (可选) 任务 ID (如果 `START_CONVOLUTION` 返回任务 ID)。
   - `rs2`: (未使用)
   - `rd`: 返回状态码。例如：
     - `0`: 计算完成且成功。
     - `1`: 仍在计算中。
     - `>1`: 错误代码。

**指令字段分配示例 (基于标准 R-Format `funct7 | rs2 | rs1 | funct3 | rd | opcode`):**

- `opcode`: RoCC 操作码。
- `funct3`: 可以固定，或者用于进一步区分操作的子类型（如果需要）。
- `funct7`: 在这里我们用 `funct` (通常在 RoCC 指令中由 `inst.funct` 字段提供) 来主要区分我们的 5 条指令。

### FSM (Finite State Machine) 状态设计

这个 FSM 描述了加速器在 RoCC 接口命令驱动下的主要行为流程。

- **`sIdle` (空闲状态):**

  - 加速器处于空闲状态，等待 CPU 发送命令。
  - `io.cmd.ready := true` (可以接收新命令)。
  - 转移:
    - 收到 `CONFIG_IFM_ADDR`: 保存 `rs1` (IFM 主存地址) 到内部寄存器。如果 `rd` 需要返回状态，设置响应。
    - 收到 `CONFIG_KERNEL_ADDR_SIZE`: 保存 `rs1` (卷积核主存地址) 和 `rs2` (卷积核尺寸 K) 到内部寄存器。设置响应。
    - 收到 `CONFIG_OFM_ADDR_PARAMS`: 保存 `rs1` (OFM 主存地址) 和 `rs2` (卷积参数) 到内部寄存器。设置响应。
    - 收到 `START_CONVOLUTION`:
      - `io.cmd.ready := false` (加速器开始忙碌)。
      - 如果所有必要配置都已完成，则转移到 `sDmaLoadIFM_Start`。
      - 否则 (例如，配置不完整)，可以保持在 `sIdle` 并通过 `GET_STATUS` 返回错误，或者设计一个 `sError` 状态。为了简单，我们假设 CPU 会按顺序发送配置指令。
    - 收到 `GET_STATUS`
      - `io.resp.bits.data := IDLE_STATUS_CODE` (例如，一个表示空闲的代码)。
      - `io.resp.valid := true`。
      - `io.resp.bits.rd := io.cmd.bits.inst.rd`。

- **`sDmaLoadIFM_Start` (开始 DMA 加载输入特征图):**

  - 向 DMA 控制器发出请求，从 `ifm_base_addr_reg` 加载 32x32 的输入特征图到加速器内部的 IFM 缓冲区。
  - 转移到 `sDmaLoadIFM_Busy`。

- **`sDmaLoadIFM_Busy` (DMA 加载 IFM 中):**

  - 等待 DMA 控制器完成 IFM 的加载。
  - `io.busy := true`。
  - 转移:
    - DMA 完成 IFM 加载: 转移到 `sDmaLoadKernel_Start`。
    - DMA 发生错误: 记录错误状态，转移到 `sError` (或直接在 `sDone` 时通过状态码返回)。

- **`sDmaLoadKernel_Start` (开始 DMA 加载卷积核):**

  - 向 DMA 控制器发出请求，从 `w_base_addr_reg` 加载 KxK 的卷积核到加速器内部的卷积核缓冲区。
  - 转移到 `sDmaLoadKernel_Busy`。

- **`sDmaLoadKernel_Busy` (DMA 加载卷积核中):**

  - 等待 DMA 控制器完成卷积核的加载。
  - `io.busy := true`。
  - 转移:
    - DMA 完成卷积核加载: 转移到 `sCompute_Setup`。
    - DMA 发生错误: 记录错误状态。

- **`sCompute_Setup` (卷积计算准备):**

  - 初始化卷积计算所需的内部指针和计数器 (例如，输出 OFM 的行/列计数器，卷积核在 IFM 上的滑动位置计数器，累加器清零等)。
  - 计算所需的 padding (基于 K 和 stride=1，目标输出 32x32)。
  - `io.busy := true`。
  - 转移到 `sCompute`。

- **`sCompute` (执行卷积计算):**

  - 这是核心计算状态。内部会有一个或多个子状态/循环来处理：
    - 遍历输出 OFM 的每一个像素位置 (32x32)。
    - 对于每个输出像素，根据卷积核大小 KxK 和 padding，从 IFM 缓冲区读取相应的输入像素块。
    - 从卷积核缓冲区读取权重。
    - 执行乘累加 (MAC) 操作。
    - 将结果存入 OFM 缓冲区。
  - `io.busy := true`。
  - 转移:
    - 所有 OFM 像素计算完成: 转移到 `sDmaStoreOFM_Start`。
    - 计算中发生错误 (例如，数值溢出，如果需要检查): 记录错误状态。

- **`sDmaStoreOFM_Start` (开始 DMA 存储输出特征图):**

  - 向 DMA 控制器发出请求，将 OFM 缓冲区的内容写回到主存的 `ofm_base_addr_reg`。
  - 转移到 `sDmaStoreOFM_Busy`。

- **`sDmaStoreOFM_Busy` (DMA 存储 OFM 中):**

  - 等待 DMA 控制器完成 OFM 的存储。
  - `io.busy := true`。
  - 转移:
    - DMA 完成 OFM 存储:
      - `computation_result_status := SUCCESS_CODE` (或之前记录的错误码)。
      - 转移到 `sDone`。
    - DMA 发生错误: 记录错误状态。

- **`sDone` (计算/存储完成):**

  - 加速器已完成所有操作，结果（或错误状态）已准备好。

  - `io.busy := true` (直到 CPU 通过 `GET_STATUS` 清除或 RoCC 响应发出)。

  - 转移:

    - 收到 

      ```
      GET_STATUS
      ```

       命令 (或者 RoCC 自动响应机制触发):

      - `io.resp.bits.data := computation_result_status`。
      - `io.resp.valid := true`。
      - `io.resp.bits.rd := rd_from_start_conv_cmd` (之前 `START_CONVOLUTION` 指令的目标寄存器)。
      - (当 `io.resp.fire` 时) 转移到 `sIdle`。
      - `io.cmd.ready := true` (可以接收下一条 `GET_STATUS` 或新配置命令)。

**FSM 状态图 (Mermaid 格式):**

代码段

```
graph TD
    subgraph "RoCC Command Handling"
        sIdle("sIdle: 空闲等待命令")
    end

    subgraph "DMA Load IFM"
        sDmaLoadIFM_Start("sDmaLoadIFM_Start: <br/>启动DMA加载IFM")
        sDmaLoadIFM_Busy("sDmaLoadIFM_Busy: <br/>等待IFM加载完成")
    end

    subgraph "DMA Load Kernel"
        sDmaLoadKernel_Start("sDmaLoadKernel_Start: <br/>启动DMA加载卷积核")
        sDmaLoadKernel_Busy("sDmaLoadKernel_Busy: <br/>等待卷积核加载完成")
    end

    subgraph "Convolution Computation"
        sCompute_Setup("sCompute_Setup: <br/>准备卷积计算参数")
        sCompute("sCompute: 执行卷积计算")
    end

    subgraph "DMA Store OFM"
        sDmaStoreOFM_Start("sDmaStoreOFM_Start: <br/>启动DMA存储OFM")
        sDmaStoreOFM_Busy("sDmaStoreOFM_Busy: <br/>等待OFM存储完成")
    end

    subgraph "Completion"
        sDone("sDone: 操作完成, <br/>等待CPU获取状态")
    end

    sIdle -- funct=0 (CONFIG_IFM_ADDR) --> sIdle
    sIdle -- funct=1 (CONFIG_KERNEL_ADDR_SIZE) --> sIdle
    sIdle -- funct=2 (CONFIG_OFM_ADDR_PARAMS) --> sIdle
    sIdle -- funct=3 (START_CONVOLUTION) --> sDmaLoadIFM_Start
    sIdle -- funct=4 (GET_STATUS) & Idle --> sIdle

    sDmaLoadIFM_Start --> sDmaLoadIFM_Busy
    sDmaLoadIFM_Busy -- DMA IFM Done --> sDmaLoadKernel_Start
    sDmaLoadKernel_Start --> sDmaLoadKernel_Busy
    sDmaLoadKernel_Busy -- DMA Kernel Done --> sCompute_Setup
    sCompute_Setup --> sCompute
    sCompute -- Computation Done --> sDmaStoreOFM_Start
    sDmaStoreOFM_Start --> sDmaStoreOFM_Busy
    sDmaStoreOFM_Busy -- DMA OFM Done --> sDone

    sDone -- funct=4 (GET_STATUS) & Done / io.resp.fire --> sIdle
```

**关于您的 2x2 矩阵乘法参考代码：**

您的 `MyMACModuleImp` 示例是一个很好的起点，它展示了基本的 RoCC 命令解码和状态管理。对于卷积加速器：

- **状态会更多：** 因为涉及到多次 DMA 操作和核心计算阶段。
- **配置寄存器更多：** 需要存储 IFM、卷积核、OFM 的地址，以及卷积核大小、步长等参数。
- **核心计算逻辑更复杂：** `sCompute` 状态内部将包含循环和地址生成逻辑，用于在 IFM 上滑动卷积核并执行 MAC。您需要设计 PE（处理单元）或一组 PE 来执行这些 MAC 操作。对于 32x32 输入和 5x5 卷积核，如果不想太复杂，可以考虑串行处理输出像素，或者实现一个小型的并行 PE 阵列（例如，一次计算一个输出像素，或者一行输出像素）。
- **DMA 交互：** 您需要一个简单的 DMA 控制器接口。在 FSM 中，当进入 DMA 相关状态时，您会向 DMA 控制器发出命令（地址、长度、方向），然后等待 DMA 控制器发回完成信号。

**下一步：**

1. **细化 RoCC 指令的 `rs1` 和 `rs2` 字段：** 确保所有必要的参数（如地址、卷积核大小 K、步长、padding 策略，如果不由 K 推断的话）都可以通过这些指令传递给加速器。
2. **设计内部缓冲区：** 决定 IFM、卷积核和 OFM 在加速器内部如何存储（例如，简单的 SRAM 块）。
3. **设计 DMA 控制器接口：** 定义加速器如何请求 DMA 操作以及如何接收完成/错误信号。
4. **设计计算核心 (`sCompute` 状态的内部逻辑)：** 这是最复杂的部分，涉及到如何从缓冲区读取数据、如何执行乘累加、如何处理 padding 以及如何写入输出缓冲区。





**要点和下一步：**

- DMA 与 Scratchpad 的连接：
  - 在 `MyCNNRoCC.scala` 中，`SimpleDMAController` 的 `spad_read` 和 `spad_write` 接口需要连接到 `Scratchpad` 模块的相应端口。
  - `CNNController` 模块需要根据当前正在进行的 DMA 操作 (加载 IFM、加载 Kernel、还是存储 OFM) 来正确地将 `SimpleDMAController` 的 `spad_read`/`spad_write` 路由到 `Scratchpad` 模块的 `ifm_write`、`kernel_write` 或 `ofm_read` 端口。
  - 目前我给出的 `CNNControllerIO` 和 `MyCNNRoCC` 中的连接是概念性的，您需要仔细设计这部分，确保 DMA 读写的是正确的内部 buffer。一种方式是 `CNNController` 直接拥有对 `Scratchpad` 所有端口的访问权，并根据当前状态来驱动它们以及 `SimpleDMAController`。
- **定点运算：** `ComputeUnit.scala` 中的 `fixed_mul` 和 `fixed_add` 只是占位符。您需要实现实际的 8.8 定点数乘法和加法，包括可能的移位和饱和逻辑。
- **`ComputeUnit` 的详细实现：** `sCompute` 状态的内部逻辑 (循环、地址生成、padding) 将是您设计中最复杂的部分。
- **`SimpleDMAController` 的 `io.mem` 交互：** 需要仔细处理 `io.mem.req.ready` 和 `io.mem.resp.valid`，以确保正确的数据传输。
- 逐步实现和测试：
  1. 先实现 RoCC 接口和配置指令的响应 (`MyCNNRoCC` 和 `CNNController` 的部分 FSM)。
  2. 然后实现 `Scratchpad` 模块，并进行单元测试。
  3. 接着实现 `SimpleDMAController` 与 `Scratchpad` 和 `io.mem` 的交互，并测试 DMA 加载/存储。
  4. 实现 `ComputeUnit` 的核心计算逻辑，并进行单元测试。
  5. 最后将所有模块集成起来，通过 `CNNController` 的 FSM 进行协调，并进行完整的 RoCC 命令序列测试。