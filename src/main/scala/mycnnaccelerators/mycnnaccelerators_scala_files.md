## File: src/main/scala/mycnnaccelerators/MyCNNRoCC.scala

```scala
// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy.LazyModule
import CNNAcceleratorISA._ // Import ISA definitions

// Key for configuration
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey) // Get base config from Parameters
  // Actual config is created in MyCNNRoCCModuleImp once xLen is known
  override lazy val module = new MyCNNRoCCModuleImp(this, baseConfig)
}

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, defaultConfig: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters { // HasCoreParameters provides xLen, etc.

  // Create the actual config using xLen from HasCoreParameters
  val config = defaultConfig.copy(xLen = xLen)

  // Instantiate internal modules
  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  // RoCC FSM states
  val sIdle :: sWaitOFMRead :: sRespond :: Nil = Enum(3)
  val rocc_state = RegInit(sIdle)

  // Registers for RoCC interaction
  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  val accelerator_status_reg = RegInit(STATUS_IDLE) // For GET_STATUS command
  val ofm_read_addr_reg = Reg(UInt(config.ofmAddrWidth.W)) // For CMD_GET_OFM_ADDR_DATA

  // Default outputs for RoCC interface
  io.cmd.ready := (rocc_state === sIdle) && !resp_valid_reg
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  io.busy := (compute_unit.io.busy || accelerator_status_reg === STATUS_COMPUTING) && (rocc_state =/= sIdle) // Simplified busy logic for now
  io.interrupt := false.B // No interrupts for this simple version

  // Default connections for buffer write ports (controlled by RoCC FSM)
  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.S(config.dataWidth.W) // CORRECTED: SInt literal
  // ifm_buffer.io.read_addr is driven by compute_unit

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.S(config.dataWidth.W) // CORRECTED: SInt literal
  // kernel_buffer.io.read_addr is driven by compute_unit

  // OFM buffer connections
  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en // CU writes to OFM
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data // SInt to SInt (OK)
  ofm_buffer.io.read_addr := 0.U // Controlled by RoCC FSM for readback via CMD_GET_OFM_ADDR_DATA

  // Default connections for compute_unit inputs (controlled by RoCC FSM or wired below)
  compute_unit.io.start := false.B
  // compute_unit.io.ifm_read_addr is an OUTPUT of compute_unit
  // compute_unit.io.ifm_read_data is an INPUT to compute_unit, connected below
  // compute_unit.io.kernel_read_addr is an OUTPUT of compute_unit
  // compute_unit.io.kernel_read_data is an INPUT to compute_unit, connected below

  // RoCC Command Handling FSM
  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B // Clear response valid when CPU accepts it
  }

  switch(rocc_state) {
    is(sIdle) {
      accelerator_status_reg := Mux(compute_unit.io.busy, STATUS_COMPUTING,
                                Mux(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING, STATUS_COMPUTE_DONE,
                                    Mux(accelerator_status_reg === STATUS_COMPUTE_DONE, STATUS_COMPUTE_DONE, STATUS_IDLE)))

      when(io.cmd.valid && !resp_valid_reg) {
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd // Capture destination register for all commands

        io.cmd.ready := false.B // Will be set back to true if cmd processed in one cycle

        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) {
            when(cmd.rs1 < config.ifmDepth.U) {
              ifm_buffer.io.write_en   := true.B
              ifm_buffer.io.write_addr := cmd.rs1
              ifm_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0).asSInt // CORRECTED: Cast to SInt
              accelerator_status_reg := STATUS_LOADING_IFM
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR
            }
            io.cmd.ready := true.B
          }
          is(CMD_SET_KERNEL_ADDR_DATA) {
            when(cmd.rs1 < config.kernelDepth.U) {
              kernel_buffer.io.write_en   := true.B
              kernel_buffer.io.write_addr := cmd.rs1
              kernel_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0).asSInt // CORRECTED: Cast to SInt
              accelerator_status_reg := STATUS_LOADING_KERNEL
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR
            }
            io.cmd.ready := true.B
          }
          is(CMD_START_COMPUTE) {
            when(!compute_unit.io.busy && accelerator_status_reg =/= STATUS_COMPUTING) { // ensure not already busy
              compute_unit.io.start := true.B
              accelerator_status_reg := STATUS_COMPUTING
            }
            io.cmd.ready := true.B
          }
          is(CMD_GET_OFM_ADDR_DATA) {
            when(cmd.rs1 < config.ofmDepth.U) {
              ofm_buffer.io.read_addr := cmd.rs1
              // ofm_read_addr_reg := cmd.rs1 // Not strictly needed if data is read next cycle directly
              rocc_state := sWaitOFMRead
            } .otherwise {
              resp_data_reg := STATUS_ERROR.asUInt // Ensure STATUS_ERROR can fit or is truncated appropriately
              resp_valid_reg := true.B
              accelerator_status_reg := STATUS_ERROR
              rocc_state := sRespond // Go to respond to send error
            }
          }
          is(CMD_GET_STATUS) {
            resp_data_reg := accelerator_status_reg // Status reg is already UInt(8.W), will be zero-extended by assignment to UInt(xLen.W)
            resp_valid_reg := true.B
            rocc_state := sRespond // Go to respond state
          }
        }
      }
    }

    is(sWaitOFMRead) {
      // Data from ofm_buffer.io.read_data is valid in this cycle
      // CORRECTED: Handle SInt from buffer to UInt RoCC response register with sign extension
      val data_from_ofm = Wire(SInt(config.xLen.W))
      data_from_ofm := ofm_buffer.io.read_data // Sign-extends from dataWidth to xLen
      resp_data_reg := data_from_ofm.asUInt    // Convert to UInt

      resp_valid_reg := true.B
      rocc_state := sRespond
    }

    is(sRespond) {
      // Wait for resp_valid_reg to be cleared by CPU io.resp.ready
      when(!resp_valid_reg) {
        io.cmd.ready := true.B
        rocc_state := sIdle
      }
    }
  }

  // Connect Compute Unit to Buffers
  // Buffers provide data to CU (input to CU)
  compute_unit.io.ifm_read_data    := ifm_buffer.io.read_data    // SInt to SInt (OK)
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data // SInt to SInt (OK)

  // CU drives its own read addresses to the buffers (output from CU)
  ifm_buffer.io.read_addr    := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr := compute_unit.io.kernel_read_addr

  // When compute unit is done, and RoCC was in computing state, update RoCC's status
  when(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING) {
      accelerator_status_reg := STATUS_COMPUTE_DONE
  }
}
```

## File: src/main/scala/mycnnaccelerators/MinimalBuffer.scala

```scala
// filename: MinimalBuffer.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class MinimalBufferIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  // Write port
  val write_en   = Input(Bool())
  val write_addr = Input(UInt(addrWidth.W))
  val write_data = Input(SInt(dataWidth.W))

  // Read port
  val read_addr  = Input(UInt(addrWidth.W))
  val read_data  = Output(SInt(dataWidth.W))
}

class MinimalBuffer(depth: Int, dataWidth: Int) extends Module {
  val addrWidth = log2Ceil(depth) // IO 仍然需要地址宽度
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  // 使用寄存器向量 (Reg of Vec) 来实现存储

  val mem_reg = Reg(Vec(depth, SInt(dataWidth.W)))

  // 写操作
  when(io.write_en) {
    mem_reg(io.write_addr) := io.write_data
  }

  // 读操作
  // 为了保持与 SyncReadMem 或 Mem + RegNext 相似的 1 周期读延迟：
  // mem_reg(io.read_addr) 是对寄存器向量的组合逻辑读
  // RegNext 会将这个组合逻辑读的结果在下一个周期输出
  io.read_data := RegNext(mem_reg(io.read_addr))

  // 如果允许0周期读延迟（纯组合逻辑读），可以这样写：
  // io.read_data := mem_reg(io.read_addr)
  // 但我们之前的目标是模拟 SyncReadMem 的行为（1周期延迟）
}
```

## File: src/main/scala/mycnnaccelerators/AcceleratorConfig.scala

```scala
// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3._
import chisel3.util.log2Ceil

// Basic Accelerator Configuration
case class AcceleratorConfig(
  // Data width for IFM, Kernel, OFM elements
  dataWidth: Int = 8,

  // IFM Dimensions (fixed)
  ifmRows: Int = 8, // Example: Small IFM
  ifmCols: Int = 8,

  // Kernel Dimensions (fixed)
  kernelRows: Int = 3, // Example: 3x3 Kernel
  kernelCols: Int = 3,

  // RoCC Core's xLen (register width, e.g., 64 bits)
  // This will be updated by the RoCC wrapper from the core's parameters.
  xLen: Int = 64 // Default, will be overridden
) {
  // Derived parameters
  val ifmDepth: Int = ifmRows * ifmCols
  val kernelDepth: Int = kernelRows * kernelCols

  // OFM Dimensions (calculated for 'valid' convolution)
  val ofmRows: Int = ifmRows - kernelRows + 1
  val ofmCols: Int = ifmCols - kernelCols + 1
  val ofmDepth: Int = ofmRows * ofmCols

  // Address widths for buffers
  val ifmAddrWidth: Int = log2Ceil(ifmDepth)
  val kernelAddrWidth: Int = log2Ceil(kernelDepth)
  val ofmAddrWidth: Int = log2Ceil(ofmDepth)
}

// Default configuration object, xLen is a placeholder here.
// MyCNNRoCC will create the actual config with the correct xLen.
object DefaultAcceleratorConfig extends AcceleratorConfig()
```

## File: src/main/scala/mycnnaccelerators/ComputeUnit.scala

```scala
// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())

  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W))
  val ifm_read_data = Input(SInt(config.dataWidth.W)) // Changed to SInt

  val kernel_read_addr = Output(UInt(config.kernelAddrWidth.W))
  val kernel_read_data = Input(SInt(config.dataWidth.W)) // Changed to SInt

  val ofm_write_en   = Output(Bool())
  val ofm_write_addr = Output(UInt(config.ofmAddrWidth.W))
  val ofm_write_data = Output(SInt(config.dataWidth.W)) // Changed to SInt
}

class ComputeUnit(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitIO(config))

  val F_BITS = 8 // Number of fractional bits for Q8.8 format

  // States for the Compute Unit FSM
  val sIdle :: sFetchIFM :: sFetchKernel :: sMAC :: sWriteOFM :: sDoneCU :: Nil = Enum(6)
  val state = RegInit(sIdle)

  // Registers for loop counters
  val ofm_row = RegInit(0.U(log2Ceil(config.ofmRows).W))
  val ofm_col = RegInit(0.U(log2Ceil(config.ofmCols).W))
  val k_row   = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_col   = RegInit(0.U(log2Ceil(config.kernelCols).W))

  // Accumulator:
  // IFM (Q8.8) * Kernel (Q8.8) = Product (Q16.16, 32-bit SInt)
  // Accumulator adds Product values. To prevent overflow, it needs guard bits for the integer part.
  // Accumulator precision: Q(16 + GuardBits).16
  // GuardBits = log2Ceil(max number of accumulations, i.e., kernelRows * kernelCols)
  val guardBits = log2Ceil(config.kernelRows * config.kernelCols)
  val accProductIntWidth = config.dataWidth // Integer part of product is 16 bits (dataWidth for Q8.8 means 8 for int, but 8+8=16 for product int part)
  val accFracWidth = config.dataWidth     // Fractional part of product is 16 bits
  val accIntWidth = accProductIntWidth + guardBits
  val accWidth = accIntWidth + accFracWidth // Total width for Q(16+G).16

  val accumulator = RegInit(0.S(accWidth.W))

  // Registers to hold data read from buffers
  val ifm_data_reg    = Reg(SInt(config.dataWidth.W))
  val kernel_data_reg = Reg(SInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B)

  // Default outputs
  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)

  io.ifm_read_addr   := 0.U
  io.kernel_read_addr:= 0.U
  io.ofm_write_en    := false.B
  io.ofm_write_addr  := 0.U

  // Output conversion: Accumulator is Q(I_acc).16, OFM is Q8.8 (SInt(16.W))
  // Need to shift right by (16_frac_acc - 8_frac_ofm) = 8 bits
  val FRACTIONAL_BITS_IN_ACCUMULATOR = config.dataWidth // Product is Q16.16, so 16 fractional bits
  val shift_amount = FRACTIONAL_BITS_IN_ACCUMULATOR - F_BITS
  val shifted_accumulator = (accumulator >> shift_amount) // Now effectively Q(I_acc).(F_BITS)

  // Select the 16 bits for Q8.8 output (SInt(16.W))
  // The LSB of shifted_accumulator now aligns with the LSB of the target F_BITS fractional part.
  // We need to take 'config.dataWidth' (16) bits.
  // Add rounding and saturation for better results (not implemented here for simplicity)
  io.ofm_write_data := shifted_accumulator(config.dataWidth - 1, 0).asSInt


  switch(state) {
    is(sIdle) {
      when(io.start) {
        ofm_row     := 0.U
        ofm_col     := 0.U
        k_row       := 0.U
        k_col       := 0.U
        accumulator := 0.S
        state       := sFetchIFM
        data_fetch_cycle := false.B
      }
    }

    is(sFetchIFM) {
      when(!data_fetch_cycle) {
        val current_ifm_row = ofm_row + k_row
        val current_ifm_col = ofm_col + k_col
        io.ifm_read_addr := current_ifm_row * config.ifmCols.U + current_ifm_col
        data_fetch_cycle := true.B
      } .otherwise {
        ifm_data_reg := io.ifm_read_data // Latch IFM data
        state := sFetchKernel
        data_fetch_cycle := false.B
      }
    }

    is(sFetchKernel) {
      when(!data_fetch_cycle) {
        io.kernel_read_addr := k_row * config.kernelCols.U + k_col
        data_fetch_cycle := true.B
      } .otherwise {
        kernel_data_reg := io.kernel_read_data // Latch Kernel data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      // ifm_data_reg (SInt Q8.8 - 16 bits), kernel_data_reg (SInt Q8.8 - 16 bits)
      // Product is SInt Q16.16 - 32 bits
      val product = (ifm_data_reg * kernel_data_reg).asSInt // Chisel handles widening

      // Accumulator is Q(16+G).16. Product is Q16.16.
      // Sign-extend product to accumulator's width before adding.
      // The .asSInt conversion might not be needed if product is already SInt.
      // The '+' operator with different SInt widths should handle sign extension.
      accumulator := accumulator + product

      k_col := k_col + 1.U
      when(k_col + 1.U === config.kernelCols.U) {
        k_col := 0.U
        k_row := k_row + 1.U
        when(k_row + 1.U === config.kernelRows.U) {
          k_row := 0.U
          state := sWriteOFM
        } .otherwise {
          state := sFetchIFM
        }
      } .otherwise {
        state := sFetchIFM
      }
    }

    is(sWriteOFM) {
      io.ofm_write_en   := true.B
      io.ofm_write_addr := ofm_row * config.ofmCols.U + ofm_col
      // io.ofm_write_data is connected via default assignment above

      accumulator := 0.S // Reset accumulator for next OFM element

      ofm_col := ofm_col + 1.U
      when(ofm_col + 1.U === config.ofmCols.U) {
        ofm_col := 0.U
        ofm_row := ofm_row + 1.U
        when(ofm_row + 1.U === config.ofmRows.U) {
          state := sDoneCU
        } .otherwise {
          k_row := 0.U
          k_col := 0.U
          state := sFetchIFM
        }
      } .otherwise {
        k_row := 0.U
        k_col := 0.U
        state := sFetchIFM
      }
    }

    is(sDoneCU) {
      when(!io.start) { // Wait for start to de-assert before idling
        state := sIdle
      }
    }
  }
}
```

## File: src/main/scala/mycnnaccelerators/CNNAcceleratorISA.scala

```scala
// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._

object CNNAcceleratorISA {
  // Funct values for RoCC custom instructions (7-bit)
  // rs1 generally for address, rs2 for data (if applicable)
  // rd generally for response data (if applicable)

  val CMD_SET_IFM_ADDR_DATA   = 0.U(7.W) // rs1: address in IFM buffer, rs2: data
  val CMD_SET_KERNEL_ADDR_DATA= 1.U(7.W) // rs1: address in Kernel buffer, rs2: data
  val CMD_START_COMPUTE       = 2.U(7.W) // No operands needed if data is pre-loaded
  val CMD_GET_OFM_ADDR_DATA   = 3.U(7.W) // rs1: address in OFM buffer, rd: data from OFM
  val CMD_GET_STATUS          = 4.U(7.W) // rd: status code

  // Status codes (ensure fits in xLen, typically 8-bits are plenty)
  val STATUS_IDLE             = 0.U(8.W)
  val STATUS_LOADING_IFM      = 1.U(8.W) // Optional: if we want finer grain status
  val STATUS_LOADING_KERNEL   = 2.U(8.W) // Optional
  val STATUS_COMPUTING        = 3.U(8.W)
  val STATUS_COMPUTE_DONE     = 4.U(8.W) // Computation finished, OFM is ready
  val STATUS_ERROR            = 255.U(8.W) // General error
}
```

## Content from: RISC-V.md

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

