## File: src/main/scala/mycnnaccelerators/MyCNNRoCCModuleImp.scala

```scala
// filename: MyCNNRoCCModuleImp.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.diplomacy._ // Required for LazyModule
import CNNAcceleratorISA._

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, defaultConfig: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) // outer is MyCNNRoCC
    with HasCoreParameters { // HasCoreParameters provides xLen, etc.

  val config = defaultConfig.copy(xLen = xLen)

  // Instantiate internal modules
  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  // Access the module of the DMA engine defined in the outer MyCNNRoCC
  // This provides the control/status interface for the DMA
  val dma_ctrl_io = outer.dma_engine_lazy.module.io

  // The RoCC's io.mem is diplomatically connected to the DMA's TileLink node.
  // No explicit connection like dma_engine.io.mem <> io.mem is needed here.

  // RoCC FSM states
  val sIdle :: sWaitOFMRead :: sRespond :: sDmaActive :: Nil = Enum(4)
  val rocc_state = RegInit(sIdle)

  // Registers for RoCC interaction
  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  // Accelerator status and DMA configuration registers
  val accelerator_status_reg = RegInit(STATUS_IDLE)

  val dma_mem_base_addr_reg = Reg(UInt(xLen.W)) // xLen should match coreMaxAddrBits for addresses typically
  val dma_length_bytes_reg = Reg(UInt(dmaConfigLenBits.W))
  val dma_direction_reg = Reg(UInt(dmaConfigDirBits.W))
  val dma_buffer_id_reg = Reg(UInt(dmaConfigBufIdBits.W))
  val dma_configured_flag = RegInit(false.B)

  // Default outputs for RoCC interface
  io.cmd.ready := (rocc_state === sIdle) && !resp_valid_reg && !dma_ctrl_io.busy
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  io.busy := (compute_unit.io.busy || dma_ctrl_io.busy || (rocc_state =/= sIdle))
  io.interrupt := false.B

  // Default connections for buffer write ports (DMA will control these during DMA ops)
  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.S(config.dataWidth.W)

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.S(config.dataWidth.W)

  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en // CU writes to OFM
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data
  ofm_buffer.io.read_addr := 0.U // Controlled by RoCC FSM or DMA

  // Compute unit connections
  compute_unit.io.start := false.B
  // ... (rest of compute_unit connections)

  // DMA Engine control inputs (controlled by this FSM)
  dma_ctrl_io.start := false.B
  dma_ctrl_io.mem_base_addr := dma_mem_base_addr_reg
  dma_ctrl_io.length_bytes := dma_length_bytes_reg
  dma_ctrl_io.directionBits := dma_direction_reg
  dma_ctrl_io.buffer_id := dma_buffer_id_reg // RoCC tells DMA which conceptual buffer to use

  // RoCC Command Handling FSM
  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B
  }

  val dma_addr_configured_internal_flag = RegInit(false.B)

  switch(rocc_state) {
    is(sIdle) {
      when(dma_ctrl_io.busy) {
        accelerator_status_reg := STATUS_DMA_BUSY
      } .elsewhen(compute_unit.io.busy) {
        accelerator_status_reg := STATUS_COMPUTING
      } .elsewhen(accelerator_status_reg === STATUS_DMA_BUSY && dma_ctrl_io.done) { // DMA just finished
        // Status update upon DMA completion is now handled in sDmaActive state
        // Here, we transition from DMA_BUSY (set by sDmaActive moving to sIdle) back to IDLE or specific done
        // This logic path might need refinement depending on how sDmaActive sets status before returning to sIdle
        accelerator_status_reg := STATUS_IDLE // Placeholder, sDmaActive should set a more specific status
        dma_configured_flag := false.B
        dma_addr_configured_internal_flag := false.B
      } .elsewhen(accelerator_status_reg === STATUS_COMPUTING && compute_unit.io.done) {
        accelerator_status_reg := STATUS_COMPUTE_DONE
      } .elsewhen(accelerator_status_reg === STATUS_COMPUTE_DONE ||
                  accelerator_status_reg === STATUS_DMA_CONFIG_READY ||
                  accelerator_status_reg === STATUS_DMA_IFM_LOAD_DONE || // Persist DMA done states
                  accelerator_status_reg === STATUS_DMA_KERNEL_LOAD_DONE ||
                  accelerator_status_reg === STATUS_DMA_OFM_STORE_DONE ||
                  accelerator_status_reg === STATUS_DMA_ERROR) {
        // Persist these states until cleared or new operation starts
      } .otherwise {
        when(!dma_configured_flag && !dma_ctrl_io.busy && !compute_unit.io.busy) {
            accelerator_status_reg := STATUS_IDLE
        }
      }

      when(io.cmd.valid && !resp_valid_reg) {
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd

        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) { /* ... */ }
          is(CMD_SET_KERNEL_ADDR_DATA) { /* ... */ }
          is(CMD_START_COMPUTE) { /* ... */ }
          is(CMD_GET_OFM_ADDR_DATA) { /* ... */ }

          is(CMD_DMA_CONFIG_ADDR) {
            dma_mem_base_addr_reg := cmd.rs1
            dma_addr_configured_internal_flag := true.B
            dma_configured_flag := false.B
            accelerator_status_reg := STATUS_IDLE // Or DMA_CONFIG_IN_PROGRESS
          }
          is(CMD_DMA_CONFIG_PARAMS) {
            when(dma_addr_configured_internal_flag) {
              val param_val = cmd.rs1
              dma_buffer_id_reg := param_val(dmaConfigBufIdBits - 1, 0)
              dma_direction_reg := param_val(dmaConfigBufIdBits)
              dma_length_bytes_reg := param_val(dmaConfigTotalParamBits - 1, dmaConfigBufIdBits + dmaConfigDirBits)

              dma_configured_flag := true.B
              dma_addr_configured_internal_flag := false.B
              accelerator_status_reg := STATUS_DMA_CONFIG_READY
            } .otherwise {
              accelerator_status_reg := STATUS_DMA_ERROR // Error: PARAMS before ADDR
            }
          }
          is(CMD_DMA_START) {
            when(dma_configured_flag && !dma_ctrl_io.busy && !compute_unit.io.busy) {
              dma_ctrl_io.start := true.B // Start the DMA
              rocc_state := sDmaActive
              accelerator_status_reg := STATUS_DMA_BUSY
            } .otherwise {
              resp_data_reg := STATUS_ERROR.asUInt // Cannot start DMA
              resp_valid_reg := true.B
              rocc_state := sRespond
            }
          }
          is(CMD_GET_STATUS) {
            resp_data_reg := accelerator_status_reg
            resp_valid_reg := true.B
            rocc_state := sRespond
          }
        }
      }
    } // sIdle

    is(sDmaActive) {
      // Monitor DMA completion or error
      when(dma_ctrl_io.done) {
        // Use the RoCC's own registers that configured this DMA operation
        val completed_op_buffer = dma_buffer_id_reg
        val completed_op_dir = dma_direction_reg

        when(completed_op_dir === DMADirection.MEM_TO_BUF) {
          when(completed_op_buffer === BufferIDs.IFM) { accelerator_status_reg := STATUS_DMA_IFM_LOAD_DONE }
          .elsewhen(completed_op_buffer === BufferIDs.KERNEL) { accelerator_status_reg := STATUS_DMA_KERNEL_LOAD_DONE }
          .otherwise { accelerator_status_reg := STATUS_DMA_ERROR }
        } .elsewhen (completed_op_dir === DMADirection.BUF_TO_MEM) {
          when(completed_op_buffer === BufferIDs.OFM) { accelerator_status_reg := STATUS_DMA_OFM_STORE_DONE }
          .otherwise { accelerator_status_reg := STATUS_DMA_ERROR }
        } .otherwise {
          accelerator_status_reg := STATUS_DMA_ERROR
        }
        dma_configured_flag := false.B // Ready for new DMA config
        dma_addr_configured_internal_flag := false.B
        rocc_state := sIdle
      } .elsewhen(dma_ctrl_io.dma_error) {
        accelerator_status_reg := STATUS_DMA_ERROR
        dma_configured_flag := false.B
        dma_addr_configured_internal_flag := false.B
        rocc_state := sIdle // Or a persistent error state
      }
      // else, remain in sDmaActive, dma_ctrl_io.start remains asserted by default (false)
      // dma_ctrl_io.start should be de-asserted once DMA is seen to be busy or done
      // This is typically handled by the DMA itself (latches start) or by de-asserting start in the next cycle.
      // For now, assuming DMA latches start. If not, dma_ctrl_io.start := false.B would be needed here.
    }

    is(sWaitOFMRead) { /* ... */ }
    is(sRespond) {
      when(!resp_valid_reg) { // Response has been taken by CPU
        rocc_state := sIdle
      }
    }
  }

  // Placeholder for DMA to Buffer Muxing (Part 2)
  // Example:
  // when(dma_ctrl_io.spad_write_en && dma_ctrl_io.buffer_id === BufferIDs.IFM) { // buffer_id here is from DMA's perspective if it outputs it
  // IFM buffer's write port might be driven by dma_ctrl_io.spad_write_...
  // }
  // For now, DMA directly controls its spad outputs based on its internal buffer_id input.
  // The RoCC needs to connect these spad signals to the correct buffer.
  // E.g. ifm_buffer.io.write_en := dma_ctrl_io.spad_write_en && (dma_buffer_id_reg === BufferIDs.IFM) && (dma_direction_reg === DMADirection.MEM_TO_BUF)
  // And dma_ctrl_io.spad_read_data needs to be fed from the correct buffer.
  // This part requires careful muxing based on dma_buffer_id_reg and dma_direction_reg.

  // Connect Compute Unit to Buffers
  compute_unit.io.ifm_read_data   := ifm_buffer.io.read_data
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data
  ifm_buffer.io.read_addr      := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr   := compute_unit.io.kernel_read_addr

  when(compute_unit.io.done && (accelerator_status_reg === STATUS_COMPUTING || accelerator_status_reg === STATUS_DMA_KERNEL_LOAD_DONE || accelerator_status_reg === STATUS_DMA_IFM_LOAD_DONE) ) {
    accelerator_status_reg := STATUS_COMPUTE_DONE
  }

  // TODO: Mux DMA spad interface to/from ifm, kernel, ofm buffers
  // This is a simplified example, actual muxing depends on DMA engine's spad signals
  // and how buffer_id is used by the DMA engine. Assuming DMA drives spad_write_data
  // and expects spad_read_data based on its configured operation.

  val spad_write_target_is_ifm = dma_buffer_id_reg === BufferIDs.IFM && dma_direction_reg === DMADirection.MEM_TO_BUF
  val spad_write_target_is_kernel = dma_buffer_id_reg === BufferIDs.KERNEL && dma_direction_reg === DMADirection.MEM_TO_BUF

  ifm_buffer.io.write_en := dma_ctrl_io.spad_write_en && spad_write_target_is_ifm
  ifm_buffer.io.write_addr := dma_ctrl_io.spad_write_addr // Assuming DMA addr is local to buffer
  ifm_buffer.io.write_data := dma_ctrl_io.spad_write_data

  kernel_buffer.io.write_en := dma_ctrl_io.spad_write_en && spad_write_target_is_kernel
  kernel_buffer.io.write_addr := dma_ctrl_io.spad_write_addr // Assuming DMA addr is local to buffer
  kernel_buffer.io.write_data := dma_ctrl_io.spad_write_data

  // For DMA reading from OFM buffer and writing to memory
  val spad_read_source_is_ofm = dma_buffer_id_reg === BufferIDs.OFM && dma_direction_reg === DMADirection.BUF_TO_MEM

  // DMA engine requests read from a specific buffer, RoCC provides data from that buffer
  // Assuming dma_ctrl_io.spad_read_addr is the address for the selected OFM buffer
  when(spad_read_source_is_ofm) {
    ofm_buffer.io.read_addr := dma_ctrl_io.spad_read_addr
    dma_ctrl_io.spad_read_data := ofm_buffer.io.read_data
  } .otherwise {
    // Default: if DMA is not reading OFM, or for other buffers if DMA also reads from IFM/Kernel (not typical for this setup)
    ofm_buffer.io.read_addr := 0.U // Or driven by CMD_GET_OFM_ADDR_DATA logic if that's still used
    dma_ctrl_io.spad_read_data := 0.S // Default, prevent latching stale data
  }
  // Note: if CMD_GET_OFM_ADDR_DATA is active, it would also drive ofm_buffer.io.read_addr.
  // An arbiter or careful state management for ofm_buffer.io.read_addr might be needed
  // if both DMA and RoCC direct commands can read OFM concurrently (not typical).
  // For now, assume DMA read and direct RoCC OFM read are mutually exclusive.

}
```

## File: src/main/scala/mycnnaccelerators/MyCNNRoCC.scala

```scala
// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy._
// import freechips.rocketchip.tilelink.TLIdentityNode // Not directly needed here with override

import CNNAcceleratorISA._ // Import ISA definitions

// Key for configuration
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey) // Get base config from Parameters

  // Instantiate the DMA LazyModule.
  // The AcceleratorConfig might need xLen, which is known in the ModuleImp.
  // SimpleDMAEngine takes accConfig.
  val dma_engine_lazy = LazyModule(new SimpleDMAEngine(baseConfig))

  // This RoCC will use the dma_engine_lazy's TileLink node as its primary memory master interface.
  // Override atlNode from CanHavePTW (mixed into LazyRoCC) or a similar node if defined by LazyRoCC.
  // This effectively makes the DMA the RoCC's connection to the memory system via TileLink.
  override val atlNode = dma_engine_lazy.node

  // The actual config with correct xLen is created in MyCNNRoCCModuleImp
  override lazy val module = new MyCNNRoCCModuleImp(this, baseConfig)
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

}
```

## File: src/main/scala/mycnnaccelerators/SimpleDMAEngine.scala

```scala
// filename: SimpleDMAEngine.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile.HasCoreParameters // For coreMaxAddrBits, xLen etc.
// It's good practice to ensure TileKey is available if p(TileKey) is used,
// but here we pass coreMaxAddrBits explicitly to the IO bundle.
// import freechips.rocketchip.tile.TileKey

import CNNAcceleratorISA._

// IO Bundle for the implementation part of the LazyModule
class SimpleDMATLModuleIO(
    val config: AcceleratorConfig,
    val coreMaxAddrBitsParam: Int // Explicitly pass the address width
)(implicit p: Parameters) extends Bundle {
  // Control from MyCNNRoCC (or a higher-level controller)
  val start = Input(Bool())
  val mem_base_addr = Input(UInt(coreMaxAddrBitsParam.W)) // Use passed width
  val length_bytes = Input(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  // The following line was causing: type mismatch; found: chisel3.UInt required: chisel3.ActualDirection
  // This usually means the Chisel compiler is confused, often due to other errors.
  // The syntax itself is correct. If the error persists after other fixes,
  // we might try renaming 'direction' or using a literal for the width to isolate.
  val directionBits = Input(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))
  
  val buffer_id = Input(UInt(CNNAcceleratorISA.dmaConfigBufIdBits.W))

  // Status to MyCNNRoCC
  val busy = Output(Bool())
  val done = Output(Bool())
  val dma_error = Output(Bool())

  // Scratchpad (Internal Buffer) Interface - driven by DMA to be connected by RoCC
  val spad_write_en = Output(Bool())
  val spad_write_addr = Output(UInt(32.W)) // Generic width, RoCC maps this to buffer
  val spad_write_data = Output(SInt(config.dataWidth.W))

  val spad_read_addr = Output(UInt(32.W))  // Generic width, RoCC maps this from buffer
  val spad_read_data = Input(SInt(config.dataWidth.W))
}

// SimpleDMAEngine as a LazyModule for TileLink integration
class SimpleDMAEngine(val accConfig: AcceleratorConfig, numOutstandingReqs: Int = 4)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "simple-dma-engine",
    sourceId = IdRange(0, numOutstandingReqs),
    requestFifo = true
  )))))

  lazy val module = new SimpleDMAEngineModuleImp(this)
}

class SimpleDMAEngineModuleImp(outer: SimpleDMAEngine)(implicit p: Parameters) extends LazyModuleImp(outer)
  with HasCoreParameters { // Provides xLen, coreMaxAddrBits, etc.

  // Instantiate the IO, passing the coreMaxAddrBits from HasCoreParameters
  val io = IO(new SimpleDMATLModuleIO(outer.accConfig, coreMaxAddrBits)(p))

  val (tl, edge) = outer.node.out.head

  val sIdle :: sTLRead_ReqAddr :: sTLRead_WaitData :: sTLRead_WriteToBuf :: sTLWrite_ReadFromBuf :: sTLWrite_ReqAddr :: sTLWrite_WaitAck :: sDone :: sError :: Nil = Enum(9)
  val state = RegInit(sIdle)

  val mem_base_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val length_bytes_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val direction_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))
  // val buffer_id_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigBufIdBits.W)) // DMA uses io.buffer_id directly

  val bytes_transferred_count = RegInit(0.U(CNNAcceleratorISA.dmaConfigLenBits.W))
  val current_mem_addr = Reg(UInt(coreMaxAddrBits.W))
  val current_spad_addr = Reg(UInt(32.W)) // Assuming spad addresses are local to each buffer concept

  val beatBytes = tl.a.bits.data.getWidth / 8
  // Ensure buffer_elems_per_beat is at least 1, and handle dataWidth being 0 (though unlikely for practical data)
  val buffer_elems_per_beat = if (outer.accConfig.dataWidth > 0) {
    val elems = beatBytes * 8 / outer.accConfig.dataWidth
    if (elems > 0) elems else 1
  } else {
    1
  }


  io.busy := (state =/= sIdle) && (state =/= sDone) && (state =/= sError)
  io.done := (state === sDone)
  io.dma_error := (state === sError)

  io.spad_write_en := false.B
  io.spad_write_addr := 0.U // Default, will be current_spad_addr in active states
  io.spad_write_data := 0.S
  io.spad_read_addr := 0.U  // Default, will be current_spad_addr in active states

  tl.a.valid := false.B
  tl.a.bits := DontCare
  tl.b.ready := true.B
  tl.c.valid := false.B
  tl.c.bits := DontCare
  tl.d.ready := false.B
  tl.e.valid := false.B
  tl.e.bits := DontCare

  val mem_data_beat_reg = Reg(UInt((beatBytes * 8).W))
  val spad_data_to_write_beat_reg = Reg(UInt((beatBytes*8).W)) // For BUF_TO_MEM
  // val sub_beat_counter = RegInit(0.U(log2Ceil(buffer_elems_per_beat + 1).W)) // May not be needed if writing full beat

  val dma_start_reg = RegNext(io.start, false.B) // Detect rising edge of start
  val dma_started_this_cycle = io.start && !dma_start_reg


  switch(state) {
    is(sIdle) {
      when(dma_started_this_cycle) { // Use registered start to avoid re-triggering if start held high
        mem_base_addr_reg := io.mem_base_addr
        length_bytes_reg := io.length_bytes
        direction_reg := io.directionBits
        // buffer_id_reg := io.buffer_id // Latch buffer_id for the current operation
        current_mem_addr := io.mem_base_addr
        current_spad_addr := 0.U
        bytes_transferred_count := 0.U
        // sub_beat_counter := 0.U // Reset if used
        mem_data_beat_reg := 0.U

        when(io.length_bytes === 0.U) {
          state := sDone
        } .elsewhen(io.directionBits === DMADirection.MEM_TO_BUF) {
          state := sTLRead_ReqAddr
        } .elsewhen(io.directionBits === DMADirection.BUF_TO_MEM) {
          state := sTLWrite_ReadFromBuf // Enable write path
        } .otherwise {
          state := sError
        }
      }
    }

    is(sTLRead_ReqAddr) {
      val (legal, get) = edge.Get(
        fromSource = 0.U, // Manage source IDs for multiple outstanding requests
        toAddress = current_mem_addr,
        lgSize = log2Ceil(beatBytes).U
      )
      when(legal) {
        tl.a.valid := true.B
        tl.a.bits := get
        when(tl.a.ready) {
          state := sTLRead_WaitData
        }
      } .otherwise {
        state := sError
      }
    }

    is(sTLRead_WaitData) {
      tl.d.ready := true.B
      when(tl.d.fire) {
        when(tl.d.bits.denied || tl.d.bits.corrupt) {
            state := sError
        } .otherwise {
            mem_data_beat_reg := tl.d.bits.data
            state := sTLRead_WriteToBuf
        }
      }
    }

    is(sTLRead_WriteToBuf) {
      // Simplified: write one beat from mem_data_beat_reg to spad.
      // Assumes dataWidth matches beatBytes*8 or RoCC handles sub-beat packing/unpacking.
      // For now, let's assume RoCC handles data element extraction from spad_write_data
      // if spad_write_data is wider than one element. Or DMA does packing if spad_write_data is one element.
      // The current SimpleDMATLModuleIO has spad_write_data as config.dataWidth.
      // This implies the DMA engine must break down the 'mem_data_beat_reg' (beatBytes*8 wide)
      // into 'config.dataWidth' chunks and write them sequentially to SPAD.
      // This adds complexity with a sub_beat_counter.
      // For simplicity, let's assume config.dataWidth IS beatBytes*8 for this example,
      // or that spad_write_data is a Vec.
      // Given current io.spad_write_data is SInt(config.dataWidth.W), DMA must iterate.
      // This part is not fully implemented here for brevity.
      // A common simplification: spad_write_data is beat-wide, RoCC unpacks.
      // Or, this state iterates for each element within the beat.
      // For now, we'll pretend one write is one element of config.dataWidth, and advance spad_addr by 1 element.
      // And assume beatBytes is one element. THIS IS A MAJOR SIMPLIFICATION.

      io.spad_write_en := true.B
      io.spad_write_addr := current_spad_addr
      // This assignment is problematic if beatBytes*8 != config.dataWidth
      // For now, taking the LSBs, assuming RoCC will handle it or config matches.
      io.spad_write_data := mem_data_beat_reg(outer.accConfig.dataWidth - 1, 0).asSInt

      // This state should ideally last for buffer_elems_per_beat cycles if dataWidth < beatBytes*8
      // current_spad_addr needs to increment per element written.
      // bytes_transferred_count increments per memory beat.

      // After one "write to buffer" cycle (could be one element or one full beat based on above simplification)
      val next_spad_addr = current_spad_addr + 1.U // Increment by 1 element address
      val next_bytes_transferred = bytes_transferred_count + beatBytes.U // One full beat from memory processed

      current_spad_addr := next_spad_addr
      // current_mem_addr should advance only when a full beat is consumed and a new one is needed.
      // If we are processing elements within a beat, mem_addr doesn't change yet.
      // The FSM implies one memory read maps to one spad write cycle.
      current_mem_addr := current_mem_addr + beatBytes.U
      bytes_transferred_count := next_bytes_transferred


      when(next_bytes_transferred >= length_bytes_reg) {
        state := sDone
      } .elsewhen (next_bytes_transferred < length_bytes_reg) {
        state := sTLRead_ReqAddr // Request next beat
      } .otherwise { // Should not happen if length_bytes_reg is multiple of beatBytes or handled
        state := sDone // Or error
      }
    }

    is(sTLWrite_ReadFromBuf) {
        // Read data from SPAD via io.spad_read_data
        // This state needs to assert io.spad_read_addr
        io.spad_read_addr := current_spad_addr
        // Data will be available on io.spad_read_data on the next cycle (combinatorial read from buffer + RegNext in MinimalBuffer)
        // Or if buffer is sync read, it takes a cycle. Assume MinimalBuffer's RegNext makes it available "next cycle" relative to addr.
        // For TL Put, we need to prepare data for tl.a.bits.data.
        // This might require a cycle to read from spad and then a cycle to send.

        // Let's assume data is read and available in the same cycle for simplicity of FSM state count,
        // or that MyCNNRoCCModuleImp ensures data is ready for spad_read_data when DMA wants it.
        // A more realistic FSM would have: sTLWrite_SetSpadAddr, sTLWrite_LatchSpadData, then sTLWrite_ReqAddr.
        // For now, assume io.spad_read_data is valid when this state is entered (after current_spad_addr is set).

        // This simplified FSM will take spad_read_data and directly try to send it.
        // This again assumes spad_read_data provides a full beatBytes chunk.
        // If config.dataWidth < beatBytes*8, packing logic is needed here.
        // Taking LSBs for now.
        spad_data_to_write_beat_reg := io.spad_read_data.asUInt // Zero-extend if narrower

        state := sTLWrite_ReqAddr
    }

    is(sTLWrite_ReqAddr) {
        // Prepare a TileLink PutFullData message (write request)
        val (legal, put) = edge.Put(
            fromSource = 0.U,
            toAddress = current_mem_addr,
            lgSize = log2Ceil(beatBytes).U,
            data = spad_data_to_write_beat_reg // Assumes spad_data_to_write_beat_reg is prepared and beatBytes wide
        )
        // For PutPartialData: edge.Put(..., mask = ...)

        when(legal) {
            tl.a.valid := true.B
            tl.a.bits := put
            when(tl.a.ready) { // TL slave accepts the request
                // For PutFullData, data is on channel A. Slave grants on D.
                state := sTLWrite_WaitAck
            }
        } .otherwise {
            state := sError
        }
    }

    is(sTLWrite_WaitAck) {
        // Wait for Grant/Ack on Channel D
        tl.d.ready := true.B // We are ready to accept the grant/response
        when(tl.d.fire) {
            // Check tl.d.bits.denied or tl.d.bits.corrupt for errors
            when(tl.d.bits.denied || tl.d.bits.corrupt) { // Check for errors from slave
                state := sError
            } .otherwise {
                // Write successful for this beat
                val next_spad_addr = current_spad_addr + 1.U // Increment by 1 element address
                val next_bytes_transferred = bytes_transferred_count + beatBytes.U

                current_spad_addr := next_spad_addr
                current_mem_addr := current_mem_addr + beatBytes.U
                bytes_transferred_count := next_bytes_transferred

                when(next_bytes_transferred >= length_bytes_reg) {
                    state := sDone
                } .elsewhen (next_bytes_transferred < length_bytes_reg) {
                    state := sTLWrite_ReadFromBuf // Read next data from SPAD
                } .otherwise {
                     state := sDone // Or error
                }
            }
        }
    }

    is(sDone) {
      // io.start is a level signal. Wait for it to go low before returning to sIdle,
      // or ensure controller only pulses start.
      // If start is kept high, we might immediately re-start.
      // Using dma_started_this_cycle for sIdle transition handles this.
      state := sIdle
    }
    is(sError) {
      state := sIdle // Or a persistent error state needing explicit clear
    }
  }
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
  // Existing Funct values
  val CMD_SET_IFM_ADDR_DATA    = 0.U(7.W)
  val CMD_SET_KERNEL_ADDR_DATA = 1.U(7.W)
  val CMD_START_COMPUTE        = 2.U(7.W)
  val CMD_GET_OFM_ADDR_DATA    = 3.U(7.W)
  val CMD_GET_STATUS           = 4.U(7.W)

  // New DMA Funct values
  val CMD_DMA_CONFIG_ADDR      = 5.U(7.W) // rs1: main memory base address
  val CMD_DMA_CONFIG_PARAMS    = 6.U(7.W) // rs1: Cat(length_in_bytes (24 bits max suggested), direction (1 bit), buffer_id (2 bits))
  val CMD_DMA_START            = 7.U(7.W) // No operands needed, uses configured values

  // Existing Status codes
  val STATUS_IDLE            = 0.U(8.W)
  val STATUS_LOADING_IFM     = 1.U(8.W) // Will be replaced by DMA status
  val STATUS_LOADING_KERNEL  = 2.U(8.W) // Will be replaced by DMA status
  val STATUS_COMPUTING       = 3.U(8.W)
  val STATUS_COMPUTE_DONE    = 4.U(8.W)
  // val STATUS_ERROR        = 255.U(8.W) // Standard error

  // New DMA Status codes
  val STATUS_DMA_BUSY              = 10.U(8.W) // General DMA busy
  val STATUS_DMA_CONFIG_READY    = 11.U(8.W) // DMA configured, ready to start
  val STATUS_DMA_IFM_LOAD_DONE   = 12.U(8.W)
  val STATUS_DMA_KERNEL_LOAD_DONE= 13.U(8.W)
  val STATUS_DMA_OFM_STORE_DONE  = 14.U(8.W)
  val STATUS_DMA_ERROR           = 254.U(8.W) // DMA specific error
  val STATUS_ERROR               = 255.U(8.W) // General error (keep)


  // Helper objects for DMA configuration
  object BufferIDs {
    val IFM    = 0.U(2.W)
    val KERNEL = 1.U(2.W)
    val OFM    = 2.U(2.W)
    // val MAX_ID = 2.U // For validation if needed
  }

  object DMADirection {
    val MEM_TO_BUF = 0.U(1.W) // Read from Main Memory, Write to Accelerator Buffer
    val BUF_TO_MEM = 1.U(1.W) // Read from Accelerator Buffer, Write to Main Memory
  }

  // Bit widths for packing DMA_CONFIG_PARAMS into rs1 of CMD_DMA_CONFIG_PARAMS
  // Assuming rs1 is xLen, but RoCC commands typically use general purpose registers (e.g., 64-bit)
  // Let's define based on a common register width, e.g., 64.
  // If xLen can be 32, then this packing needs to be conditional or more constrained.
  // For now, assume parameters fit.
  val dmaConfigLenBits    = 24 // Allows up to 16MB transfers
  val dmaConfigDirBits    = 1
  val dmaConfigBufIdBits  = 2
  val dmaConfigTotalParamBits = dmaConfigLenBits + dmaConfigDirBits + dmaConfigBufIdBits
}
```

## File: src/test/scala/mycnnaccelerators/ComputeUnitTest.scala

```scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import chisel3.simulator.EphemeralSimulator._
import org.scalatest.flatspec.AnyFlatSpec
// Please do not use chiseltest has been archived and is not recommended for new projects.

case class AcceleratorConfig(
  dataWidth: Int,
  ifmRows: Int, ifmCols: Int,
  kernelRows: Int, kernelCols: Int,
  xLen: Int
) {
  val ofmRows: Int = ifmRows - kernelRows + 1
  val ofmCols: Int = ifmCols - kernelCols + 1
}

class ComputeUnit(config: AcceleratorConfig) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val done = Output(Bool())
    val busy = Output(Bool())
    val ifm_read_addr = Output(UInt(log2Ceil(config.ifmRows * config.ifmCols).W))
    val ifm_read_data = Input(SInt(config.dataWidth.W))
    val kernel_read_addr = Output(UInt(log2Ceil(config.kernelRows * config.kernelCols).W))
    val kernel_read_data = Input(SInt(config.dataWidth.W))
    val ofm_write_en = Output(Bool())
    val ofm_write_addr = Output(UInt(log2Ceil(config.ofmRows * config.ofmCols).W))
    val ofm_write_data = Output(SInt(config.dataWidth.W))
  })

  val s_idle :: s_busy :: s_done :: Nil = chisel3.util.Enum(3)
  val state = RegInit(s_idle)

  io.busy := (state === s_busy)
  io.done := (state === s_done)

  io.ifm_read_addr := 0.U
  io.kernel_read_addr := 0.U
  io.ofm_write_en := false.B
  io.ofm_write_addr := 0.U
  io.ofm_write_data := 0.S

  switch(state) {
    is(s_idle) {
      when(io.start) { state := s_busy }
    }
    is(s_busy) {
      when(RegNext(io.start)) { state := s_done }
    }
    is(s_done) { /* Stays done */ }
  }
}


class ComputeUnitTest extends AnyFlatSpec {

  val F_BITS_CU = 8
  def toFixed(d: Double): BigInt = (d * (1L << F_BITS_CU)).round
  def toDouble(sIntValue: BigInt): Double = {
    sIntValue.doubleValue / (1L << F_BITS_CU)
  }

  def referenceConv(
    ifm: Seq[Seq[Double]],
    kernel: Seq[Seq[Double]],
    config: AcceleratorConfig
  ): Seq[Seq[Double]] = {
    val ofm = Array.ofDim[Double](config.ofmRows, config.ofmCols)
    val ACC_FRAC_BITS = config.dataWidth
    for (r_ofm <- 0 until config.ofmRows) {
      for (c_ofm <- 0 until config.ofmCols) {
        var accumulatorDouble: Double = 0.0
        for (r_k <- 0 until config.kernelRows) {
          for (c_k <- 0 until config.kernelCols) {
            val r_ifm = r_ofm + r_k
            val c_ifm = c_ofm + c_k
            if (r_ifm < config.ifmRows && c_ifm < config.ifmCols) {
                 accumulatorDouble += ifm(r_ifm)(c_ifm) * kernel(r_k)(c_k)
            }
          }
        }
        val accFixedInternal = (accumulatorDouble * (1L << ACC_FRAC_BITS)).round
        val shiftAmount = ACC_FRAC_BITS - F_BITS_CU
        val shiftedAcc = if (shiftAmount >= 0) accFixedInternal >> shiftAmount else accFixedInternal << -shiftAmount
        var finalSIntValue = shiftedAcc & ((BigInt(1) << config.dataWidth) - 1)
        if ((finalSIntValue & (BigInt(1) << (config.dataWidth - 1))) != 0) {
          finalSIntValue = finalSIntValue - (BigInt(1) << config.dataWidth)
        }
        ofm(r_ofm)(c_ofm) = toDouble(finalSIntValue)
      }
    }
    ofm.map(_.toSeq).toSeq
  }

  behavior of "ComputeUnit"

  it should "perform a 3x3 IFM, 2x2 Kernel convolution correctly" in {
    val testConfig = AcceleratorConfig(
      dataWidth = 16,
      ifmRows = 3, ifmCols = 3,
      kernelRows = 2, kernelCols = 2,
      xLen = 64
    )

    val ifm_double = Seq(Seq(1.0, 2.0, 0.5), Seq(3.0, 1.5, 1.0), Seq(0.25, 2.5, 0.75))
    val ifm_fixed_flat = ifm_double.flatten.map(d => toFixed(d)).toArray
    val kernel_double = Seq(Seq(0.5, -1.0), Seq(1.0, 0.25))
    val kernel_fixed_flat = kernel_double.flatten.map(d => toFixed(d)).toArray
    val expected_ofm_double = referenceConv(ifm_double, kernel_double, testConfig)

    simulate(new ComputeUnit(testConfig)) { dut =>
      dut.io.start.poke(false)
      dut.io.ifm_read_data.poke(BigInt(0))
      dut.io.kernel_read_data.poke(BigInt(0))
      dut.clock.step(1)

      dut.io.start.poke(true)
      dut.clock.step(1)
      dut.io.start.poke(false)

      val maxCycles = testConfig.ifmRows * testConfig.ifmCols * testConfig.kernelRows * testConfig.kernelCols *
                      testConfig.ofmRows * testConfig.ofmCols + 200
      var cycles = 0
      val actual_ofm_map = collection.mutable.Map[(Int, Int), BigInt]()
      var prev_ifm_addr: Option[Int] = None
      var prev_kernel_addr: Option[Int] = None

      println("Starting ChiselSim test loop...")
      // Corrected: API for peeking values with EphemeralSimulator
      // For Bool: .peek().litValue == BigInt(1) (or != 0.BigInt)
      // For UInt/SInt: .peek().litValue
      while (dut.io.done.peek().litValue != BigInt(1) && cycles < maxCycles) {
        prev_ifm_addr match {
          case Some(addr) if addr >= 0 && addr < ifm_fixed_flat.length =>
            dut.io.ifm_read_data.poke(ifm_fixed_flat(addr))
          case _ => dut.io.ifm_read_data.poke(BigInt(0))
        }
        prev_kernel_addr match {
          case Some(addr) if addr >= 0 && addr < kernel_fixed_flat.length =>
            dut.io.kernel_read_data.poke(kernel_fixed_flat(addr))
          case _ => dut.io.kernel_read_data.poke(BigInt(0))
        }

        if (dut.io.ofm_write_en.peek().litValue == BigInt(1)) {
          val ofm_addr = dut.io.ofm_write_addr.peek().litValue.toInt
          val ofm_data_raw = dut.io.ofm_write_data.peek().litValue
          
          val r_ofm = ofm_addr / testConfig.ofmCols
          val c_ofm = ofm_addr % testConfig.ofmCols
          actual_ofm_map((r_ofm, c_ofm)) = ofm_data_raw
        }

        dut.clock.step(1)

        prev_ifm_addr = Some(dut.io.ifm_read_addr.peek().litValue.toInt)
        prev_kernel_addr = Some(dut.io.kernel_read_addr.peek().litValue.toInt)
        
        cycles += 1
      }

      println(s"ChiselSim simulation finished in $cycles cycles.")
      assert(cycles < maxCycles, "Simulation timed out.")
      assert(dut.io.done.peek().litValue == BigInt(1), "DUT did not signal done at the end.")

      println("Verifying OFM results:")
      var mismatches = 0
      for (r <- 0 until testConfig.ofmRows) {
        for (c <- 0 until testConfig.ofmCols) {
          val actual_fixed = actual_ofm_map.getOrElse((r, c), BigInt(0))
          val actual_double = toDouble(actual_fixed)
          val expected_double = expected_ofm_double(r)(c)
          print(f"OFM($r%d,$c%d): Actual=${actual_double}%6.4f (Fixed:$actual_fixed%d), Expected=${expected_double}%6.4f. ")
          val tolerance = 1.0 / (1L << (F_BITS_CU - 1))
          if (Math.abs(actual_double - expected_double) < tolerance) {
            println("Match!")
          } else {
            println(f"Mismatch! Diff: ${Math.abs(actual_double - expected_double)}%.4f")
            mismatches +=1
          }
        }
      }
      assert(mismatches == 0, s"$mismatches OFM value mismatches found.")
    }
  }
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

