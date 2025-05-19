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