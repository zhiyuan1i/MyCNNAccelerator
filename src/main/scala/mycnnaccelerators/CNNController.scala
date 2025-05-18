// filename: CNNController.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.RoCCCommand
import CNNAcceleratorISA._
import org.chipsalliance.cde.config.Parameters

// CNNControllerIO now takes the config object directly
class CNNControllerIO(val config: AcceleratorConfig) extends Bundle {
  // RoCC Command Interface (from CPU)
  val cmd_in = Flipped(Decoupled(new RoCCCommand()(Parameters.empty)))

  // RoCC Response Interface (to CPU)
  val cmd_resp_data_out  = Output(UInt(config.xLen.W)) // Use xLen from config
  val cmd_resp_valid_out = Output(Bool())
  val cmd_resp_rd_out    = Output(UInt(5.W))

  // Accelerator Busy Signal (to RoCC top level)
  val busy_out = Output(Bool())

  // DMA Control Interface
  val dma_req_out     = Decoupled(new DMARequest(config.coreMaxAddrBits, log2Ceil(config.ifmDepth.max(config.kernelMaxDepth).max(config.ofmDepth))))
  val dma_busy_in     = Input(Bool())
  val dma_done_in     = Input(Bool())
  val dma_error_in    = Input(Bool())

  // Compute Unit Control Interface
  val compute_start_out      = Output(Bool())
  val compute_kernel_dim_out = Output(UInt(log2Ceil(config.kernelMaxRows + 1).W))
  val compute_busy_in        = Input(Bool())
  val compute_done_in        = Input(Bool())
  val compute_error_in       = Input(Bool())
}

// CNNController now takes the config object which includes xLen
class CNNController(val config: AcceleratorConfig)(implicit p: Parameters) extends Module {
  val io = IO(new CNNControllerIO(config)) // Pass the config object to the IO bundle

  // FSM States
  val sIdle :: sDmaLoadIFM_Setup :: sDmaLoadIFM_Busy :: sDmaLoadKernel_Setup :: sDmaLoadKernel_Busy :: sCompute_Start :: sCompute_Busy :: sDmaStoreOFM_Setup :: sDmaStoreOFM_Busy :: sRespondDone :: sRespondError :: Nil = Enum(11)
  val state = RegInit(sIdle)

  // Configuration Registers
  val ifm_main_mem_addr_reg    = Reg(UInt(config.coreMaxAddrBits.W))
  val kernel_main_mem_addr_reg = Reg(UInt(config.coreMaxAddrBits.W))
  val kernel_dim_reg           = Reg(UInt(log2Ceil(config.kernelMaxRows + 1).W))
  val ofm_main_mem_addr_reg    = Reg(UInt(config.coreMaxAddrBits.W))

  val cfg_ifm_loaded    = RegInit(false.B)
  val cfg_kernel_loaded = RegInit(false.B)
  val cfg_ofm_loaded    = RegInit(false.B)
  val all_configs_loaded = cfg_ifm_loaded && cfg_kernel_loaded && cfg_ofm_loaded

  val accelerator_status_reg = RegInit(STATUS_IDLE)
  val rd_for_resp_reg        = Reg(UInt(5.W))

  // Default outputs
  io.cmd_in.ready := (state === sIdle && !io.cmd_resp_valid_out)
  io.busy_out     := (state =/= sIdle) || io.cmd_resp_valid_out

  io.cmd_resp_valid_out := false.B
  io.cmd_resp_data_out  := accelerator_status_reg
  io.cmd_resp_rd_out    := rd_for_resp_reg

  io.dma_req_out.valid := false.B
  io.dma_req_out.bits  := DontCare

  io.compute_start_out      := false.B
  io.compute_kernel_dim_out := kernel_dim_reg

  val fixed_point_element_bytes = config.ifmDataType.dataWidth / 8

  switch(state) {
    is(sIdle) {
      accelerator_status_reg := STATUS_IDLE
      when(io.cmd_in.valid) {
        val funct = io.cmd_in.bits.inst.funct
        val rs1_val = io.cmd_in.bits.rs1
        val rs2_val = io.cmd_in.bits.rs2
        rd_for_resp_reg := io.cmd_in.bits.inst.rd

        io.cmd_in.ready := false.B

        when(funct === CONFIG_IFM_ADDR) {
          ifm_main_mem_addr_reg := rs1_val
          cfg_ifm_loaded := true.B
          accelerator_status_reg := STATUS_IDLE
          io.cmd_resp_valid_out := true.B
        }.elsewhen(funct === CONFIG_KERNEL_ADDR_SIZE) {
          kernel_main_mem_addr_reg := rs1_val
          kernel_dim_reg := rs2_val(log2Ceil(config.kernelMaxRows + 1) -1, 0)
          cfg_kernel_loaded := true.B
          accelerator_status_reg := STATUS_IDLE
          io.cmd_resp_valid_out := true.B
        }.elsewhen(funct === CONFIG_OFM_ADDR_PARAMS) {
          ofm_main_mem_addr_reg := rs1_val
          cfg_ofm_loaded := true.B
          accelerator_status_reg := STATUS_IDLE
          io.cmd_resp_valid_out := true.B
        }.elsewhen(funct === START_CONVOLUTION) {
          when(all_configs_loaded) {
            accelerator_status_reg := STATUS_BUSY_DMA_IFM
            state := sDmaLoadIFM_Setup
          }.otherwise {
            accelerator_status_reg := STATUS_ERROR_CONFIG
            io.cmd_resp_valid_out := true.B
            cfg_ifm_loaded := false.B; cfg_kernel_loaded := false.B; cfg_ofm_loaded := false.B
          }
        }.elsewhen(funct === GET_STATUS) {
          io.cmd_resp_data_out := accelerator_status_reg
          io.cmd_resp_valid_out := true.B
        } .otherwise {
            accelerator_status_reg := STATUS_ERROR_CONFIG
            io.cmd_resp_valid_out := true.B
        }
      }
    }

    is(sDmaLoadIFM_Setup) {
      io.dma_req_out.valid := true.B
      io.dma_req_out.bits.main_mem_addr := ifm_main_mem_addr_reg
      io.dma_req_out.bits.spad_base_addr := 0.U
      io.dma_req_out.bits.length_bytes := (config.ifmRows * config.ifmCols * fixed_point_element_bytes).U
      io.dma_req_out.bits.is_write_to_main_mem := false.B
      io.dma_req_out.bits.spad_target_is_ifm := true.B
      io.dma_req_out.bits.spad_target_is_kernel := false.B

      when(io.dma_req_out.fire) {
        state := sDmaLoadIFM_Busy
        accelerator_status_reg := STATUS_BUSY_DMA_IFM
      }
    }
    is(sDmaLoadIFM_Busy) {
      when(io.dma_done_in) {
        state := sDmaLoadKernel_Setup
        accelerator_status_reg := STATUS_BUSY_DMA_KERNEL
      }
      when(io.dma_error_in) {
        accelerator_status_reg := STATUS_ERROR_DMA
        state := sRespondError
      }
    }

    is(sDmaLoadKernel_Setup) {
      io.dma_req_out.valid := true.B
      io.dma_req_out.bits.main_mem_addr := kernel_main_mem_addr_reg
      io.dma_req_out.bits.spad_base_addr := 0.U
      io.dma_req_out.bits.length_bytes := kernel_dim_reg * kernel_dim_reg * fixed_point_element_bytes.U
      io.dma_req_out.bits.is_write_to_main_mem := false.B
      io.dma_req_out.bits.spad_target_is_ifm := false.B
      io.dma_req_out.bits.spad_target_is_kernel := true.B

      when(io.dma_req_out.fire) {
        state := sDmaLoadKernel_Busy
        accelerator_status_reg := STATUS_BUSY_DMA_KERNEL
      }
    }
    is(sDmaLoadKernel_Busy) {
      when(io.dma_done_in) {
        state := sCompute_Start
        accelerator_status_reg := STATUS_BUSY_COMPUTE
      }
      when(io.dma_error_in) {
        accelerator_status_reg := STATUS_ERROR_DMA
        state := sRespondError
      }
    }

    is(sCompute_Start) {
      io.compute_start_out := true.B
      state := sCompute_Busy
    }
    is(sCompute_Busy) {
      io.compute_start_out := false.B
      when(io.compute_done_in) {
        state := sDmaStoreOFM_Setup
        accelerator_status_reg := STATUS_BUSY_DMA_OFM
      }
      when(io.compute_error_in) {
        accelerator_status_reg := STATUS_ERROR_COMPUTE
        state := sRespondError
      }
    }

    is(sDmaStoreOFM_Setup) {
      io.dma_req_out.valid := true.B
      io.dma_req_out.bits.main_mem_addr := ofm_main_mem_addr_reg
      io.dma_req_out.bits.spad_base_addr := 0.U
      io.dma_req_out.bits.length_bytes := (config.ofmRows * config.ofmCols * fixed_point_element_bytes).U
      io.dma_req_out.bits.is_write_to_main_mem := true.B
      io.dma_req_out.bits.spad_target_is_ifm := false.B
      io.dma_req_out.bits.spad_target_is_kernel := false.B

      when(io.dma_req_out.fire) {
        state := sDmaStoreOFM_Busy
        accelerator_status_reg := STATUS_BUSY_DMA_OFM
      }
    }
    is(sDmaStoreOFM_Busy) {
      when(io.dma_done_in) {
        accelerator_status_reg := STATUS_DONE_SUCCESS
        state := sRespondDone
      }
      when(io.dma_error_in) {
        accelerator_status_reg := STATUS_ERROR_DMA
        state := sRespondError
      }
    }

    is(sRespondDone) {
      cfg_ifm_loaded := false.B
      cfg_kernel_loaded := false.B
      cfg_ofm_loaded := false.B
      state := sIdle
    }
    is(sRespondError) {
      cfg_ifm_loaded := false.B
      cfg_kernel_loaded := false.B
      cfg_ofm_loaded := false.B
      state := sIdle
    }
  }

  when(io.cmd_resp_valid_out) {
    io.cmd_in.ready := false.B
  }
}
