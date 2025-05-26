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

  // Debug print enable flag
  val enableRoccDebugPrints = true.B // Set to false.B to disable RoCC debug prints

  val roccCycleCount = RegInit(0.U(32.W))
  when(enableRoccDebugPrints) {
    roccCycleCount := roccCycleCount + 1.U
  }

  // Instantiate internal modules
  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  // Access the module of the DMA engine defined in the outer MyCNNRoCC
  val dma_ctrl_io = outer.dma_engine_lazy.module.io

  // RoCC FSM states
  val sIdle :: sWaitOFMRead :: sRespond :: sDmaActive :: Nil = Enum(4)
  val rocc_state = RegInit(sIdle)
  val rocc_state_prev = RegNext(rocc_state, sIdle) // For detecting state changes

  // Registers for RoCC interaction
  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  // Accelerator status and DMA configuration registers
  val accelerator_status_reg = RegInit(STATUS_IDLE)
  val acc_status_prev = RegNext(accelerator_status_reg, STATUS_IDLE) // For detecting status changes

  val dma_mem_base_addr_reg = Reg(UInt(xLen.W))
  val dma_length_bytes_reg = Reg(UInt(dmaConfigLenBits.W))
  val dma_direction_reg = Reg(UInt(dmaConfigDirBits.W))
  val dma_buffer_id_reg = Reg(UInt(dmaConfigBufIdBits.W))
  val dma_configured_flag = RegInit(false.B)
  val dma_addr_configured_internal_flag = RegInit(false.B)

  // Operational flags
  val ifm_loaded_flag = RegInit(false.B)
  val kernel_loaded_flag = RegInit(false.B)
  val compute_completed_flag = RegInit(false.B)

  // Previous states of flags for debug printing
  val ifm_loaded_flag_prev = RegNext(ifm_loaded_flag, false.B)
  val kernel_loaded_flag_prev = RegNext(kernel_loaded_flag, false.B)
  val compute_completed_flag_prev = RegNext(compute_completed_flag, false.B)
  val dma_configured_flag_prev = RegNext(dma_configured_flag, false.B)


  io.cmd.ready := (rocc_state === sIdle) && !resp_valid_reg && !dma_ctrl_io.busy && !compute_unit.io.busy
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  // RoCC is busy if CU is busy, DMA is busy, or FSM is not sIdle, or it's currently trying to send a response.
  io.busy := (compute_unit.io.busy || dma_ctrl_io.busy || (rocc_state =/= sIdle) || resp_valid_reg)
  io.interrupt := false.B

  // Default assignments for buffer writes (mostly controlled by DMA or ComputeUnit)
  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.S(config.dataWidth.W)

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.S(config.dataWidth.W)

  // OFM buffer is written by ComputeUnit
  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data
  ofm_buffer.io.read_addr := 0.U // Read address for OFM typically driven by DMA

  // ComputeUnit connections
  compute_unit.io.start := false.B // Controlled by FSM
  compute_unit.io.ifm_read_data   := ifm_buffer.io.read_data
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data
  ifm_buffer.io.read_addr       := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr      := compute_unit.io.kernel_read_addr

  // DMA controller connections
  dma_ctrl_io.start := false.B // Controlled by FSM
  dma_ctrl_io.mem_base_addr := dma_mem_base_addr_reg
  dma_ctrl_io.length_bytes := dma_length_bytes_reg
  dma_ctrl_io.directionBits := dma_direction_reg
  dma_ctrl_io.buffer_id := dma_buffer_id_reg

  // Logic for handling response acknowledgement from CPU
  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B // Clear valid once CPU takes it
    if (enableRoccDebugPrints.litToBoolean) {
      printf(p"RoCC Cycle[${roccCycleCount}]: CPU took response. rd=${io.resp.bits.rd}, data=0x${Hexadecimal(io.resp.bits.data)}\n")
    }
  }

  // Debug prints for state and status changes
  if (enableRoccDebugPrints.litToBoolean) {
    when(rocc_state =/= rocc_state_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: FSM State Change: ${rocc_state_prev} -> ${rocc_state}\n")
    }
    when(accelerator_status_reg =/= acc_status_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: Accelerator Status Change: ${acc_status_prev} -> ${accelerator_status_reg}\n")
    }
    when(ifm_loaded_flag =/= ifm_loaded_flag_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: ifm_loaded_flag changed to ${ifm_loaded_flag}\n")
    }
    when(kernel_loaded_flag =/= kernel_loaded_flag_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: kernel_loaded_flag changed to ${kernel_loaded_flag}\n")
    }
    when(compute_completed_flag =/= compute_completed_flag_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: compute_completed_flag changed to ${compute_completed_flag}\n")
    }
    when(dma_configured_flag =/= dma_configured_flag_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: dma_configured_flag changed to ${dma_configured_flag}\n")
    }
    when(io.cmd.fire) { // Log received command
        printf(p"RoCC Cycle[${roccCycleCount}]: CMD Received: funct=${io.cmd.bits.inst.funct}, rs1=0x${Hexadecimal(io.cmd.bits.rs1)}, rs2=0x${Hexadecimal(io.cmd.bits.rs2)}, rd=${io.cmd.bits.inst.rd}\n")
    }
    // More specific debugs for start signals
    val dma_start_asserted_for_debug = RegNext(dma_ctrl_io.start, false.B)
    when(dma_ctrl_io.start && !dma_start_asserted_for_debug) { // Print only on rising edge
        printf(p"RoCC Cycle[${roccCycleCount}]: DMA Start (dma_ctrl_io.start) asserted by RoCC. BufID=${dma_buffer_id_reg}, Dir=${dma_direction_reg}, Len=${dma_length_bytes_reg}, Addr=0x${Hexadecimal(dma_mem_base_addr_reg)}\n")
    }
    val compute_start_asserted_for_debug = RegNext(compute_unit.io.start, false.B)
    when(compute_unit.io.start && !compute_start_asserted_for_debug) { // Print only on rising edge
        printf(p"RoCC Cycle[${roccCycleCount}]: Compute Unit Start (compute_unit.io.start) asserted by RoCC.\n")
    }
  }


  // Main RoCC FSM
  switch(rocc_state) {
    is(sIdle) {
      // Check for asynchronous events completion (like compute unit finishing)
      when(accelerator_status_reg === STATUS_COMPUTING && compute_unit.io.done && !compute_unit.io.busy) {
        if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: Compute unit finished (detected in sIdle). Setting STATUS_COMPUTE_DONE.\n")
        }
        accelerator_status_reg := STATUS_COMPUTE_DONE
        compute_completed_flag := true.B
      }

      // If DMA was started (status is DMA_BUSY) and the DMA controller is indeed busy, RoCC should be in sDmaActive
      when(accelerator_status_reg === STATUS_DMA_BUSY && dma_ctrl_io.busy) {
         if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: DMA is busy and status is DMA_BUSY, RoCC FSM transitioning from sIdle to sDmaActive.\n")
         }
         rocc_state := sDmaActive
      }

      // Process incoming RoCC command
      when(io.cmd.valid && !resp_valid_reg) { // Ensure not already processing a response
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd // Capture rd for the response

        resp_data_reg := STATUS_ERROR.asUInt // Default response data
        resp_valid_reg := true.B             // Default to send a response
        rocc_state := sRespond               // Default next state to handle CPU taking the response.

        if (enableRoccDebugPrints.litToBoolean) {
            // This Chisel 'when' will create hardware that prints only if cmd.inst.funct is CMD_START_COMPUTE
            when(cmd.inst.funct === CMD_START_COMPUTE) { // io.cmd.valid is true here
              printf(p"RoCC Cycle[${roccCycleCount}]: sIdle: Attempting CMD_START_COMPUTE. For io.cmd.ready check: rocc_state_is_sIdle=${rocc_state === sIdle}, not_resp_valid_reg=${!resp_valid_reg}, dma_busy=${dma_ctrl_io.busy}, cu_busy=${compute_unit.io.busy}, ifm_loaded=${ifm_loaded_flag}, kernel_loaded=${kernel_loaded_flag}\n")
            }
        }
        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_IFM_ADDR_DATA (funct=${cmd.inst.funct}) - Not Implemented. Responding with STATUS_ERROR.\n") }
            // resp_data_reg is already STATUS_ERROR
          }
          is(CMD_SET_KERNEL_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_KERNEL_ADDR_DATA (funct=${cmd.inst.funct}) - Not Implemented. Responding with STATUS_ERROR.\n") }
            // resp_data_reg is already STATUS_ERROR
          }
          is(CMD_GET_OFM_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_GET_OFM_ADDR_DATA (funct=${cmd.inst.funct}) - Not Implemented. Responding with STATUS_ERROR.\n") }
            // resp_data_reg is already STATUS_ERROR
          }

          is(CMD_START_COMPUTE) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: sIdle: Processing CMD_START_COMPUTE case. Actual values: IFM_L=${ifm_loaded_flag}, KERN_L=${kernel_loaded_flag}, CU_Busy=${compute_unit.io.busy}, DMA_Busy=${dma_ctrl_io.busy}\n")}
            when(ifm_loaded_flag && kernel_loaded_flag && !compute_unit.io.busy && !dma_ctrl_io.busy) {
              compute_unit.io.start := true.B
              accelerator_status_reg := STATUS_COMPUTING
              compute_completed_flag := false.B
              resp_data_reg := STATUS_COMPUTING.asUInt // Use the constant value for immediate response
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_START_COMPUTE: Conditions met. Starting compute. Responding with ACK (${STATUS_COMPUTING}).\n")}
            } .otherwise {
              // resp_data_reg is already STATUS_ERROR (255) by default (set before the switch)
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_START_COMPUTE: Error - preconditions not met. Responding with STATUS_ERROR.\n")}
            }
          }
          is(CMD_DMA_CONFIG_ADDR) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: Processing CMD_DMA_CONFIG_ADDR. Addr=0x${Hexadecimal(cmd.rs1)}\n")}
            dma_mem_base_addr_reg := cmd.rs1
            dma_addr_configured_internal_flag := true.B
            dma_configured_flag := false.B
            accelerator_status_reg := STATUS_IDLE
            resp_data_reg := accelerator_status_reg
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_ADDR: Configured. Responding with ACK (${accelerator_status_reg}).\n")}
          }
          is(CMD_DMA_CONFIG_PARAMS) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: Processing CMD_DMA_CONFIG_PARAMS. Params=0x${Hexadecimal(cmd.rs1)}. AddrInternalFlag=${dma_addr_configured_internal_flag}\n")}
            when(dma_addr_configured_internal_flag) {
              val param_val = cmd.rs1
              dma_buffer_id_reg := param_val(dmaConfigBufIdBits - 1, 0)
              dma_direction_reg := param_val(dmaConfigBufIdBits)
              dma_length_bytes_reg := param_val(dmaConfigTotalParamBits - 1, dmaConfigBufIdBits + dmaConfigDirBits)
              dma_configured_flag := true.B
              dma_addr_configured_internal_flag := false.B
              accelerator_status_reg := STATUS_DMA_CONFIG_READY
              resp_data_reg := accelerator_status_reg
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_PARAMS: Parsed. BufID=${dma_buffer_id_reg}, Dir=${dma_direction_reg}, Len=${dma_length_bytes_reg}. Status=${accelerator_status_reg}. Responding with ACK.\n")}
            } .otherwise {
              accelerator_status_reg := STATUS_DMA_ERROR
              resp_data_reg := STATUS_DMA_ERROR.asUInt
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_PARAMS: Error - ADDR not configured. Responding with STATUS_DMA_ERROR.\n")}
            }
          }
          is(CMD_DMA_START) {
            val is_ofm_dma_write = (dma_direction_reg === DMADirection.BUF_TO_MEM && dma_buffer_id_reg === BufferIDs.OFM)
            val common_dma_start_conditions = dma_configured_flag && !dma_ctrl_io.busy && !compute_unit.io.busy
            if (enableRoccDebugPrints.litToBoolean) {
                printf(p"RoCC Cycle[${roccCycleCount}]: Processing CMD_DMA_START. IsOFMWrite=${is_ofm_dma_write}, DMAConf=${dma_configured_flag}, CompDone(ifOFM)=${compute_completed_flag}, DMABusy=${dma_ctrl_io.busy}, CUBusy=${compute_unit.io.busy}\n")
            }

            val can_start_dma = WireDefault(false.B)
            when(common_dma_start_conditions) {
              when(is_ofm_dma_write) {
                when(compute_completed_flag) {
                  can_start_dma := true.B
                } .otherwise { /* Error: Compute not done for OFM write */ }
              } .elsewhen(dma_direction_reg === DMADirection.MEM_TO_BUF) {
                  can_start_dma := true.B
              } .otherwise { /* Error: Invalid DMA direction/buffer combo for start */ }
            } .otherwise { /* Error: Common conditions not met (not configured or busy) */ }

            when(can_start_dma) {
                dma_ctrl_io.start := true.B
                when(dma_direction_reg === DMADirection.MEM_TO_BUF) {
                    when(dma_buffer_id_reg === BufferIDs.IFM) { ifm_loaded_flag := false.B }
                    .elsewhen(dma_buffer_id_reg === BufferIDs.KERNEL) { kernel_loaded_flag := false.B }
                }
                accelerator_status_reg := STATUS_DMA_BUSY
                resp_data_reg := accelerator_status_reg // This will be STATUS_DMA_BUSY
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START: Conditions met. Starting DMA. Responding with ACK (${accelerator_status_reg}).\n")}
            } .otherwise {
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START: Error - conditions not met. Responding with STATUS_ERROR.\n")}
                // resp_data_reg is already STATUS_ERROR from default
            }
          }
          is(CMD_GET_STATUS) {
            resp_data_reg := accelerator_status_reg
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: Processing CMD_GET_STATUS. Returning status: ${accelerator_status_reg}\n")}
          }
        }
      }
    } // end is(sIdle)

    is(sDmaActive) {
      if (enableRoccDebugPrints.litToBoolean) {
          when(dma_ctrl_io.busy) {
            // printf(p"RoCC Cycle[${roccCycleCount}]: In sDmaActive, DMA controller is busy (dma_ctrl_io.busy=${dma_ctrl_io.busy}).\n")
          }
      }

      when(dma_ctrl_io.done) {
        val completed_op_buffer = dma_buffer_id_reg
        val completed_op_dir = dma_direction_reg

        if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: DMA Done signal received in sDmaActive. Op BufID=${completed_op_buffer}, Op Dir=${completed_op_dir}.\n")}

        when(completed_op_dir === DMADirection.MEM_TO_BUF) {
          when(completed_op_buffer === BufferIDs.IFM) {
            accelerator_status_reg := STATUS_DMA_IFM_LOAD_DONE
            ifm_loaded_flag := true.B
            // REMOVED direct .mem_reg access printf here
          }
          .elsewhen(completed_op_buffer === BufferIDs.KERNEL) {
            accelerator_status_reg := STATUS_DMA_KERNEL_LOAD_DONE
            kernel_loaded_flag := true.B
            // REMOVED direct .mem_reg access printf here
          }
          .otherwise { accelerator_status_reg := STATUS_DMA_ERROR }
        } .elsewhen (completed_op_dir === DMADirection.BUF_TO_MEM) {
          when(completed_op_buffer === BufferIDs.OFM) {
            accelerator_status_reg := STATUS_DMA_OFM_STORE_DONE
            ifm_loaded_flag := false.B
            kernel_loaded_flag := false.B
            compute_completed_flag := false.B
          }
          .otherwise { accelerator_status_reg := STATUS_DMA_ERROR }
        } .otherwise {
          accelerator_status_reg := STATUS_DMA_ERROR
        }
        dma_configured_flag := false.B
        rocc_state := sIdle
      } .elsewhen(dma_ctrl_io.dma_error) {
        if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: DMA Error signal received in sDmaActive.\n")}
        accelerator_status_reg := STATUS_DMA_ERROR
        ifm_loaded_flag := false.B
        kernel_loaded_flag := false.B
        compute_completed_flag := false.B
        dma_configured_flag := false.B
        dma_addr_configured_internal_flag := false.B
        rocc_state := sIdle
      } .elsewhen(!dma_ctrl_io.busy) {
        if (enableRoccDebugPrints.litToBoolean) {
          printf(p"RoCC Cycle[${roccCycleCount}]: In sDmaActive, DMA controller no longer busy (and no done/error this cycle). Returning to sIdle.\n")
        }
        rocc_state := sIdle
      }
    } // end is(sDmaActive)

    is(sWaitOFMRead) {
      if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: In UNUSED sWaitOFMRead state. Transitioning to sIdle.\n") }
      rocc_state := sIdle
    }

    is(sRespond) {
      if (enableRoccDebugPrints.litToBoolean) {
        // printf(p"RoCC Cycle[${roccCycleCount}]: In sRespond. resp_valid_reg=${resp_valid_reg}, io.resp.ready=${io.resp.ready}\n")
      }
      when(!resp_valid_reg) { // Response has been taken by CPU
        rocc_state := sIdle
        if (enableRoccDebugPrints.litToBoolean) {
             printf(p"RoCC Cycle[${roccCycleCount}]: Response taken by CPU, transitioning from sRespond to sIdle.\n")
        }
      }
    } // end is(sRespond)
  } // end switch(rocc_state)


  // ** NEW PRINTF LOGIC FOR VERIFYING LOADED DATA (Address 0) **
  // This logic prints one cycle AFTER the respective _loaded_flag is set.
  // It relies on ComputeUnit being idle and driving read_addr 0 to the buffers.
  val ifm_loaded_print_trigger = ifm_loaded_flag && !ifm_loaded_flag_prev
  val kernel_loaded_print_trigger = kernel_loaded_flag && !kernel_loaded_flag_prev

  // We need to delay the actual printing by one cycle to allow MinimalBuffer's RegNext to update io.read_data
  val do_print_ifm_sample = RegNext(ifm_loaded_print_trigger, false.B)
  val do_print_kernel_sample = RegNext(kernel_loaded_print_trigger, false.B)

  if(enableRoccDebugPrints.litToBoolean) {
    when(do_print_ifm_sample) {
      printf(p"RoCC DBG: IFM Load Sample. IFM_BUF[0 (via read_data)] = ${ifm_buffer.io.read_data} (raw SInt)\n")
      // Example to print a few more if you add a simple counter & mux for read_addr for debug:
      // This is a simplified example; proper sequential read needs more logic.
      // val ifm_debug_read_addr = RegInit(0.U(config.ifmAddrWidth.W))
      // val ifm_debug_read_count = RegInit(0.U(3.W)) // e.g. read 3 elements
      // when(ifm_loaded_print_trigger) { ifm_debug_read_count := 3.U; ifm_debug_read_addr := 0.U }
      // when(ifm_debug_read_count > 0.U) {
      //    // Problem: This conflicts with compute_unit driving ifm_buffer.io.read_addr
      //    // ifm_buffer.io.read_addr := ifm_debug_read_addr
      //    // printf ... (would need to wait for next cycle for data)
      //    ifm_debug_read_addr := ifm_debug_read_addr + 1.U
      //    ifm_debug_read_count := ifm_debug_read_count - 1.U
      // }
    }
    when(do_print_kernel_sample) {
      printf(p"RoCC DBG: KERNEL Load Sample. KERNEL_BUF[0 (via read_data)] = ${kernel_buffer.io.read_data} (raw SInt)\n")
    }
  }
  // ** END OF NEW PRINTF LOGIC **


  val dma_is_writing_to_spad = dma_direction_reg === DMADirection.MEM_TO_BUF && (rocc_state === sDmaActive || dma_ctrl_io.start)
  val spad_write_target_is_ifm    = dma_buffer_id_reg === BufferIDs.IFM    && dma_is_writing_to_spad
  val spad_write_target_is_kernel = dma_buffer_id_reg === BufferIDs.KERNEL && dma_is_writing_to_spad

  ifm_buffer.io.write_en    := dma_ctrl_io.spad_write_en && spad_write_target_is_ifm
  ifm_buffer.io.write_addr  := dma_ctrl_io.spad_write_addr
  ifm_buffer.io.write_data  := dma_ctrl_io.spad_write_data

  kernel_buffer.io.write_en   := dma_ctrl_io.spad_write_en && spad_write_target_is_kernel
  kernel_buffer.io.write_addr := dma_ctrl_io.spad_write_addr
  kernel_buffer.io.write_data := dma_ctrl_io.spad_write_data

  val dma_is_reading_from_spad = dma_direction_reg === DMADirection.BUF_TO_MEM && (rocc_state === sDmaActive || dma_ctrl_io.start)
  val spad_read_source_is_ofm  = dma_buffer_id_reg === BufferIDs.OFM && dma_is_reading_from_spad

  when(spad_read_source_is_ofm) {
    ofm_buffer.io.read_addr     := dma_ctrl_io.spad_read_addr
    dma_ctrl_io.spad_read_data  := ofm_buffer.io.read_data
    printf(p"RoCC Cycle[${roccCycleCount}]: DMA reading OFM_BUF: Addr=${dma_ctrl_io.spad_read_addr}, DataOut(next cycle)=${ofm_buffer.io.read_data}\n")
  } .otherwise {
    dma_ctrl_io.spad_read_data  := 0.S
  }

  // if (enableRoccDebugPrints.litToBoolean) {
  //    printf(p"RoCC Cycle[${roccCycleCount}]: --- End of Cycle --- AccelStatus=${accelerator_status_reg}, RoCCState=${rocc_state}, IFM_L=${ifm_loaded_flag}, KRN_L=${kernel_loaded_flag}, CMP_D=${compute_completed_flag}, DMA_CFG=${dma_configured_flag}, DMA_Busy=${dma_ctrl_io.busy}, CU_Busy=${compute_unit.io.busy}, RespValid=${io.resp.valid}\n")
  // }

}