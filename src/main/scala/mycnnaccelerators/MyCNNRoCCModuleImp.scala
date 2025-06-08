// filename: MyCNNRoCCModuleImp.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.diplomacy._
import CNNAcceleratorISA._

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, defaultConfig: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {

  val config = defaultConfig.copy(xLen = xLen, kernelRows = 5, kernelCols = 5)

  val enableRoccDebugPrints = false.B

  val roccCycleCount = RegInit(0.U(32.W))
  when(enableRoccDebugPrints) {
    roccCycleCount := roccCycleCount + 1.U
  }

  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  val dma_ctrl_io = outer.dma_engine_lazy.module.io

  val sIdle :: sWaitOFMRead :: sRespond :: sDmaActive :: Nil = Enum(4)
  val rocc_state = RegInit(sIdle)
  val rocc_state_prev = RegNext(rocc_state, sIdle)

  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  val accelerator_status_reg = RegInit(STATUS_IDLE)
  val acc_status_prev = RegNext(accelerator_status_reg, STATUS_IDLE)

  val dma_mem_base_addr_reg = Reg(UInt(xLen.W))
  val dma_length_bytes_reg = Reg(UInt(dmaConfigLenBits.W))
  val dma_direction_reg = Reg(UInt(dmaConfigDirBits.W))
  val dma_buffer_id_reg = Reg(UInt(dmaConfigBufIdBits.W))
  val dma_configured_flag = RegInit(false.B)
  val dma_addr_configured_internal_flag = RegInit(false.B)

  val current_kernel_dim_reg = RegInit(config.kernelRows.U(log2Ceil(config.kernelRows + 1).W))

  val ifm_loaded_flag = RegInit(false.B)
  val kernel_loaded_flag = RegInit(false.B)
  val compute_completed_flag = RegInit(false.B)

  val ifm_loaded_flag_prev = RegNext(ifm_loaded_flag, false.B)
  val kernel_loaded_flag_prev = RegNext(kernel_loaded_flag, false.B)
  val compute_completed_flag_prev = RegNext(compute_completed_flag, false.B)
  val dma_configured_flag_prev = RegNext(dma_configured_flag, false.B)
  val current_kernel_dim_reg_prev = RegNext(current_kernel_dim_reg, config.kernelRows.U)


  val ofm_dma_read_print_count = RegInit(0.U(4.W))

  val is_get_status_command = WireDefault(false.B)
  when(io.cmd.valid) {
      is_get_status_command := io.cmd.bits.inst.funct === CMD_GET_STATUS
  }

  val get_status_can_be_accepted = is_get_status_command

  val other_commands_can_be_accepted = !is_get_status_command && !dma_ctrl_io.busy && !compute_unit.io.busy

  val rocc_interface_ready = (rocc_state === sIdle) && !resp_valid_reg

  io.cmd.ready := rocc_interface_ready && (get_status_can_be_accepted || other_commands_can_be_accepted)
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  io.busy := (compute_unit.io.busy || dma_ctrl_io.busy || (rocc_state =/= sIdle) || resp_valid_reg)
  io.interrupt := false.B

  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.S(config.dataWidth.W)

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.S(config.dataWidth.W)

  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data

  compute_unit.io.start := false.B
  compute_unit.io.ifm_read_data    := ifm_buffer.io.read_data
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data
  ifm_buffer.io.read_addr          := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr       := compute_unit.io.kernel_read_addr
  compute_unit.io.actual_kernel_dim_in := current_kernel_dim_reg

  dma_ctrl_io.start := false.B
  dma_ctrl_io.mem_base_addr := dma_mem_base_addr_reg
  dma_ctrl_io.length_bytes := dma_length_bytes_reg
  dma_ctrl_io.directionBits := dma_direction_reg
  dma_ctrl_io.buffer_id := dma_buffer_id_reg

  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B
    if (enableRoccDebugPrints.litToBoolean) {
      printf(p"RoCC Cycle[${roccCycleCount}]: CPU took response. rd=${io.resp.bits.rd}, data=0x${Hexadecimal(io.resp.bits.data)}\n")
    }
  }

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
    when(current_kernel_dim_reg =/= current_kernel_dim_reg_prev) {
      printf(p"RoCC Cycle[${roccCycleCount}]: current_kernel_dim_reg changed from ${current_kernel_dim_reg_prev} to ${current_kernel_dim_reg}\n")
    }
    when(io.cmd.fire) {
        printf(p"RoCC Cycle[${roccCycleCount}]: CMD Received: funct=${io.cmd.bits.inst.funct}, rs1=0x${Hexadecimal(io.cmd.bits.rs1)}, rs2=0x${Hexadecimal(io.cmd.bits.rs2)}, rd=${io.cmd.bits.inst.rd}\n")
    }
    val dma_start_asserted_for_debug = RegNext(dma_ctrl_io.start, false.B)
    when(dma_ctrl_io.start && !dma_start_asserted_for_debug) {
        printf(p"RoCC Cycle[${roccCycleCount}]: DMA Start asserted. BufID=${dma_buffer_id_reg}, Dir=${dma_direction_reg}, Len=${dma_length_bytes_reg}, Addr=0x${Hexadecimal(dma_mem_base_addr_reg)}\n")
    }
    val compute_start_asserted_for_debug = RegNext(compute_unit.io.start, false.B)
    when(compute_unit.io.start && !compute_start_asserted_for_debug) {
        printf(p"RoCC Cycle[${roccCycleCount}]: Compute Unit Start asserted. KernelDim=${current_kernel_dim_reg}\n")
    }
  }


  switch(rocc_state) {
    is(sIdle) {
      // *** MODIFIED BLOCK START ***
      // Check for compute completion
      when(accelerator_status_reg === STATUS_COMPUTING && compute_unit.io.done && !compute_unit.io.busy) {
        // Check the saturation event flag from the compute unit
        when(compute_unit.io.saturation_event) {
          if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: Compute unit finished with SATURATION. Setting STATUS_COMPUTE_DONE_SATURATED.\n")
          }
          accelerator_status_reg := STATUS_COMPUTE_DONE_SATURATED
        } .otherwise {
          if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: Compute unit finished. Setting STATUS_COMPUTE_DONE.\n")
          }
          accelerator_status_reg := STATUS_COMPUTE_DONE
        }
        compute_completed_flag := true.B // Computation is complete in both cases
      }
      // *** MODIFIED BLOCK END ***

      when(accelerator_status_reg === STATUS_DMA_BUSY && dma_ctrl_io.busy) {
        if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: DMA busy, status DMA_BUSY. RoCC FSM sIdle -> sDmaActive.\n")
        }
        rocc_state := sDmaActive
      }

      when(io.cmd.valid && !resp_valid_reg) {
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd
        resp_data_reg := STATUS_ERROR.asUInt
        resp_valid_reg := true.B
        rocc_state := sRespond

        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_IFM_ADDR_DATA - Not Implemented. Responding STATUS_ERROR.\n") }
          }
          is(CMD_SET_KERNEL_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_KERNEL_ADDR_DATA - Not Implemented. Responding STATUS_ERROR.\n") }
          }
          is(CMD_GET_OFM_ADDR_DATA) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_GET_OFM_ADDR_DATA - Not Implemented. Responding STATUS_ERROR.\n") }
          }

          is(CMD_START_COMPUTE) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: sIdle: CMD_START_COMPUTE. IFM_L=${ifm_loaded_flag}, KERN_L=${kernel_loaded_flag}, KERN_DIM=${current_kernel_dim_reg}, CU_Busy=${compute_unit.io.busy}, DMA_Busy=${dma_ctrl_io.busy}\n")}
            when(ifm_loaded_flag && kernel_loaded_flag && !compute_unit.io.busy && !dma_ctrl_io.busy) {
              when(current_kernel_dim_reg === 1.U || current_kernel_dim_reg === 3.U || current_kernel_dim_reg === 5.U) {
                compute_unit.io.start := true.B
                accelerator_status_reg := STATUS_COMPUTING
                compute_completed_flag := false.B
                resp_data_reg := STATUS_COMPUTING.asUInt
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_START_COMPUTE: Starting compute. Kernel Dim=${current_kernel_dim_reg}. Responding ACK (${STATUS_COMPUTING}).\n")}
              } .otherwise {
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_START_COMPUTE: Error - Invalid kernel_dim_reg=${current_kernel_dim_reg}. Responding STATUS_ERROR.\n")}
                accelerator_status_reg := STATUS_ERROR
              }
            } .otherwise {
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_START_COMPUTE: Error - preconditions not met. Responding STATUS_ERROR.\n")}
                accelerator_status_reg := STATUS_ERROR
            }
          }
          is(CMD_DMA_CONFIG_ADDR) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_ADDR. Addr=0x${Hexadecimal(cmd.rs1)}\n")}
            dma_mem_base_addr_reg := cmd.rs1
            dma_addr_configured_internal_flag := true.B
            dma_configured_flag := false.B
            accelerator_status_reg := STATUS_IDLE
            resp_data_reg := accelerator_status_reg
          }
          is(CMD_DMA_CONFIG_PARAMS) {
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_PARAMS. Params=0x${Hexadecimal(cmd.rs1)}. AddrInternalFlag=${dma_addr_configured_internal_flag}\n")}
            when(dma_addr_configured_internal_flag) {
              val param_val = cmd.rs1
              dma_buffer_id_reg := param_val(dmaConfigBufIdBits - 1, 0)
              dma_direction_reg := param_val(dmaConfigBufIdBits)
              dma_length_bytes_reg := param_val(dmaConfigTotalParamBits - 1, dmaConfigBufIdBits + dmaConfigDirBits)
              dma_configured_flag := true.B
              dma_addr_configured_internal_flag := false.B
              accelerator_status_reg := STATUS_DMA_CONFIG_READY
              resp_data_reg := accelerator_status_reg
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_PARAMS: Parsed. BufID=${dma_buffer_id_reg}, Dir=${dma_direction_reg}, Len=${dma_length_bytes_reg}. Status=${accelerator_status_reg}. Responding ACK.\n")}
            } .otherwise {
              accelerator_status_reg := STATUS_DMA_ERROR
              resp_data_reg := STATUS_DMA_ERROR.asUInt
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_CONFIG_PARAMS: Error - ADDR not configured. Responding STATUS_DMA_ERROR.\n")}
            }
          }
          is(CMD_DMA_START) {
            val is_ofm_dma_write = (dma_direction_reg === DMADirection.BUF_TO_MEM && dma_buffer_id_reg === BufferIDs.OFM)
            val common_dma_start_conditions = dma_configured_flag && !dma_ctrl_io.busy && !compute_unit.io.busy
            if (enableRoccDebugPrints.litToBoolean) {
                printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START. IsOFMWrite=${is_ofm_dma_write}, DMAConf=${dma_configured_flag}, CompDone(ifOFM)=${compute_completed_flag}, DMABusy=${dma_ctrl_io.busy}, CUBusy=${compute_unit.io.busy}\n")
            }
            val can_start_dma = WireDefault(false.B)
            when(common_dma_start_conditions) {
              when(is_ofm_dma_write) {
                when(compute_completed_flag) {
                  can_start_dma := true.B
                  ofm_dma_read_print_count := 0.U
                }
              } .elsewhen(dma_direction_reg === DMADirection.MEM_TO_BUF) {
                  when(dma_buffer_id_reg === BufferIDs.KERNEL) {
                      when(current_kernel_dim_reg === 1.U || current_kernel_dim_reg === 3.U || current_kernel_dim_reg === 5.U) {
                          can_start_dma := true.B
                      } .otherwise {
                          if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START: Error - KERNEL load with invalid dim ${current_kernel_dim_reg}.\n")}
                          accelerator_status_reg := STATUS_ERROR
                      }
                  } .otherwise {
                      can_start_dma := true.B
                  }
              }
            }

            when(can_start_dma) {
              dma_ctrl_io.start := true.B
              when(dma_direction_reg === DMADirection.MEM_TO_BUF) {
                when(dma_buffer_id_reg === BufferIDs.IFM) { ifm_loaded_flag := false.B }
                .elsewhen(dma_buffer_id_reg === BufferIDs.KERNEL) { kernel_loaded_flag := false.B }
              }
              accelerator_status_reg := STATUS_DMA_BUSY
              resp_data_reg := STATUS_DMA_BUSY.asUInt
              if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START: Starting DMA. Responding ACK (${STATUS_DMA_BUSY}).\n")}
            } .otherwise {
              when (accelerator_status_reg =/= STATUS_ERROR && accelerator_status_reg =/= STATUS_DMA_ERROR) {
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_DMA_START: Error - conditions not met. Responding STATUS_ERROR.\n")}
              }
            }
          }
          is(CMD_GET_STATUS) {
            resp_data_reg := accelerator_status_reg
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_GET_STATUS. Returning status: ${accelerator_status_reg}\n")}
          }
          is(CMD_SET_KERNEL_PARAMS) {
            val K_dim = cmd.rs1(log2Ceil(config.kernelRows + 1) -1, 0)
            if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_KERNEL_PARAMS. K_dim from rs1=${K_dim}\n")}
            when(K_dim === 1.U || K_dim === 3.U || K_dim === 5.U) {
                current_kernel_dim_reg := K_dim
                accelerator_status_reg := STATUS_KERNEL_PARAMS_SET
                resp_data_reg := accelerator_status_reg
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: Kernel Dim set to ${K_dim}. Status ${STATUS_KERNEL_PARAMS_SET}\n")}
            } .otherwise {
                if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: CMD_SET_KERNEL_PARAMS: Error - Invalid K_dim ${K_dim}. Responding STATUS_ERROR.\n")}
                accelerator_status_reg := STATUS_ERROR
            }
          }
        }
      }
    }

    is(sDmaActive) {
      when(dma_ctrl_io.done) {
        val completed_op_buffer = dma_buffer_id_reg
        val completed_op_dir = dma_direction_reg
        if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: DMA Done. Op BufID=${completed_op_buffer}, Op Dir=${completed_op_dir}.\n")}

        when(completed_op_dir === DMADirection.MEM_TO_BUF) {
          when(completed_op_buffer === BufferIDs.IFM) {
            accelerator_status_reg := STATUS_DMA_IFM_LOAD_DONE
            ifm_loaded_flag := true.B
          }
          .elsewhen(completed_op_buffer === BufferIDs.KERNEL) {
            accelerator_status_reg := STATUS_DMA_KERNEL_LOAD_DONE
            kernel_loaded_flag := true.B
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
          printf(p"RoCC Cycle[${roccCycleCount}]: In sDmaActive, DMA no longer busy (no done/error this cycle). To sIdle. AccStatus=${accelerator_status_reg}\n")
        }
        rocc_state := sIdle
      }
    }

    is(sWaitOFMRead) {
      if (enableRoccDebugPrints.litToBoolean) { printf(p"RoCC Cycle[${roccCycleCount}]: In UNUSED sWaitOFMRead state. To sIdle.\n") }
      rocc_state := sIdle
    }

    is(sRespond) {
      when(!resp_valid_reg) {
        rocc_state := sIdle
        if (enableRoccDebugPrints.litToBoolean) {
            printf(p"RoCC Cycle[${roccCycleCount}]: Response taken by CPU, sRespond -> sIdle.\n")
        }
      }
    }
  }


  val ifm_loaded_print_trigger = ifm_loaded_flag && !ifm_loaded_flag_prev
  val kernel_loaded_print_trigger = kernel_loaded_flag && !kernel_loaded_flag_prev
  val do_print_ifm_sample = RegNext(ifm_loaded_print_trigger, false.B)
  val do_print_kernel_sample = RegNext(kernel_loaded_print_trigger, false.B)

  if(enableRoccDebugPrints.litToBoolean) {
    when(do_print_ifm_sample && (rocc_state === sIdle || rocc_state === sDmaActive) && !compute_unit.io.busy) {
      val temp_ifm_read_addr = WireDefault(compute_unit.io.ifm_read_addr)
      when(rocc_state === sIdle && !compute_unit.io.busy){
          temp_ifm_read_addr := 0.U
      }
      ifm_buffer.io.read_addr := temp_ifm_read_addr
      printf(p"RoCC DBG: IFM Load Sample. IFM_BUF[${temp_ifm_read_addr}] = ${ifm_buffer.io.read_data}\n")
    }
    when(do_print_kernel_sample && (rocc_state === sIdle || rocc_state === sDmaActive) && !compute_unit.io.busy) {
      val temp_kernel_read_addr = WireDefault(compute_unit.io.kernel_read_addr)
      when(rocc_state === sIdle && !compute_unit.io.busy){
        temp_kernel_read_addr := 0.U
      }
      kernel_buffer.io.read_addr := temp_kernel_read_addr
      printf(p"RoCC DBG: KERNEL Load Sample. KERNEL_BUF[${temp_kernel_read_addr}] = ${kernel_buffer.io.read_data}\n")
    }
  }

  val dma_is_writing_to_spad = dma_direction_reg === DMADirection.MEM_TO_BUF && (dma_ctrl_io.busy || dma_ctrl_io.start)
  val spad_write_target_is_ifm    = dma_buffer_id_reg === BufferIDs.IFM    && dma_is_writing_to_spad
  val spad_write_target_is_kernel = dma_buffer_id_reg === BufferIDs.KERNEL && dma_is_writing_to_spad

  ifm_buffer.io.write_en      := dma_ctrl_io.spad_write_en && spad_write_target_is_ifm
  ifm_buffer.io.write_addr    := dma_ctrl_io.spad_write_addr
  ifm_buffer.io.write_data    := dma_ctrl_io.spad_write_data

  kernel_buffer.io.write_en   := dma_ctrl_io.spad_write_en && spad_write_target_is_kernel
  kernel_buffer.io.write_addr := dma_ctrl_io.spad_write_addr
  kernel_buffer.io.write_data := dma_ctrl_io.spad_write_data

  ofm_buffer.io.read_addr   := dma_ctrl_io.spad_read_addr
  dma_ctrl_io.spad_read_data := 0.S

  val is_configured_for_ofm_store = (dma_direction_reg === DMADirection.BUF_TO_MEM && dma_buffer_id_reg === BufferIDs.OFM)

  when(is_configured_for_ofm_store && (dma_ctrl_io.busy || dma_ctrl_io.start) ) {
    dma_ctrl_io.spad_read_data := ofm_buffer.io.read_data
    if (enableRoccDebugPrints.litToBoolean) {
      when(dma_ctrl_io.busy && ofm_dma_read_print_count < 8.U) {
        printf(p"RoCC Cycle[${roccCycleCount}]: DMA reading OFM_BUF: Addr=${dma_ctrl_io.spad_read_addr}, Data=${ofm_buffer.io.read_data}\n")
        ofm_dma_read_print_count := ofm_dma_read_print_count + 1.U
      }
    }
  }
}
