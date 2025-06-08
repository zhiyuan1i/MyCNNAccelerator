// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())
  // New output to signal that a saturation event has occurred at least once
  val saturation_event = Output(Bool())

  // Inputs for actual kernel dimensions to use for this computation
  // Width is based on max possible kernel dimension (e.g., up to 5 for 5x5)
  val actual_kernel_dim_in = Input(UInt(log2Ceil(config.kernelRows + 1).W))

  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W))
  val ifm_read_data = Input(SInt(config.dataWidth.W))

  // kernel_read_addr width is for the max-sized buffer
  val kernel_read_addr = Output(UInt(config.kernelAddrWidth.W))
  val kernel_read_data = Input(SInt(config.dataWidth.W))

  val ofm_write_en   = Output(Bool())
  val ofm_write_addr = Output(UInt(config.ofmAddrWidth.W))
  val ofm_write_data = Output(SInt(config.dataWidth.W))
}

class ComputeUnit(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitIO(config))

  val dutCycleCount = RegInit(0.U(32.W))
  dutCycleCount := dutCycleCount + 1.U

  val sIdle :: sFetchIFM :: sFetchKernel :: sMAC :: sWriteOFM :: sDoneCU :: Nil = Enum(6)
  val state = RegInit(sIdle)

  val out_r = RegInit(0.U(log2Ceil(config.ifmRows).W))
  val out_c = RegInit(0.U(log2Ceil(config.ifmCols).W))

  val k_r = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_c = RegInit(0.U(log2Ceil(config.kernelCols).W))

  val current_kernel_dim = io.actual_kernel_dim_in
  val pad_val = Wire(UInt(log2Ceil(config.kernelRows).W))

  when(current_kernel_dim > 1.U) {
    pad_val := (current_kernel_dim - 1.U) / 2.U
  } .otherwise {
    pad_val := 0.U
  }
  val pad_rows_val = pad_val
  val pad_cols_val = pad_val


  val guardBits = log2Ceil(config.kernelRows * config.kernelCols)
  val accWidth = (2 * config.dataWidth) + guardBits
  val accumulator = RegInit(0.S(accWidth.W))

  val ifm_data_reg    = Reg(SInt(config.dataWidth.W))
  val kernel_data_reg = Reg(SInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B)
  val ifm_access_is_pad = RegInit(false.B)

  // This sticky register will be set if saturation occurs at any point
  val saturation_occurred_reg = RegInit(false.B)

  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)
  io.saturation_event := saturation_occurred_reg

  io.ifm_read_addr   := 0.U
  io.kernel_read_addr:= 0.U
  io.ofm_write_en    := false.B
  io.ofm_write_addr  := out_r * config.ifmCols.U + out_c

  // --- Saturation Logic ---
  val minSIntValue = -(BigInt(1) << (config.dataWidth - 1))
  val maxSIntValue = (BigInt(1) << (config.dataWidth - 1)) - 1
  val shifted_accumulator = (accumulator >> config.F_BITS).asSInt

  val is_saturating_high = shifted_accumulator > maxSIntValue.S
  val is_saturating_low  = shifted_accumulator < minSIntValue.S

  val saturated_ofm_data = Wire(SInt(config.dataWidth.W))
  when(is_saturating_high) {
    saturated_ofm_data := maxSIntValue.S(config.dataWidth.W)
  } .elsewhen(is_saturating_low) {
    saturated_ofm_data := minSIntValue.S(config.dataWidth.W)
  } .otherwise {
    saturated_ofm_data := shifted_accumulator(config.dataWidth - 1, 0).asSInt
  }
  io.ofm_write_data := saturated_ofm_data
  // --- End of Saturation Logic ---


  val enableDebugPrints = false.B
  if (enableDebugPrints.litToBoolean) {
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: State=${state}, actual_k_dim=${current_kernel_dim}, pad_val=${pad_val}\n")
  }

  switch(state) {
    is(sIdle) {
      when(io.start) {
        out_r        := 0.U
        out_c        := 0.U
        k_r          := 0.U
        k_c          := 0.U
        accumulator  := 0.S
        saturation_occurred_reg := false.B // Reset flag on start
        state        := sFetchIFM
        data_fetch_cycle := false.B
        ifm_access_is_pad := false.B
      }
    }

    is(sFetchIFM) {
      when(!data_fetch_cycle) {
        val ifm_r_eff = out_r.zext.asSInt - pad_rows_val.zext.asSInt + k_r.zext.asSInt
        val ifm_c_eff = out_c.zext.asSInt - pad_cols_val.zext.asSInt + k_c.zext.asSInt

        val current_ifm_access_is_pad = (ifm_r_eff < 0.S || ifm_r_eff >= config.ifmRows.S ||
                                         ifm_c_eff < 0.S || ifm_c_eff >= config.ifmCols.S)
        ifm_access_is_pad := current_ifm_access_is_pad

        when(current_ifm_access_is_pad) {
          io.ifm_read_addr := 0.U
        } .otherwise {
          io.ifm_read_addr := ifm_r_eff.asUInt * config.ifmCols.U + ifm_c_eff.asUInt
        }
        data_fetch_cycle := true.B
      } .otherwise {
        when(ifm_access_is_pad) {
          ifm_data_reg := 0.S(config.dataWidth.W)
        } .otherwise {
          ifm_data_reg := io.ifm_read_data
        }
        state := sFetchKernel
        data_fetch_cycle := false.B
      }
    }

    is(sFetchKernel) {
      when(!data_fetch_cycle) {
        io.kernel_read_addr := k_r * current_kernel_dim + k_c
        data_fetch_cycle := true.B
      } .otherwise {
        kernel_data_reg := io.kernel_read_data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      val product = (ifm_data_reg * kernel_data_reg).asSInt
      accumulator := accumulator + product

      val k_c_is_last = (k_c === (current_kernel_dim - 1.U))
      when(k_c_is_last) {
        k_c := 0.U
        val k_r_is_last = (k_r === (current_kernel_dim - 1.U))
        when(k_r_is_last) {
          k_r := 0.U
          state := sWriteOFM
        } .otherwise {
          k_r := k_r + 1.U
          state := sFetchIFM
        }
      } .otherwise {
        k_c := k_c + 1.U
        state := sFetchIFM
      }
    }

    is(sWriteOFM) {
      io.ofm_write_en := true.B
      
      // If saturation is detected during this write cycle, set the sticky flag
      when(is_saturating_high || is_saturating_low) {
        saturation_occurred_reg := true.B
      }

      accumulator := 0.S

      val out_c_is_last = (out_c === (config.ifmCols.U - 1.U))
      when(out_c_is_last) {
        out_c := 0.U
        val out_r_is_last = (out_r === (config.ifmRows.U - 1.U))
        when(out_r_is_last) {
          state := sDoneCU
        } .otherwise {
          out_r := out_r + 1.U
          state := sFetchIFM
          k_r := 0.U
          k_c := 0.U
        }
      } .otherwise {
        out_c := out_c + 1.U
        state := sFetchIFM
        k_r := 0.U
        k_c := 0.U
      }
      data_fetch_cycle := false.B
    }

    is(sDoneCU) {
      when(!io.start) {
        state := sIdle
      }
    }
  }
}