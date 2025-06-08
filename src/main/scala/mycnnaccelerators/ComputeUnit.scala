// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

// Stage 1 to Stage 2 Pipeline Register Bundle
class PipeRegsS2(val config: AcceleratorConfig) extends Bundle {
  val is_first_mac = Bool()
  val is_last_mac = Bool()
  val ofm_write_addr = UInt(config.ofmAddrWidth.W)
  val ifm_access_is_pad = Bool()
}

// Stage 2 to Stage 3 Pipeline Register Bundle
class PipeRegsS3(val config: AcceleratorConfig) extends Bundle {
  val ifm_data = SInt(config.dataWidth.W)
  val kernel_data = SInt(config.dataWidth.W)
  val is_first_mac = Bool()
  val is_last_mac = Bool()
  val ofm_write_addr = UInt(config.ofmAddrWidth.W)
}

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())
  val saturation_event = Output(Bool())
  val actual_kernel_dim_in = Input(UInt(log2Ceil(config.kernelRows + 1).W))
  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W))
  val ifm_read_data = Input(SInt(config.dataWidth.W))
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

  val sIdle :: sRunning :: sDraining :: sDoneCU :: Nil = Enum(4)
  val state = RegInit(sIdle)

  // --- Pipeline Registers ---
  val s2_pipe_regs = Reg(new PipeRegsS2(config))
  val s2_pipe_valid = RegInit(false.B)

  val s3_pipe_regs = Reg(new PipeRegsS3(config))
  val s3_pipe_valid = RegInit(false.B)

  // --- Loop Counters ---
  val out_r = RegInit(0.U(log2Ceil(config.ifmRows).W))
  val out_c = RegInit(0.U(log2Ceil(config.ifmCols).W))
  val k_r   = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_c   = RegInit(0.U(log2Ceil(config.kernelCols).W))

  val current_kernel_dim = Reg(UInt(log2Ceil(config.kernelRows + 1).W))
  val pad_val = Wire(UInt(log2Ceil(config.kernelRows).W))
  when(current_kernel_dim > 1.U) {
    pad_val := (current_kernel_dim - 1.U) / 2.U
  } .otherwise {
    pad_val := 0.U
  }

  // --- Accumulator and Saturation ---
  val guardBits = log2Ceil(config.kernelRows * config.kernelCols)
  val accWidth = (2 * config.dataWidth) + guardBits
  val accumulator = RegInit(0.S(accWidth.W))
  val saturation_occurred_reg = RegInit(false.B)

  // --- Control Signals and Defaults ---
  io.busy := (state === sRunning || state === sDraining)
  io.done := (state === sDoneCU)
  io.saturation_event := saturation_occurred_reg
  io.ofm_write_en := false.B
  io.ofm_write_addr := 0.U
  io.ofm_write_data := 0.S

  // =================================================================
  // == STAGE 1: Address Generation & Fetch Request (Combinational) ==
  // =================================================================
  val ifm_r_eff = out_r.zext.asSInt - pad_val.zext.asSInt + k_r.zext.asSInt
  val ifm_c_eff = out_c.zext.asSInt - pad_val.zext.asSInt + k_c.zext.asSInt
  val ifm_access_is_pad_s1 = (ifm_r_eff < 0.S || ifm_r_eff >= config.ifmRows.S ||
                              ifm_c_eff < 0.S || ifm_c_eff >= config.ifmCols.S)

  io.ifm_read_addr := Mux(ifm_access_is_pad_s1, 0.U, ifm_r_eff.asUInt * config.ifmCols.U + ifm_c_eff.asUInt)
  io.kernel_read_addr := k_r * current_kernel_dim + k_c

  // =====================================================================
  // == STAGE 2 -> 3 Latching (End of Cycle)                             ==
  // =====================================================================
  when(s2_pipe_valid) {
    s3_pipe_valid := true.B
    s3_pipe_regs.ifm_data := Mux(s2_pipe_regs.ifm_access_is_pad, 0.S, io.ifm_read_data)
    s3_pipe_regs.kernel_data := io.kernel_read_data
    s3_pipe_regs.is_first_mac := s2_pipe_regs.is_first_mac
    s3_pipe_regs.is_last_mac := s2_pipe_regs.is_last_mac
    s3_pipe_regs.ofm_write_addr := s2_pipe_regs.ofm_write_addr
  } .otherwise {
    s3_pipe_valid := false.B
  }

  // =====================================================================
  // == STAGE 1 -> 2 Latching (End of Cycle)                             ==
  // =====================================================================
  s2_pipe_valid := (state === sRunning) // Only feed the pipeline when in sRunning
  s2_pipe_regs.is_first_mac := (k_r === 0.U) && (k_c === 0.U)
  s2_pipe_regs.is_last_mac := (k_r === current_kernel_dim - 1.U) && (k_c === current_kernel_dim - 1.U)
  s2_pipe_regs.ofm_write_addr := out_r * config.ifmCols.U + out_c
  s2_pipe_regs.ifm_access_is_pad := ifm_access_is_pad_s1

  // =====================================================================
  // == STAGE 3: MAC and Writeback (Uses S3 registers)                  ==
  // =====================================================================
  when(s3_pipe_valid) {
    val product = (s3_pipe_regs.ifm_data * s3_pipe_regs.kernel_data).asSInt
    val next_accumulator = Mux(s3_pipe_regs.is_first_mac, product, accumulator + product)
    accumulator := next_accumulator

    when(s3_pipe_regs.is_last_mac) {
      val minSIntValue = -(BigInt(1) << (config.dataWidth - 1))
      val maxSIntValue = (BigInt(1) << (config.dataWidth - 1)) - 1
      val shifted_accumulator = (next_accumulator >> config.F_BITS).asSInt

      val is_saturating_high = shifted_accumulator > maxSIntValue.S
      val is_saturating_low  = shifted_accumulator < minSIntValue.S

      when(is_saturating_high || is_saturating_low) {
        saturation_occurred_reg := true.B
      }
      
      val saturated_ofm_data = Mux(is_saturating_high, maxSIntValue.S(config.dataWidth.W),
                                Mux(is_saturating_low, minSIntValue.S(config.dataWidth.W),
                                    shifted_accumulator(config.dataWidth - 1, 0).asSInt))

      io.ofm_write_en   := true.B
      io.ofm_write_addr := s3_pipe_regs.ofm_write_addr
      io.ofm_write_data := saturated_ofm_data
    }
  }
  
  val drain_counter = Reg(UInt(2.W))

  // --- FSM and Counter Control Logic ---
  switch(state) {
    is(sIdle) {
      when(io.start) {
        state := sRunning
        out_r := 0.U
        out_c := 0.U
        k_r   := 0.U
        k_c   := 0.U
        accumulator := 0.S
        saturation_occurred_reg := false.B
        current_kernel_dim := io.actual_kernel_dim_in
      }
    }

    is(sRunning) {
      val k_c_is_last = (k_c === (current_kernel_dim - 1.U))
      val k_r_is_last = (k_r === (current_kernel_dim - 1.U))

      when(k_c_is_last) {
        k_c := 0.U
        when(k_r_is_last) {
          k_r := 0.U
          val out_c_is_last = (out_c === (config.ifmCols.U - 1.U))
          when(out_c_is_last) {
            out_c := 0.U
            val out_r_is_last = (out_r === (config.ifmRows.U - 1.U))
            when(out_r_is_last) {
                state := sDraining
                drain_counter := 2.U
            } .otherwise {
                out_r := out_r + 1.U
            }
          } .otherwise {
            out_c := out_c + 1.U
          }
        } .otherwise {
          k_r := k_r + 1.U
        }
      } .otherwise {
        k_c := k_c + 1.U
      }
    }
    
    is(sDraining) {
        drain_counter := drain_counter - 1.U
        when(drain_counter === 1.U) {
            state := sDoneCU
        }
    }

    is(sDoneCU) {
      when(!io.start) {
        state := sIdle
      }
    }
  }
}