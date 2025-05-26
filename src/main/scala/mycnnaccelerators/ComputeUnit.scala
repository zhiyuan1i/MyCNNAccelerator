// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())

  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W)) // Width based on actual IFM buffer size
  val ifm_read_data = Input(SInt(config.dataWidth.W))

  val kernel_read_addr = Output(UInt(config.kernelAddrWidth.W))
  val kernel_read_data = Input(SInt(config.dataWidth.W))

  // For 'same' convolution, OFM dimensions match IFM dimensions.
  // The 'config' instance passed to ComputeUnit MUST reflect this for OFM parameters:
  // E.g., config.ofmRows should be set to config.ifmRows by the instantiating module.
  // config.ofmCols should be set to config.ifmCols.
  // config.ofmAddrWidth should be log2Ceil(config.ifmRows * config.ifmCols).
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

  // Iterators for OFM (output feature map)
  // For 'same' convolution, OFM has same dimensions as IFM.
  // These iterators will go up to config.ifmRows-1 and config.ifmCols-1.
  val out_r = RegInit(0.U(log2Ceil(config.ifmRows).W))
  val out_c = RegInit(0.U(log2Ceil(config.ifmCols).W))

  // Iterators for kernel
  val k_r   = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_c   = RegInit(0.U(log2Ceil(config.kernelCols).W))

  // Padding calculation for 'same' convolution (integer division is fine)
  val pad_rows_val = (config.kernelRows - 1) / 2
  val pad_cols_val = (config.kernelCols - 1) / 2

  val guardBits = log2Ceil(config.kernelRows * config.kernelCols)
  val accWidth = (2 * config.dataWidth) + guardBits
  val accumulator = RegInit(0.S(accWidth.W))

  val ifm_data_reg    = Reg(SInt(config.dataWidth.W))
  val kernel_data_reg = Reg(SInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B) // True if IFM/Kernel data is expected next cycle

  // This register will be true if the current IFM fetch corresponds to a padded region
  val ifm_access_is_pad = RegInit(false.B)

  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)

  io.ifm_read_addr   := 0.U // Default
  io.kernel_read_addr:= 0.U // Default
  io.ofm_write_en    := false.B // Default

  val shifted_accumulator = (accumulator >> config.F_BITS)
  io.ofm_write_data  := shifted_accumulator(config.dataWidth - 1, 0).asSInt

  // OFM write address: out_r * <num_ofm_cols> + out_c
  // For 'same' convolution, <num_ofm_cols> is config.ifmCols.
  // config.ofmAddrWidth (used for io.ofm_write_addr width) MUST be appropriate.
  io.ofm_write_addr  := out_r * config.ifmCols.U + out_c

  val enableDebugPrints = false.B // Set to true for local debugging
  if (enableDebugPrints.litToBoolean) {
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: -------------------- Cycle Start --------------------\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: State: ${state} (Idle=${sIdle}, FetchIFM=${sFetchIFM}, FetchKernel=${sFetchKernel}, MAC=${sMAC}, WriteOFM=${sWriteOFM}, DoneCU=${sDoneCU})\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: data_fetch_cycle: ${data_fetch_cycle}, ifm_access_is_pad: ${ifm_access_is_pad}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Counters: out_r=${out_r}, out_c=${out_c}, k_r=${k_r}, k_c=${k_c}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Padding: pad_rows=${pad_rows_val}, pad_cols=${pad_cols_val}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Accumulator: ${accumulator} (dec), 0x${Hexadecimal(accumulator)}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Inputs: start=${io.start}, ifm_rd_data=${io.ifm_read_data}, krnl_rd_data=${io.kernel_read_data}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Outputs: ifm_addr=${io.ifm_read_addr}, krnl_addr=${io.kernel_read_addr}\n")
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: Outputs: ofm_wr_en=${io.ofm_write_en}, ofm_wr_addr=${io.ofm_write_addr}, ofm_data=${io.ofm_write_data} (dec)\n")
  }

  switch(state) {
    is(sIdle) {
      when(io.start) {
        out_r       := 0.U
        out_c       := 0.U
        k_r         := 0.U
        k_c         := 0.U
        accumulator := 0.S
        state       := sFetchIFM
        data_fetch_cycle := false.B
        ifm_access_is_pad := false.B
      }
    }

    is(sFetchIFM) {
      when(!data_fetch_cycle) { // First cycle of sFetchIFM: calculate address and padding status
        // Effective IFM coordinates (signed because subtraction can lead to negative values for padding cases)
        val ifm_r_eff = out_r.zext.asSInt - pad_rows_val.S + k_r.zext.asSInt
        val ifm_c_eff = out_c.zext.asSInt - pad_cols_val.S + k_c.zext.asSInt

        val current_ifm_access_is_pad = (ifm_r_eff < 0.S || ifm_r_eff >= config.ifmRows.S ||
                                         ifm_c_eff < 0.S || ifm_c_eff >= config.ifmCols.S)
        ifm_access_is_pad := current_ifm_access_is_pad

        when(current_ifm_access_is_pad) {
          io.ifm_read_addr := 0.U // Address for padded reads doesn't matter if data is forced to 0
        } .otherwise {
          io.ifm_read_addr := ifm_r_eff.asUInt * config.ifmCols.U + ifm_c_eff.asUInt
        }
        data_fetch_cycle := true.B

        if (enableDebugPrints.litToBoolean) {
            printf(p"RoCC DUT Cycle[${dutCycleCount}]: sFetchIFM (Cycle 1): out_r=${out_r}, out_c=${out_c}, k_r=${k_r}, k_c=${k_c} => ifm_r_eff=${ifm_r_eff}, ifm_c_eff=${ifm_c_eff}. Padded? ${current_ifm_access_is_pad}. IFM Addr Sent: ${io.ifm_read_addr}\n")
        }
      } .otherwise { // Second cycle of sFetchIFM: latch data
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
      when(!data_fetch_cycle) { // First cycle: calculate address
        io.kernel_read_addr := k_r * config.kernelCols.U + k_c
        data_fetch_cycle := true.B
      } .otherwise { // Second cycle: latch data
        kernel_data_reg := io.kernel_read_data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      val product = (ifm_data_reg * kernel_data_reg).asSInt
      accumulator := accumulator + product

      val k_c_is_last = (k_c === (config.kernelCols.U - 1.U))
      when(k_c_is_last) {
        k_c := 0.U
        val k_r_is_last = (k_r === (config.kernelRows.U - 1.U))
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
      io.ofm_write_en   := true.B
      // io.ofm_write_addr is connected to: out_r * config.ifmCols.U + out_c
      accumulator := 0.S // Reset accumulator for the next OFM pixel

      // Advance OFM iterators (out_c, then out_r) up to IFM dimensions
      val out_c_is_last = (out_c === (config.ifmCols.U - 1.U))
      when(out_c_is_last) {
        out_c := 0.U
        val out_r_is_last = (out_r === (config.ifmRows.U - 1.U))
        when(out_r_is_last) {
          state := sDoneCU
        } .otherwise {
          out_r := out_r + 1.U
          state := sFetchIFM
          k_r := 0.U // Reset kernel iterators for new OFM pixel
          k_c := 0.U
        }
      } .otherwise {
        out_c := out_c + 1.U
        state := sFetchIFM
        k_r := 0.U // Reset kernel iterators for new OFM pixel
        k_c := 0.U
      }
      data_fetch_cycle := false.B // Ensure correct entry into sFetchIFM
    }

    is(sDoneCU) {
      when(!io.start) {
        state := sIdle
      }
    }
  }
  if (enableDebugPrints.litToBoolean) {
    printf(p"RoCC DUT Cycle[${dutCycleCount}]: -------------------- Cycle End ----------------------\n")
  }
}