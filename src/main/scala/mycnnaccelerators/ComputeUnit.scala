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