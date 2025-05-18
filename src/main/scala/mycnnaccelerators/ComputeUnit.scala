// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool()) // Start computation signal from CNNController
  val busy  = Output(Bool()) // Accelerator is busy computing
  val done  = Output(Bool())  // Computation finished successfully
  val error = Output(Bool()) // Error during computation (e.g., overflow if checked)

  // Configuration for the current convolution
  val kernel_dim = Input(UInt(log2Ceil(config.kernelMaxRows + 1).W)) // K (e.g., 3 for 3x3, 5 for 5x5)
  // Stride is assumed to be 1. Padding is calculated to maintain OFM size = IFM size.

  // Scratchpad Read Interfaces (ComputeUnit requests data)
  val ifm_read_req    = Output(new ScratchpadReadIO(log2Ceil(config.ifmDepth), config.ifmDataType.dataWidth))
  val kernel_read_req = Output(new ScratchpadReadIO(log2Ceil(config.kernelMaxDepth), config.kernelDataType.dataWidth))

  // Scratchpad Write Interface (ComputeUnit writes OFM data)
  val ofm_write_req   = Output(new ScratchpadWriteIO(log2Ceil(config.ofmDepth), config.ofmDataType.dataWidth))
}

class ComputeUnit(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitIO(config))

  // Define States for the FSM
  val sIdle :: sSetup :: sCalcOFMRow :: sCalcOFMCol_NewElement :: sReqIFMKernel :: sWaitForData :: sMAC :: sWriteOFM :: sDone :: sErrorState :: Nil = Enum(10)
  val state = RegInit(sIdle)

  // Registers for loop counters, configuration, and accumulator
  val K = Reg(UInt(log2Ceil(config.kernelMaxRows + 1).W)) // Current kernel dimension (e.g., 3 or 5)
  val padding = Reg(UInt(log2Ceil(config.kernelMaxRows).W)) // Calculated padding, e.g., (K-1)/2

  val ofm_row_idx = RegInit(0.U(log2Ceil(config.ofmRows).W))
  val ofm_col_idx = RegInit(0.U(log2Ceil(config.ofmCols).W))
  val k_row_idx   = RegInit(0.U(log2Ceil(config.kernelMaxRows).W))
  val k_col_idx   = RegInit(0.U(log2Ceil(config.kernelMaxCols).W))

  // Accumulator, using SInt for signed fixed-point arithmetic
  val accumulator = RegInit(0.S(config.accumulatorWidth.W))

  // Registers to hold data read from scratchpad (due to SyncReadMem latency)
  val ifm_data_reg   = Reg(UInt(config.ifmDataType.dataWidth.W))
  val kernel_data_reg = Reg(UInt(config.kernelDataType.dataWidth.W))
  val ifm_val_is_zero_padding = Reg(Bool()) // True if the current IFM value should be from zero padding


  // Default outputs
  io.busy  := (state =/= sIdle) && (state =/= sDone) && (state =/= sErrorState)
  io.done  := (state === sDone)
  io.error := (state === sErrorState)

  io.ifm_read_req.en     := false.B
  io.ifm_read_req.addr   := 0.U
  io.kernel_read_req.en  := false.B
  io.kernel_read_req.addr:= 0.U
  io.ofm_write_req.en    := false.B
  io.ofm_write_req.addr  := 0.U
  io.ofm_write_req.data  := accumulator.asUInt // Truncate/saturate if accumulatorWidth > ofmDataType.dataWidth

  // Fixed-point arithmetic helper functions (assuming Q point is config.ifmDataType.fractionBits)
  // These are placeholders. Proper fixed-point libraries or more detailed implementations are needed for accuracy.
  def fixed_mul(a: SInt, b: SInt): SInt = {
    // (a * b) >> fractionBits. For Q8.8 * Q8.8 = Q16.16, then shift right by 8 to get Q16.8
    // Ensure intermediate multiplication doesn't overflow before shift
    val product =Wire(SInt((a.getWidth + b.getWidth).W))
    product := a * b
    (product >> config.ifmDataType.fractionBits).asSInt // Assuming IFM, Kernel, OFM have same fractionBits
  }

  def fixed_add(a: SInt, b: SInt): SInt = {
    a + b
  }


  switch(state) {
    is(sIdle) {
      when(io.start) {
        state := sSetup
      }
    }
    is(sSetup) {
      K             := io.kernel_dim
      padding       := (io.kernel_dim - 1.U) / 2.U // (K-1)/2 for 'same' convolution with stride 1
      ofm_row_idx   := 0.U
      ofm_col_idx   := 0.U
      state         := sCalcOFMRow
    }

    is(sCalcOFMRow) { // Iterate over OFM rows
      when(ofm_row_idx < config.ofmRows.U) {
        ofm_col_idx := 0.U
        state       := sCalcOFMCol_NewElement
      } .otherwise {
        state := sDone // All OFM rows processed
      }
    }

    is(sCalcOFMCol_NewElement) { // Iterate over OFM columns, start new OFM element
      when(ofm_col_idx < config.ofmCols.U) {
        accumulator := 0.S          // Reset accumulator for new OFM element
        k_row_idx   := 0.U          // Reset kernel row counter
        k_col_idx   := 0.U          // Reset kernel col counter
        state       := sReqIFMKernel // Start MAC loop for this OFM element
      } .otherwise { // Current OFM row done
        ofm_row_idx := ofm_row_idx + 1.U
        state       := sCalcOFMRow // Move to next OFM row
      }
    }

    is(sReqIFMKernel) { // Request IFM and Kernel data for current MAC operation
      // Calculate IFM coordinates (accounting for padding)
      // Zext: zero-extend to signed. SInt for comparison with padding.
      val ifm_eff_row = ofm_row_idx.zext + k_row_idx.zext - padding.zext
      val ifm_eff_col = ofm_col_idx.zext + k_col_idx.zext - padding.zext

      // Check if the required IFM element is within bounds (not padding)
      val is_padding_access = ifm_eff_row < 0.S || ifm_eff_row >= config.ifmRows.S ||
                              ifm_eff_col < 0.S || ifm_eff_col >= config.ifmCols.S

      ifm_val_is_zero_padding := is_padding_access

      when(!is_padding_access) {
        io.ifm_read_req.en   := true.B
        io.ifm_read_req.addr := ifm_eff_row.asUInt * config.ifmCols.U + ifm_eff_col.asUInt // Linear address
      } .otherwise {
        io.ifm_read_req.en   := false.B // Don't read if padding
      }

      // Request Kernel data
      io.kernel_read_req.en  := true.B
      io.kernel_read_req.addr:= k_row_idx * K + k_col_idx // Linear address for kernel

      state := sWaitForData // Next state: wait for SyncReadMem data
    }

    is(sWaitForData) {
      // Data from IFM (if not padding) and Kernel will be available on respective *.data ports THIS cycle
      // because the read was asserted in the PREVIOUS cycle (sReqIFMKernel).
      // We capture them into registers.
      // The actual io.ifm_read_req.data is used directly in MyCNNRoCC to connect to SPAD output.
      // Here, ComputeUnit expects the connected data to be valid.
      ifm_data_reg    := io.ifm_read_req.data // This is actually SPAD output connected to CU input
      kernel_data_reg := io.kernel_read_req.data // This is actually SPAD output connected to CU input

      // De-assert read enables for next cycle unless specifically needed
      io.ifm_read_req.en   := false.B
      io.kernel_read_req.en  := false.B
      state := sMAC
    }

    is(sMAC) { // Perform Multiply-Accumulate
      val current_ifm_val_sint = Mux(ifm_val_is_zero_padding,
                                     0.S(config.ifmDataType.dataWidth.W),
                                     ifm_data_reg.asSInt)
      val current_kernel_val_sint = kernel_data_reg.asSInt

      accumulator := fixed_add(accumulator, fixed_mul(current_ifm_val_sint, current_kernel_val_sint))

      // Increment kernel column
      k_col_idx := k_col_idx + 1.U
      when(k_col_idx + 1.U === K) { // Current kernel row finished
        k_col_idx := 0.U
        k_row_idx := k_row_idx + 1.U
        when(k_row_idx + 1.U === K) { // All kernel elements processed for this OFM
          state := sWriteOFM
        } .otherwise { // Next kernel row
          state := sReqIFMKernel
        }
      } .otherwise { // Next kernel column
        state := sReqIFMKernel
      }
    }

    is(sWriteOFM) { // Write accumulated OFM element to Scratchpad
      io.ofm_write_req.en   := true.B
      io.ofm_write_req.addr := ofm_row_idx * config.ofmCols.U + ofm_col_idx
      // Data conversion: accumulator (SInt) to OFM data type (UInt)
      // This needs proper handling of fixed-point format (saturation, truncation/rounding)
      // For simplicity, direct cast, assuming ofmDataType.dataWidth can hold the relevant part.
      val result_to_write = Wire(SInt(config.ofmDataType.dataWidth.W))
      // Example: if accumulator is Q16.16 and OFM is Q8.8, shift and truncate.
      // This depends on your exact fixed-point representation.
      // Assuming accumulator is already in the correct fixed point format for OFM, just possibly wider.
      // Taking lower bits for simplicity:
      result_to_write := accumulator(config.ofmDataType.dataWidth - 1, 0).asSInt
      io.ofm_write_req.data := result_to_write.asUInt

      // Next OFM element
      ofm_col_idx := ofm_col_idx + 1.U
      state       := sCalcOFMCol_NewElement // Go to process next OFM col or next OFM row
    }

    is(sDone) {
      when(!io.start) { // Wait for start to go low before returning to Idle (optional debounce)
        state := sIdle
      }
    }
    is(sErrorState) {
      // Stay here until reset or a specific clear command (not implemented)
    }
  }
}