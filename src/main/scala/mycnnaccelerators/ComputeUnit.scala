// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())

  // IFM Buffer Interface (CU reads from IFM buffer)
  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W))
  val ifm_read_data = Input(UInt(config.dataWidth.W)) // Data from IFM buffer

  // Kernel Buffer Interface (CU reads from Kernel buffer)
  val kernel_read_addr = Output(UInt(config.kernelAddrWidth.W))
  val kernel_read_data = Input(UInt(config.dataWidth.W)) // Data from Kernel buffer

  // OFM Buffer Interface (CU writes to OFM buffer)
  val ofm_write_en   = Output(Bool())
  val ofm_write_addr = Output(UInt(config.ofmAddrWidth.W))
  val ofm_write_data = Output(UInt(config.dataWidth.W))
}

class ComputeUnit(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitIO(config))

  // States for the Compute Unit FSM
  val sIdle :: sFetchIFM :: sFetchKernel :: sMAC :: sWriteOFM :: sDoneCU :: Nil = Enum(6)
  val state = RegInit(sIdle)

  // Registers for loop counters and data
  val ofm_row = RegInit(0.U(log2Ceil(config.ofmRows).W))
  val ofm_col = RegInit(0.U(log2Ceil(config.ofmCols).W))

  val k_row = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_col = RegInit(0.U(log2Ceil(config.kernelCols).W))

  // Accumulator - make it wider to prevent overflow during MAC
  val accWidth = config.dataWidth * 2 + log2Ceil(config.kernelRows * config.kernelCols)
  val accumulator = RegInit(0.S(accWidth.W)) // Signed for potential future use, works for UInt too

  // Registers to hold data read from buffers (due to SyncReadMem 1-cycle latency)
  val ifm_data_reg   = Reg(UInt(config.dataWidth.W))
  val kernel_data_reg = Reg(UInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B) // To manage 2-cycle read

  // Default outputs
  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)

  io.ifm_read_addr := 0.U
  io.kernel_read_addr := 0.U

  io.ofm_write_en   := false.B
  io.ofm_write_addr := 0.U
  io.ofm_write_data := accumulator(config.dataWidth - 1, 0).asUInt // Truncate/saturate as needed

  switch(state) {
    is(sIdle) {
      when(io.start) {
        ofm_row := 0.U
        ofm_col := 0.U
        k_row   := 0.U
        k_col   := 0.U
        accumulator := 0.S
        state := sFetchIFM
        data_fetch_cycle := false.B // Start fetch cycle 0 (addressing)
      }
    }

    is(sFetchIFM) {
      // Cycle 0: Assert address for IFM data
      // Cycle 1: Latch data, assert address for Kernel data
      when(!data_fetch_cycle) { // Cycle 0
        val current_ifm_row = ofm_row + k_row
        val current_ifm_col = ofm_col + k_col
        io.ifm_read_addr := current_ifm_row * config.ifmCols.U + current_ifm_col
        data_fetch_cycle := true.B
        // Stay in sFetchIFM for next cycle to latch data
      } .otherwise { // Cycle 1
        ifm_data_reg := io.ifm_read_data // Latch IFM data from previous cycle's address
        state := sFetchKernel
        data_fetch_cycle := false.B // Reset for kernel fetch
      }
    }

    is(sFetchKernel) {
      // Cycle 0: Assert address for Kernel data
      // Cycle 1: Latch data, proceed to MAC
      when(!data_fetch_cycle) { // Cycle 0
        io.kernel_read_addr := k_row * config.kernelCols.U + k_col
        data_fetch_cycle := true.B
      } .otherwise { // Cycle 1
        kernel_data_reg := io.kernel_read_data // Latch Kernel data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      accumulator := accumulator + (ifm_data_reg.zext * kernel_data_reg.zext).asSInt

      // Increment kernel iterators
      k_col := k_col + 1.U
      when(k_col + 1.U === config.kernelCols.U) {
        k_col := 0.U
        k_row := k_row + 1.U
        when(k_row + 1.U === config.kernelRows.U) {
          k_row := 0.U
          state := sWriteOFM // All kernel elements processed for this OFM pixel
        } .otherwise {
          state := sFetchIFM // Next row in kernel
        }
      } .otherwise {
        state := sFetchIFM // Next col in kernel
      }
    }

    is(sWriteOFM) {
      io.ofm_write_en   := true.B
      io.ofm_write_addr := ofm_row * config.ofmCols.U + ofm_col
      // Data is set by default connection. Ensure accumulator is appropriately sized and truncated.

      accumulator := 0.S // Reset accumulator for next OFM element

      // Increment OFM iterators
      ofm_col := ofm_col + 1.U
      when(ofm_col + 1.U === config.ofmCols.U) {
        ofm_col := 0.U
        ofm_row := ofm_row + 1.U
        when(ofm_row + 1.U === config.ofmRows.U) {
          state := sDoneCU // All OFM pixels computed
        } .otherwise {
          k_row := 0.U // Reset kernel iterators for new OFM element
          k_col := 0.U
          state := sFetchIFM // Next row in OFM
        }
      } .otherwise {
        k_row := 0.U // Reset kernel iterators for new OFM element
        k_col := 0.U
        state := sFetchIFM // Next col in OFM
      }
    }

    is(sDoneCU) {
      when(!io.start) { // Optional: wait for start to de-assert before idling
        state := sIdle
      }
    }
  }
}