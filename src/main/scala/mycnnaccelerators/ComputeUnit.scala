// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())

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

  val sIdle :: sFetchIFM :: sFetchKernel :: sMAC :: sWriteOFM :: sDoneCU :: Nil = Enum(6)
  val state = RegInit(sIdle)

  // Determine maximum possible width for sums
  val maxIfmRowCount = config.ifmRows -1 // Max value of ofm_row or k_row (e.g. ifmRows=3 -> max index 2)
  val maxIfmColCount = config.ifmCols -1
  val maxKernelRowCount = config.kernelRows -1
  val maxKernelColCount = config.kernelCols -1
  
  // Max sum for row indices: (max ofm_r + max k_r)
  // ofm_r max index is config.ofmRows - 1
  // k_r max index is config.kernelRows - 1
  // Max value of ofm_row + k_row is (config.ofmRows - 1) + (config.kernelRows - 1)
  val curRowSumWidth = log2Ceil((config.ofmRows - 1) + (config.kernelRows - 1) + 1) // 应该是2
  val curColSumWidth = log2Ceil((config.ofmCols - 1) + (config.kernelCols - 1) + 1)


  val ofm_row = RegInit(0.U(log2Ceil(config.ofmRows).W))
  val ofm_col = RegInit(0.U(log2Ceil(config.ofmCols).W))
  val k_row   = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_col   = RegInit(0.U(log2Ceil(config.kernelCols).W))

  val guardBits = log2Ceil(config.kernelRows * config.kernelCols)
  val accWidth = (2 * config.dataWidth) + guardBits
  val accumulator = RegInit(0.S(accWidth.W))

  val ifm_data_reg    = Reg(SInt(config.dataWidth.W))
  val kernel_data_reg = Reg(SInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B)

  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)

  io.ifm_read_addr   := 0.U
  io.kernel_read_addr:= 0.U
  io.ofm_write_en    := false.B
  io.ofm_write_addr  := 0.U

  val shifted_accumulator = (accumulator >> config.F_BITS)
  io.ofm_write_data  := shifted_accumulator(config.dataWidth - 1, 0).asSInt

  val enableDebugPrints = false.B
  if (enableDebugPrints.litToBoolean) {
    printf(p"DUT Cycle[${dutCycleCount}]: -------------------- Cycle Start --------------------\n")
    printf(p"DUT Cycle[${dutCycleCount}]: State: ${state} (Idle=${sIdle}, FetchIFM=${sFetchIFM}, FetchKernel=${sFetchKernel}, MAC=${sMAC}, WriteOFM=${sWriteOFM}, DoneCU=${sDoneCU})\n")
    printf(p"DUT Cycle[${dutCycleCount}]: data_fetch_cycle: ${data_fetch_cycle}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Counters: ofm_r=${ofm_row}, ofm_c=${ofm_col}, k_r=${k_row}, k_c=${k_col}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Accumulator: ${accumulator} (decimal), 0x${Hexadecimal(accumulator)}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Shifted Accumulator (for OFM): ${shifted_accumulator} (decimal), 0x${Hexadecimal(shifted_accumulator)}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Inputs: start=${io.start}, ifm_read_data=${io.ifm_read_data}, kernel_read_data=${io.kernel_read_data}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Internal Regs: ifm_data_reg=${ifm_data_reg}, kernel_data_reg=${kernel_data_reg}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Outputs: ifm_addr=${io.ifm_read_addr}, kernel_addr=${io.kernel_read_addr}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Outputs: ofm_wr_en=${io.ofm_write_en}, ofm_wr_addr=${io.ofm_write_addr}, ofm_data=${io.ofm_write_data} (decimal), 0x${Hexadecimal(io.ofm_write_data)}\n")
    printf(p"DUT Cycle[${dutCycleCount}]: Outputs: busy=${io.busy}, done=${io.done}\n")
  }

  switch(state) {
    is(sIdle) {
      when(io.start) {
        ofm_row   := 0.U
        ofm_col   := 0.U
        k_row     := 0.U
        k_col     := 0.U
        accumulator := 0.S
        state       := sFetchIFM
        data_fetch_cycle := false.B
      }
    }

    is(sFetchIFM) {
      when(!data_fetch_cycle) {
        // Explicitly create Wires for sums to ensure correct width before multiplication
        val widened_ofm_row = Cat(0.U(1.W), ofm_row) // 将 UInt(1.W) 扩展为 UInt(2.W)
        val widened_k_row   = Cat(0.U(1.W), k_row)   // 将 UInt(1.W) 扩展为 UInt(2.W)
        val current_ifm_row_sum = widened_ofm_row + widened_k_row // UInt(2.W) + UInt(2.W) 结果是 UInt(3.W) 或者能容纳2的UInt(2.W)

        val widened_ofm_col = Cat(0.U(1.W), ofm_col)
        val widened_k_col   = Cat(0.U(1.W), k_col)
        val current_ifm_col_sum = widened_ofm_col + widened_k_col
        
        val calculated_addr = current_ifm_row_sum * config.ifmCols.U + current_ifm_col_sum

        if (enableDebugPrints.litToBoolean) { // Conditional printf
            printf(p"DUT Cycle[${dutCycleCount}]: AddrCalc IFM: ofm_r=${ofm_row}, k_r=${k_row} => cur_r_sum=${current_ifm_row_sum}(W=${current_ifm_row_sum.getWidth}) | ofm_c=${ofm_col}, k_c=${k_col} => cur_c_sum=${current_ifm_col_sum}(W=${current_ifm_col_sum.getWidth}) | ifmCols=${config.ifmCols} | Result Addr=${calculated_addr}\n")
        }
        io.ifm_read_addr := calculated_addr
        data_fetch_cycle := true.B
      } .otherwise {
        ifm_data_reg := io.ifm_read_data
        state := sFetchKernel
        data_fetch_cycle := false.B
      }
    }

    is(sFetchKernel) {
      when(!data_fetch_cycle) {
        // Similar explicit Wires for kernel address calculation if needed, though it's simpler
        val current_kernel_row_val = k_row // Directly use, width is fine
        val current_kernel_col_val = k_col // Directly use, width is fine

        val calculated_addr = current_kernel_row_val * config.kernelCols.U + current_kernel_col_val
        if (enableDebugPrints.litToBoolean) { // Conditional printf
             printf(p"DUT Cycle[${dutCycleCount}]: AddrCalc Kernel: k_r=${k_row} => cur_r=${current_kernel_row_val} | k_c=${k_col} => cur_c=${current_kernel_col_val} | kernelCols=${config.kernelCols} | Result Addr=${calculated_addr}\n")
        }
        io.kernel_read_addr := calculated_addr
        data_fetch_cycle := true.B
      } .otherwise {
        kernel_data_reg := io.kernel_read_data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      val product = (ifm_data_reg.asSInt * kernel_data_reg.asSInt).asSInt
      accumulator := accumulator + product

      val k_col_is_last = (k_col === (config.kernelCols.U - 1.U))
      when(k_col_is_last) {
        k_col := 0.U
        val k_row_is_last = (k_row === (config.kernelRows.U - 1.U))
        when(k_row_is_last) {
          k_row := 0.U
          state := sWriteOFM
        } .otherwise {
          k_row := k_row + 1.U
          state := sFetchIFM
        }
      } .otherwise {
        k_col := k_col + 1.U
        state := sFetchIFM
      }
    }

    is(sWriteOFM) {
      io.ofm_write_en   := true.B
      io.ofm_write_addr := ofm_row * config.ofmCols.U + ofm_col
      accumulator := 0.S

      val ofm_col_is_last = (ofm_col === (config.ofmCols.U - 1.U))
      when(ofm_col_is_last) {
        ofm_col := 0.U
        val ofm_row_is_last = (ofm_row === (config.ofmRows.U - 1.U))
        when(ofm_row_is_last) {
          state := sDoneCU
        } .otherwise {
          ofm_row := ofm_row + 1.U
          state := sFetchIFM
          data_fetch_cycle := false.B
        }
      } .otherwise {
        ofm_col := ofm_col + 1.U
        state := sFetchIFM
        data_fetch_cycle := false.B
      }
    }

    is(sDoneCU) {
      when(!io.start) {
        state := sIdle
      }
    }
  }
  if (enableDebugPrints.litToBoolean) {
    printf(p"DUT Cycle[${dutCycleCount}]: -------------------- Cycle End ----------------------\n")
  }
}