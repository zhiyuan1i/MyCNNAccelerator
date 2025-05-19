package mycnnaccelerators

import chisel3._
import chisel3.util._
import chisel3.simulator.EphemeralSimulator._
import org.scalatest.flatspec.AnyFlatSpec
// Please do not use chiseltest has been archived and is not recommended for new projects.

case class AcceleratorConfig(
  dataWidth: Int,
  ifmRows: Int, ifmCols: Int,
  kernelRows: Int, kernelCols: Int,
  xLen: Int
) {
  val ofmRows: Int = ifmRows - kernelRows + 1
  val ofmCols: Int = ifmCols - kernelCols + 1
}

class ComputeUnit(config: AcceleratorConfig) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val done = Output(Bool())
    val busy = Output(Bool())
    val ifm_read_addr = Output(UInt(log2Ceil(config.ifmRows * config.ifmCols).W))
    val ifm_read_data = Input(SInt(config.dataWidth.W))
    val kernel_read_addr = Output(UInt(log2Ceil(config.kernelRows * config.kernelCols).W))
    val kernel_read_data = Input(SInt(config.dataWidth.W))
    val ofm_write_en = Output(Bool())
    val ofm_write_addr = Output(UInt(log2Ceil(config.ofmRows * config.ofmCols).W))
    val ofm_write_data = Output(SInt(config.dataWidth.W))
  })

  val s_idle :: s_busy :: s_done :: Nil = chisel3.util.Enum(3)
  val state = RegInit(s_idle)

  io.busy := (state === s_busy)
  io.done := (state === s_done)

  io.ifm_read_addr := 0.U
  io.kernel_read_addr := 0.U
  io.ofm_write_en := false.B
  io.ofm_write_addr := 0.U
  io.ofm_write_data := 0.S

  switch(state) {
    is(s_idle) {
      when(io.start) { state := s_busy }
    }
    is(s_busy) {
      when(RegNext(io.start)) { state := s_done }
    }
    is(s_done) { /* Stays done */ }
  }
}


class ComputeUnitTest extends AnyFlatSpec {

  val F_BITS_CU = 8
  def toFixed(d: Double): BigInt = (d * (1L << F_BITS_CU)).round
  def toDouble(sIntValue: BigInt): Double = {
    sIntValue.doubleValue / (1L << F_BITS_CU)
  }

  def referenceConv(
    ifm: Seq[Seq[Double]],
    kernel: Seq[Seq[Double]],
    config: AcceleratorConfig
  ): Seq[Seq[Double]] = {
    val ofm = Array.ofDim[Double](config.ofmRows, config.ofmCols)
    val ACC_FRAC_BITS = config.dataWidth
    for (r_ofm <- 0 until config.ofmRows) {
      for (c_ofm <- 0 until config.ofmCols) {
        var accumulatorDouble: Double = 0.0
        for (r_k <- 0 until config.kernelRows) {
          for (c_k <- 0 until config.kernelCols) {
            val r_ifm = r_ofm + r_k
            val c_ifm = c_ofm + c_k
            if (r_ifm < config.ifmRows && c_ifm < config.ifmCols) {
                 accumulatorDouble += ifm(r_ifm)(c_ifm) * kernel(r_k)(c_k)
            }
          }
        }
        val accFixedInternal = (accumulatorDouble * (1L << ACC_FRAC_BITS)).round
        val shiftAmount = ACC_FRAC_BITS - F_BITS_CU
        val shiftedAcc = if (shiftAmount >= 0) accFixedInternal >> shiftAmount else accFixedInternal << -shiftAmount
        var finalSIntValue = shiftedAcc & ((BigInt(1) << config.dataWidth) - 1)
        if ((finalSIntValue & (BigInt(1) << (config.dataWidth - 1))) != 0) {
          finalSIntValue = finalSIntValue - (BigInt(1) << config.dataWidth)
        }
        ofm(r_ofm)(c_ofm) = toDouble(finalSIntValue)
      }
    }
    ofm.map(_.toSeq).toSeq
  }

  behavior of "ComputeUnit"

  it should "perform a 3x3 IFM, 2x2 Kernel convolution correctly" in {
    val testConfig = AcceleratorConfig(
      dataWidth = 16,
      ifmRows = 3, ifmCols = 3,
      kernelRows = 2, kernelCols = 2,
      xLen = 64
    )

    val ifm_double = Seq(Seq(1.0, 2.0, 0.5), Seq(3.0, 1.5, 1.0), Seq(0.25, 2.5, 0.75))
    val ifm_fixed_flat = ifm_double.flatten.map(d => toFixed(d)).toArray
    val kernel_double = Seq(Seq(0.5, -1.0), Seq(1.0, 0.25))
    val kernel_fixed_flat = kernel_double.flatten.map(d => toFixed(d)).toArray
    val expected_ofm_double = referenceConv(ifm_double, kernel_double, testConfig)

    simulate(new ComputeUnit(testConfig)) { dut =>
      dut.io.start.poke(false)
      dut.io.ifm_read_data.poke(BigInt(0))
      dut.io.kernel_read_data.poke(BigInt(0))
      dut.clock.step(1)

      dut.io.start.poke(true)
      dut.clock.step(1)
      dut.io.start.poke(false)

      val maxCycles = testConfig.ifmRows * testConfig.ifmCols * testConfig.kernelRows * testConfig.kernelCols *
                      testConfig.ofmRows * testConfig.ofmCols + 200
      var cycles = 0
      val actual_ofm_map = collection.mutable.Map[(Int, Int), BigInt]()
      var prev_ifm_addr: Option[Int] = None
      var prev_kernel_addr: Option[Int] = None

      println("Starting ChiselSim test loop...")
      // Corrected: API for peeking values with EphemeralSimulator
      // For Bool: .peek().litValue == BigInt(1) (or != 0.BigInt)
      // For UInt/SInt: .peek().litValue
      while (dut.io.done.peek().litValue != BigInt(1) && cycles < maxCycles) {
        prev_ifm_addr match {
          case Some(addr) if addr >= 0 && addr < ifm_fixed_flat.length =>
            dut.io.ifm_read_data.poke(ifm_fixed_flat(addr))
          case _ => dut.io.ifm_read_data.poke(BigInt(0))
        }
        prev_kernel_addr match {
          case Some(addr) if addr >= 0 && addr < kernel_fixed_flat.length =>
            dut.io.kernel_read_data.poke(kernel_fixed_flat(addr))
          case _ => dut.io.kernel_read_data.poke(BigInt(0))
        }

        if (dut.io.ofm_write_en.peek().litValue == BigInt(1)) {
          val ofm_addr = dut.io.ofm_write_addr.peek().litValue.toInt
          val ofm_data_raw = dut.io.ofm_write_data.peek().litValue
          
          val r_ofm = ofm_addr / testConfig.ofmCols
          val c_ofm = ofm_addr % testConfig.ofmCols
          actual_ofm_map((r_ofm, c_ofm)) = ofm_data_raw
        }

        dut.clock.step(1)

        prev_ifm_addr = Some(dut.io.ifm_read_addr.peek().litValue.toInt)
        prev_kernel_addr = Some(dut.io.kernel_read_addr.peek().litValue.toInt)
        
        cycles += 1
      }

      println(s"ChiselSim simulation finished in $cycles cycles.")
      assert(cycles < maxCycles, "Simulation timed out.")
      assert(dut.io.done.peek().litValue == BigInt(1), "DUT did not signal done at the end.")

      println("Verifying OFM results:")
      var mismatches = 0
      for (r <- 0 until testConfig.ofmRows) {
        for (c <- 0 until testConfig.ofmCols) {
          val actual_fixed = actual_ofm_map.getOrElse((r, c), BigInt(0))
          val actual_double = toDouble(actual_fixed)
          val expected_double = expected_ofm_double(r)(c)
          print(f"OFM($r%d,$c%d): Actual=${actual_double}%6.4f (Fixed:$actual_fixed%d), Expected=${expected_double}%6.4f. ")
          val tolerance = 1.0 / (1L << (F_BITS_CU - 1))
          if (Math.abs(actual_double - expected_double) < tolerance) {
            println("Match!")
          } else {
            println(f"Mismatch! Diff: ${Math.abs(actual_double - expected_double)}%.4f")
            mismatches +=1
          }
        }
      }
      assert(mismatches == 0, s"$mismatches OFM value mismatches found.")
    }
  }
}