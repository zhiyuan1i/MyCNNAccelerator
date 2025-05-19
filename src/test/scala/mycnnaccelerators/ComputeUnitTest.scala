// filename: ComputeUnitTest.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import chisel3.simulator.EphemeralSimulator._
import org.scalatest.flatspec.AnyFlatSpec

// FixedPointUtils object (as provided before)
object FixedPointUtils {
  def floatToFixed(value: Double, fBits: Int): BigInt = {
    (value * (1L << fBits)).round
  }
  def fixedToFloat(sIntValue: BigInt, fBits: Int): Double = {
    sIntValue.doubleValue / (1L << fBits)
  }
}

class ComputeUnitTest extends AnyFlatSpec {
  import FixedPointUtils._

  def referenceConv(
    ifm: Seq[Seq[Double]],
    kernel: Seq[Seq[Double]],
    config: AcceleratorConfig
  ): Seq[Seq[Double]] = {
    // ... (referenceConv implementation remains the same as provided before)
    val ofm = Array.ofDim[Double](config.ofmRows, config.ofmCols)
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
        val accIntValueFullFraction = (accumulatorDouble * (1L << (2 * config.F_BITS))).round
        val shiftedAccIntValue = accIntValueFullFraction >> config.F_BITS
        val bitmask = (BigInt(1) << config.dataWidth) - 1
        var finalSIntValue = shiftedAccIntValue & bitmask
        if ((finalSIntValue & (BigInt(1) << (config.dataWidth - 1))) != 0) {
          finalSIntValue = finalSIntValue - (BigInt(1) << config.dataWidth)
        }
        ofm(r_ofm)(c_ofm) = fixedToFloat(finalSIntValue, config.F_BITS)
      }
    }
    ofm.map(_.toSeq).toSeq
  }

  behavior of "ComputeUnit with Buffers"

  it should "perform a 3x3 IFM, 2x2 Kernel convolution correctly with F_BITS = 4" in {
    val testConfig = AcceleratorConfig(
      dataWidth = 16,
      F_BITS = 8,
      ifmRows = 3, ifmCols = 3,
      kernelRows = 2, kernelCols = 2,
      xLen = 64
    )

    val ifm_double = Seq(
      Seq(1.0,  2.0,  0.5),
      Seq(3.0,  1.5,  1.0),
      Seq(0.25, 2.5,  0.75)
    )
    val ifm_fixed_flat = ifm_double.flatten.map(d => floatToFixed(d, testConfig.F_BITS)).toArray

    val kernel_double = Seq(
      Seq(0.5, -1.0),
      Seq(1.0,  0.25)
    )
    val kernel_fixed_flat = kernel_double.flatten.map(d => floatToFixed(d, testConfig.F_BITS)).toArray

    val expected_ofm_double = referenceConv(ifm_double, kernel_double, testConfig)

    // Simulate the wrapper module
    simulate(new ComputeUnitSystemWrapper(testConfig)) { dut =>
      var cycles = 0

      // --- Phase 1: Pre-load IFM buffer ---
      // In ComputeUnitTest.scala, during IFM buffer pre-loading
        println("TB: Pre-loading IFM buffer...")
        dut.io.ifm_buffer_write_en.poke(true.B)
        for (i <- 0 until ifm_fixed_flat.length) {
        val data_to_write = ifm_fixed_flat(i)
        dut.io.ifm_buffer_write_addr.poke(i.U)
        dut.io.ifm_buffer_write_data.poke(data_to_write.S(testConfig.dataWidth.W))
        // Add this print:
        println(s"TB PRELOAD IFM: Cycle $cycles, Writing to Addr: $i, Data: $data_to_write (Expected IFM[$i])")
        dut.clock.step(1)
        cycles += 1
        }
        dut.io.ifm_buffer_write_en.poke(false.B)

      // --- Phase 2: Pre-load Kernel buffer ---
      println("TB: Pre-loading Kernel buffer...")
      dut.io.kernel_buffer_write_en.poke(true.B)
      for (i <- 0 until kernel_fixed_flat.length) {
        dut.io.kernel_buffer_write_addr.poke(i.U)
        dut.io.kernel_buffer_write_data.poke(kernel_fixed_flat(i).S(testConfig.dataWidth.W))
        // println(s"TB Cycle $cycles: Writing to KERNEL_BUFFER Addr: $i, Data: ${kernel_fixed_flat(i)}")
        dut.clock.step(1)
        cycles += 1
      }
      dut.io.kernel_buffer_write_en.poke(false.B)

      // Ensure other control signals are initially de-asserted
      dut.io.ofm_buffer_read_addr.poke(0.U)
      dut.io.start.poke(false.B)
      dut.clock.step(1) // Let signals settle
      cycles +=1

      // --- Phase 3: Start ComputeUnit and run to completion ---
      println(s"TB Cycle $cycles: Starting ComputeUnit...")
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      cycles += 1
      dut.io.start.poke(false.B)

      val maxComputeCycles = testConfig.ofmRows * testConfig.ofmCols * (testConfig.kernelRows * testConfig.kernelCols * 3 + 10) + 50
      var computeCycles = 0
      while (dut.io.done.peek().litValue != BigInt(1) && computeCycles < maxComputeCycles) {
        // println(s"TB Cycle $cycles: ComputeUnit running... (computeCycle $computeCycles)")
        dut.clock.step(1)
        cycles += 1
        computeCycles += 1
      }
      assert(computeCycles < maxComputeCycles, "ComputeUnit simulation timed out.")
      assert(dut.io.done.peek().litValue == BigInt(1), "ComputeUnit did not signal done.")
      println(s"TB Cycle $cycles: ComputeUnit finished in $computeCycles compute cycles.")

      // --- Phase 4: Read OFM buffer and verify results ---
      println("TB: Reading OFM buffer results...")
      val actual_ofm_map = collection.mutable.Map[(Int, Int), BigInt]()
      for (i <- 0 until testConfig.ofmDepth) {
        dut.io.ofm_buffer_read_addr.poke(i.U)
        // Data is valid on ofm_buffer_read_data in THIS cycle due to RegNext in MinimalBuffer
        // if read_addr was set in previous cycle's end or this cycle's beginning.
        // To be safe: set addr, step, then peek.
        dut.clock.step(1) // Allow combinational path for address to propagate, and RegNext to shift.
        cycles += 1
        val data = dut.io.ofm_buffer_read_data.peek().litValue
        // println(s"TB Cycle $cycles: Reading from OFM_BUFFER Addr: $i, Data: $data")
        
        val r_ofm = i / testConfig.ofmCols
        val c_ofm = i % testConfig.ofmCols
        actual_ofm_map((r_ofm, c_ofm)) = data
      }
      
      // De-assert read address after reading
      // dut.io.ofm_buffer_read_addr.poke(0.U) // Or let it be
      // dut.clock.step(1)
      // cycles +=1


      println(s"Total simulation cycles: $cycles.")
      println("Verifying OFM results:")
      var mismatches = 0
      for (r <- 0 until testConfig.ofmRows) {
        for (c <- 0 until testConfig.ofmCols) {
          val actual_fixed = actual_ofm_map.getOrElse((r, c), BigInt(0))
          val actual_double = fixedToFloat(actual_fixed, testConfig.F_BITS)
          val expected_double = expected_ofm_double(r)(c)
          
          print(f"OFM($r%d,$c%d): Actual=${actual_double}%9.4f (Fixed:$actual_fixed%d), Expected=${expected_double}%9.4f. ")
          
          val tolerance = 1.0 / (1L << testConfig.F_BITS)
          if (Math.abs(actual_double - expected_double) < tolerance) {
            println("Match!")
          } else {
            println(f"Mismatch! Diff: ${Math.abs(actual_double - expected_double)}%.4f")
            mismatches +=1
          }
        }
      }
      assert(mismatches == 0, s"$mismatches OFM value mismatches found. With buffers, error is likely in CU logic or reference model.")
    }
  }
}