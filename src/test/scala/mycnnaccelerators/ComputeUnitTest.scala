// filename: ComputeUnitTest.scala
package mycnnaccelerators

import chisel3._
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

object ReferenceConvolution {
  import FixedPointUtils._

  def referenceConvSame(
    ifm: Seq[Seq[Double]],
    kernel: Seq[Seq[Double]],
    config: AcceleratorConfig // This config's IFM/Kernel dimensions and F_BITS are used.
                              // Output is IFM-sized.
  ): Seq[Seq[Double]] = {
    val outputRows = config.ifmRows // 'Same' convolution output rows = IFM rows
    val outputCols = config.ifmCols // 'Same' convolution output cols = IFM cols
    val ofm = Array.ofDim[Double](outputRows, outputCols)

    val padTop = (config.kernelRows - 1) / 2
    val padLeft = (config.kernelCols - 1) / 2

    for (r_out <- 0 until outputRows) {
      for (c_out <- 0 until outputCols) {
        var accumulatorDouble: Double = 0.0
        for (r_k <- 0 until config.kernelRows) {
          for (c_k <- 0 until config.kernelCols) {
            val r_ifm_eff = r_out - padTop + r_k
            val c_ifm_eff = c_out - padLeft + c_k

            if (r_ifm_eff >= 0 && r_ifm_eff < config.ifmRows &&
                c_ifm_eff >= 0 && c_ifm_eff < config.ifmCols) {
              accumulatorDouble += ifm(r_ifm_eff)(c_ifm_eff) * kernel(r_k)(c_k)
            }
          }
        }

        // Fixed-point conversion for the accumulated sum
        // Product of two numbers with F_BITS fractional bits has 2*F_BITS fractional bits.
        // Accumulator holds sum of these products, still with 2*F_BITS fractional bits conceptually.
        val accIntValueFullFraction = (accumulatorDouble * (1L << (2 * config.F_BITS))).round
        // Shift right by F_BITS to get the OFM value with F_BITS fractional bits
        val shiftedAccIntValue = accIntValueFullFraction >> config.F_BITS
        // Truncate to dataWidth (this is what hardware typically does)
        val bitmask = (BigInt(1) << config.dataWidth) - 1
        var finalSIntValue = shiftedAccIntValue & bitmask
        // Sign extend if the MSB (of dataWidth) is set
        if ((finalSIntValue & (BigInt(1) << (config.dataWidth - 1))) != 0) {
          finalSIntValue = finalSIntValue - (BigInt(1) << config.dataWidth)
        }
        ofm(r_out)(c_out) = fixedToFloat(finalSIntValue, config.F_BITS)
      }
    }
    ofm.map(_.toSeq).toSeq
  }
}

class ComputeUnitTest extends AnyFlatSpec {
  import FixedPointUtils._
  import ReferenceConvolution.referenceConvSame

  behavior of "ComputeUnit with Buffers (Testing 'Same' Convolution)"

  def runConvolutionTest(
    testName: String,
    ifmDisplayRows: Int, ifmDisplayCols: Int,
    kernelDisplayRows: Int, kernelDisplayCols: Int,
    actualIfmRowsForDUT: Int, actualIfmColsForDUT: Int,
    actualKernelRowsForDUT: Int, actualKernelColsForDUT: Int,
    fBits: Int, dataWidth: Int = 16
  ): Unit = {

    it should s"perform $testName: ${actualIfmRowsForDUT}x${actualIfmColsForDUT} IFM, ${actualKernelRowsForDUT}x${actualKernelColsForDUT} Kernel, 'same' conv (F_BITS = $fBits)" in {
      // This dutConfig MUST now have its ofmRows, ofmCols, ofmDepth, and ofmAddrWidth
      // configured for 'same' convolution (i.e., OFM dims = IFM dims).
      // We assume AcceleratorConfig is modified to do this if isSameConvolution=true (or similar).
      val dutConfig = AcceleratorConfig(
        dataWidth = dataWidth,
        F_BITS = fBits,
        ifmRows = actualIfmRowsForDUT,
        ifmCols = actualIfmColsForDUT,
        kernelRows = actualKernelRowsForDUT,
        kernelCols = actualKernelColsForDUT,
        xLen = 64,
        isSameConvolution = true // Assuming AcceleratorConfig takes such a parameter
                                  // or calculates OFM dims for 'same' by default now.
      )

      // For 'same' convolution, output dimensions are same as input IFM dimensions
      val expectedOutputRows = dutConfig.ifmRows
      val expectedOutputCols = dutConfig.ifmCols
      // dutConfig.ofmDepth should now be expectedOutputRows * expectedOutputCols

      println(s"Starting test: $testName")
      println(s"DUT Config: IFM ${dutConfig.ifmRows}x${dutConfig.ifmCols}, Kernel ${dutConfig.kernelRows}x${dutConfig.kernelCols}")
      println(s"DUT Config: Expected OFM ${expectedOutputRows}x${expectedOutputCols} ('same' size), Expected OFM Depth ${dutConfig.ofmDepth}")
      println(s"DUT Config: F_BITS $fBits, DataWidth $dataWidth")

      val ifm_double = Seq.tabulate(actualIfmRowsForDUT, actualIfmColsForDUT) { (r, c) =>
        ((r % ifmDisplayRows) + (c % ifmDisplayCols) * 2.0 - (ifmDisplayRows + ifmDisplayCols*2.0)/2.0) / 3.0 + (r%3-1)*0.1
      }
      val ifm_fixed_flat = ifm_double.flatten.map(d => floatToFixed(d, dutConfig.F_BITS)).toArray

      val kernel_double = Seq.tabulate(actualKernelRowsForDUT, actualKernelColsForDUT) { (r, c) =>
        ((r % kernelDisplayRows) * 0.5 - (kernelDisplayRows*0.5)/2.0 + (c % kernelDisplayCols) * 0.3 - (kernelDisplayCols*0.3)/2.0) + 0.1
      }
      val kernel_fixed_flat = kernel_double.flatten.map(d => floatToFixed(d, dutConfig.F_BITS)).toArray

      val expected_ofm_same_full_double = referenceConvSame(ifm_double, kernel_double, dutConfig)
      println(s"Reference 'same' OFM calculated (${expected_ofm_same_full_double.length}x${expected_ofm_same_full_double.head.length})")

      simulate(new ComputeUnitSystemWrapper(dutConfig)) { dut =>
        var cycles = 0

        println(s"TB: Pre-loading IFM buffer (${ifm_fixed_flat.length} elements)...")
        dut.io.ifm_buffer_write_en.poke(true.B)
        assert(ifm_fixed_flat.length == dutConfig.ifmDepth, "IFM flat array size mismatch with config depth")
        for (i <- 0 until dutConfig.ifmDepth) {
          dut.io.ifm_buffer_write_addr.poke(i.U)
          dut.io.ifm_buffer_write_data.poke(ifm_fixed_flat(i).S(dutConfig.dataWidth.W))
          dut.clock.step(1); cycles += 1
        }
        dut.io.ifm_buffer_write_en.poke(false.B)

        println(s"TB: Pre-loading Kernel buffer (${kernel_fixed_flat.length} elements)...")
        dut.io.kernel_buffer_write_en.poke(true.B)
        assert(kernel_fixed_flat.length == dutConfig.kernelDepth, "Kernel flat array size mismatch with config depth")
        for (i <- 0 until dutConfig.kernelDepth) {
          dut.io.kernel_buffer_write_addr.poke(i.U)
          dut.io.kernel_buffer_write_data.poke(kernel_fixed_flat(i).S(dutConfig.dataWidth.W))
          dut.clock.step(1); cycles += 1
        }
        dut.io.kernel_buffer_write_en.poke(false.B)

        dut.io.ofm_buffer_read_addr.poke(0.U)
        dut.io.start.poke(false.B)
        dut.clock.step(1); cycles +=1

        println(s"TB Cycle $cycles: Starting ComputeUnit...")
        dut.io.start.poke(true.B)
        dut.clock.step(1); cycles += 1
        dut.io.start.poke(false.B)

        // Recalculate maxComputeCycles based on 'same' OFM dimensions
        val cyclesPerKernelElement = 5 // sFetchIFM(2) + sFetchKernel(2) + sMAC(1)
        val cyclesWriteOfmOverhead = 1 // sWriteOFM state
        val kernelOpsPerOfmPixel = dutConfig.kernelRows * dutConfig.kernelCols
        val estimatedCycles = expectedOutputRows * expectedOutputCols * (kernelOpsPerOfmPixel * cyclesPerKernelElement + cyclesWriteOfmOverhead)
        val maxComputeCycles = estimatedCycles + 500 // Increased safety margin

        var computeCycles = 0
        while (dut.io.done.peek().litValue != BigInt(1) && computeCycles < maxComputeCycles) {
          dut.clock.step(1); cycles += 1; computeCycles += 1
        }
        assert(computeCycles < maxComputeCycles, s"$testName: ComputeUnit simulation timed out after $computeCycles cycles (max allowed: $maxComputeCycles).")
        assert(dut.io.done.peek().litValue == BigInt(1), s"$testName: ComputeUnit did not signal done.")
        println(s"TB Cycle $cycles: ComputeUnit finished in $computeCycles compute cycles for ${expectedOutputRows}x${expectedOutputCols} OFM.")

        // Read OFM buffer. dutConfig.ofmDepth MUST be ifmRows * ifmCols.
        println(s"TB: Reading OFM buffer results (expecting ${dutConfig.ofmDepth} elements for 'same' output)...")
        val actual_ofm_map_dut_output = collection.mutable.Map[(Int, Int), BigInt]()
        assert(dutConfig.ofmDepth == expectedOutputRows * expectedOutputCols, "Mismatch between dutConfig.ofmDepth and expected 'same' output depth")

        for (i <- 0 until dutConfig.ofmDepth) {
          dut.io.ofm_buffer_read_addr.poke(i.U)
          dut.clock.step(1); cycles += 1 // Account for read latency of MinimalBuffer (RegNext)
          val data = dut.io.ofm_buffer_read_data.peek().litValue
          // Reconstruct 2D index based on 'same' OFM dimensions (which is ifmCols)
          val r = i / dutConfig.ifmCols // Use ifmCols for width of OFM
          val c = i % dutConfig.ifmCols // Use ifmCols for width of OFM
          actual_ofm_map_dut_output((r, c)) = data
        }

        println(s"Total simulation cycles: $cycles.")
        println(s"Verifying OFM results for $testName (comparing DUT's ${expectedOutputRows}x${expectedOutputCols} 'same' output against reference):")
        var mismatches = 0
        for (r <- 0 until expectedOutputRows) {
          for (c <- 0 until expectedOutputCols) {
            val actual_fixed = actual_ofm_map_dut_output.getOrElse((r, c), {
              println(s"CRITICAL ERROR: Missing DUT output data for OFM at ($r, $c)"); BigInt(0)
            })
            val actual_double = fixedToFloat(actual_fixed, dutConfig.F_BITS)
            val expected_double = expected_ofm_same_full_double(r)(c) // Direct comparison
            val tolerance = 6 * 1.5 / (1L << dutConfig.F_BITS)

            if (!(Math.abs(actual_double - expected_double) < tolerance)) {
              if (mismatches < 20) { // Print more mismatches if they occur
                println(f"OFM($r%d,$c%d): Actual=${actual_double}%9.4f (Fixed:$actual_fixed%d), Expected=${expected_double}%9.4f. MISMATCH! Diff: ${Math.abs(actual_double - expected_double)}%.6f, Tolerance: $tolerance%.6f")
              }
              mismatches +=1
            }
          }
        }
        if (mismatches > 0) println(s"WARNING: $mismatches OFM value mismatches found for $testName.")
        else println(s"SUCCESS: $testName OFM values match reference.")
        assert(mismatches == 0, s"$mismatches OFM value mismatches found for $testName.")
      }
    }
  }

  // --- Define Test Cases ---
  // Ensure your AcceleratorConfig is set up to produce 'same' OFM dimensions by default,
  // or pass a flag like 'isSameConvolution = true' to its constructor.

  // --- IFM 3x3 ---
  runConvolutionTest("3x3 IFM, 1x1 Kernel", 3,3, 1,1, 3,3, 1,1, fBits=4)
  runConvolutionTest("Original 3x3 IFM, 2x2 Kernel", 3,3, 2,2, 3,3, 2,2, fBits=4)
  runConvolutionTest("3x3 IFM, 3x3 Kernel", 3,3, 3,3, 3,3, 3,3, fBits=10)

  // --- IFM 5x5 ---
  runConvolutionTest("5x5 IFM, 1x1 Kernel", 5,5, 1,1, 5,5, 1,1, fBits=10)
  runConvolutionTest("5x5 IFM, 3x3 Kernel", 5,5, 3,3, 5,5, 3,3, fBits=10)
  runConvolutionTest("5x5 IFM, 5x5 Kernel", 5,5, 5,5, 5,5, 5,5, fBits=10)

  // --- IFM 10x10 ---
  runConvolutionTest("10x10 IFM, 1x1 Kernel", 5,5, 1,1, 10,10, 1,1, fBits=10)
  runConvolutionTest("10x10 IFM, 3x3 Kernel", 5,5, 3,3, 10,10, 3,3, fBits=10)
  runConvolutionTest("10x10 IFM, 5x5 Kernel", 5,5, 5,5, 10,10, 5,5, fBits=10)

  // Example for larger IFM like 32x32 (can be slow to simulate)
  runConvolutionTest("32x32 IFM, 3x3 Kernel", 4,4, 3,3, 32,32, 3,3, fBits=10)
  runConvolutionTest("32x32 IFM, 5x5 Kernel", 4,4, 5,5, 32,32, 5,5, fBits=10)
}