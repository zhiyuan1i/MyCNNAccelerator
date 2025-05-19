package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config}

class TestConfigForComputeUnit extends Config(new AcceleratorConfig)

class ComputeUnitTester extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "ComputeUnit"

  implicit val p: Parameters = new TestConfigForComputeUnit
  val accConfig = p(MyCNNAcceleratorKey)
    .copy( // Override for simpler testing if needed, e.g., smaller dimensions
      ifmRows = 4, ifmCols = 4,
      ofmRows = 4, ofmCols = 4,
      ifmDepth = 16, ofmDepth = 16,
      kernelMaxRows = 3, kernelMaxCols = 3, kernelMaxDepth = 9
    )

  // Mock scratchpad read behavior for IFM and Kernel
  def mockSpadReads(dut: ComputeUnit, ifmVal: BigInt, kernelVal: BigInt): Unit = {
    // IFM Read
    if (dut.io.ifm_read_req.en.peek().litToBoolean) {
      // In a real scenario, the address would be checked, and data supplied from a model
      dut.io.ifm_read_req.data.poke(ifmVal.U(accConfig.ifmDataType.dataWidth.W))
    }
    // Kernel Read
    if (dut.io.kernel_read_req.en.peek().litToBoolean) {
      dut.io.kernel_read_req.data.poke(kernelVal.U(accConfig.kernelDataType.dataWidth.W))
    }
  }

  it should "perform a basic convolution pass" in {
    test(new ComputeUnit(accConfig)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      c.io.start.poke(false.B)
      c.io.kernel_dim.poke(3.U) // e.g., 3x3 kernel

      // Initialize SPAD read data inputs (mocked)
      c.io.ifm_read_req.data.poke(0.U)
      c.io.kernel_read_req.data.poke(0.U)
      
      // Initialize OFM write (DUT drives this)
      // We'll observe these outputs

      c.clock.step(5) // Initial idle state

      // Start computation
      c.io.start.poke(true.B)
      c.clock.step(1)
      c.io.start.poke(false.B) // Start is a pulse

      c.io.busy.expect(true.B)

      var timeout = (accConfig.ofmRows * accConfig.ofmCols * accConfig.kernelMaxRows * accConfig.kernelMaxCols * 3) + 100 // Generous timeout
      var cycles = 0
      var lastOFMWriteAddr = -1L
      var lastOFMWriteData = -1L

      while (c.io.busy.peek().litToBoolean && timeout > 0) {
        // Mock SPAD reads: provide some dummy data
        // In a real test, this would come from a memory model based on addresses
        val ifmReadAddress = c.io.ifm_read_req.addr.peek().litValue
        val kernelReadAddress = c.io.kernel_read_req.addr.peek().litValue
        mockSpadReads(c, ifmVal = 1, kernelVal = 1) // Simplistic: always read '1'

        if (c.io.ifm_read_req.en.peek().litToBoolean) {
            //println(s"Cycle $cycles: IFM Read Addr: $ifmReadAddress")
        }
        if (c.io.kernel_read_req.en.peek().litToBoolean) {
            //println(s"Cycle $cycles: Kernel Read Addr: $kernelReadAddress")
        }

        if (c.io.ofm_write_req.en.peek().litToBoolean) {
          lastOFMWriteAddr = c.io.ofm_write_req.addr.peek().litValue.toLong
          lastOFMWriteData = c.io.ofm_write_req.data.peek().litValue.toLong
          //println(s"Cycle $cycles: OFM Write Addr: $lastOFMWriteAddr, Data: $lastOFMWriteData")
        }
        
        c.clock.step(1)
        timeout -= 1
        cycles +=1
      }

      assert(timeout > 0, "Timeout waiting for ComputeUnit to finish")
      c.io.done.expect(true.B)
      c.io.error.expect(false.B)
      
      // Basic check: ensure OFM write occurred for the last element
      // For a 4x4 OFM, last address is 15 (0-indexed)
      // This doesn't check data correctness, just that writes happened.
      assert(lastOFMWriteAddr == (accConfig.ofmRows * accConfig.ofmCols - 1), s"Last OFM write address was $lastOFMWriteAddr, expected ${accConfig.ofmRows * accConfig.ofmCols - 1}")

      println(s"ComputeUnit finished in $cycles cycles.")
    }
  }

  // Add more tests:
  // - Different kernel sizes
  // - Test padding logic by providing specific IFM values and checking accumulator
  // - Test fixed-point arithmetic (requires more detailed data setup and checking)
  // - Test error conditions (if any are implemented and signaled by ComputeUnit)
}