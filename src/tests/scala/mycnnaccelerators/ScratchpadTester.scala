package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config} // For Parameters and Config

// It's good practice to have a specific test configuration if your DUT or its parameters
// depend on the `implicit p: Parameters` pattern.
// This reuses your MyCNNConfig for consistency.
class TestConfigForMyCNN extends Config(new MyCNNConfig)

class ScratchpadTester extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "Scratchpad"

  // Create an implicit Parameter instance using your test configuration
  implicit val p: Parameters = new TestConfigForMyCNN 
  
  // Fetch the AcceleratorConfig that your Scratchpad module expects.
  // This uses the MyCNNAcceleratorKey you defined in MyCNNRoCC.scala
  val accConfig = p(MyCNNAcceleratorKey) 

  it should "write to and read from IFM memory" in {
    test(new Scratchpad(accConfig)) { c => // accConfig is passed here
      c.io.ifm_write.en.poke(true.B)
      c.io.ifm_write.addr.poke(10.U)
      c.io.ifm_write.data.poke(123.U((accConfig.ifmDataType.dataWidth).W))
      c.clock.step(1)
      c.io.ifm_write.en.poke(false.B)
      c.clock.step(1) // Give time for write to settle if any internal FSM/reg

      c.io.ifm_read.en.poke(true.B)
      c.io.ifm_read.addr.poke(10.U)
      // For SyncReadMem, data is available on the cycle *after* en & addr are asserted
      c.clock.step(1) 
      c.io.ifm_read.data.expect(123.U((accConfig.ifmDataType.dataWidth).W))
      c.io.ifm_read.en.poke(false.B)
      c.clock.step(5)
    }
  }

  it should "write to and read from Kernel memory" in {
    test(new Scratchpad(accConfig)) { c =>
      c.io.kernel_write.en.poke(true.B)
      c.io.kernel_write.addr.poke(5.U)
      c.io.kernel_write.data.poke(456.U((accConfig.kernelDataType.dataWidth).W))
      c.clock.step(1)
      c.io.kernel_write.en.poke(false.B)
      c.clock.step(1)

      c.io.kernel_read.en.poke(true.B)
      c.io.kernel_read.addr.poke(5.U)
      c.clock.step(1)
      c.io.kernel_read.data.expect(456.U((accConfig.kernelDataType.dataWidth).W))
      c.io.kernel_read.en.poke(false.B)
      c.clock.step(5)
    }
  }
  
  // Add a similar test for OFM memory
  it should "write to and read from OFM memory" in {
    test(new Scratchpad(accConfig)) { c =>
      c.io.ofm_write.en.poke(true.B)
      c.io.ofm_write.addr.poke(20.U)
      c.io.ofm_write.data.poke(789.U((accConfig.ofmDataType.dataWidth).W))
      c.clock.step(1)
      c.io.ofm_write.en.poke(false.B)
      c.clock.step(1)

      c.io.ofm_read.en.poke(true.B)
      c.io.ofm_read.addr.poke(20.U)
      c.clock.step(1)
      c.io.ofm_read.data.expect(789.U((accConfig.ofmDataType.dataWidth).W))
      c.io.ofm_read.en.poke(false.B)
      c.clock.step(5)
    }
  }
}