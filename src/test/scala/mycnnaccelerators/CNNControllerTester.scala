package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config}
import freechips.rocketchip.tile.RoCCCommand
import CNNAcceleratorISA._ // Import your ISA definitions
import freechips.rocketchip.rocket.HellaCacheIO
import AcceleratorConfig._

// Assuming you have a AcceleratorConfig similar to the one used in ScratchpadTester
// If not, you might need to create or adjust this part.
class TestConfigForCNNController extends Config(new AcceleratorConfig)

class CNNControllerTester extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "CNNController"

  implicit val p: Parameters = new TestConfigForCNNController
  val accConfig = p(MyCNNAcceleratorKey) // Fetch AcceleratorConfig

  // Helper function to send a RoCC command
  def sendCommand(dut: CNNController, funct: UInt, rs1: BigInt, rs2: BigInt, rd: Int = 0): Unit = {
    dut.io.cmd_in.valid.poke(true.B)
    dut.io.cmd_in.bits.inst.funct.poke(funct)
    dut.io.cmd_in.bits.inst.rs1.poke(rs1.U)
    dut.io.cmd_in.bits.inst.rs2.poke(rs2.U)
    dut.io.cmd_in.bits.inst.rd.poke(rd.U)
    dut.io.cmd_in.bits.status.poke(0.U) // Assuming default status, adjust if needed
    dut.clock.step(1)
    dut.io.cmd_in.valid.poke(false.B)
  }

  // Helper to check for response
  def expectResponse(dut: CNNController, expectedData: BigInt, expectedRd: Int): Unit = {
    // Wait for response valid, with a timeout
    var timeout = 0
    while (!dut.io.cmd_resp_valid_out.peek().litToBoolean && timeout < 50) {
      dut.clock.step(1)
      timeout += 1
    }
    assert(timeout < 50, "Timeout waiting for response valid")
    dut.io.cmd_resp_data_out.expect(expectedData.U)
    dut.io.cmd_resp_rd_out.expect(expectedRd.U)
    // Response should ideally go low after one cycle if ready is asserted by CPU,
    // or controller should hold it until ready. For basic check, just observe.
    // In a real RoCC environment, io.cmd.ready would be managed.
    // Here, we assume the controller makes it valid and then it might go low.
    // To ensure we see it, we might need io.cmd_in.ready to be high.
    // For this test, we assume the controller drives resp_valid for at least one cycle.
  }


  it should "configure IFM, Kernel, OFM addresses and start convolution" in {
    test(new CNNController(accConfig)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      c.io.cmd_in.initSource().setSourceClock(c.clock)
      c.io.cmd_resp_valid_out.poke(false.B) // Model CPU not ready initially for response

      // Mock DMA and Compute inputs
      c.io.dma_busy_in.poke(false.B)
      c.io.dma_done_in.poke(false.B)
      c.io.dma_error_in.poke(false.B) 
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(false.B)
      c.io.compute_error_in.poke(false.B)
      c.io.dma_req_out.initSink().setSinkClock(c.clock) // Allow requests to be dequeued

      // 1. Configure IFM Address
      sendCommand(c, CONFIG_IFM_ADDR, 0x1000, 0x0, 1)
      c.clock.step(1) // Allow controller to process
      // Controller should make response valid. Poke ready high to consume response.
      c.io.cmd_in.ready.expect(false.B) // Controller should be busy processing or responding
      // Assuming response is immediate for config if cmd_in.ready was high on controller side
      // Here, we wait for cmd_resp_valid_out
      expectResponse(c, STATUS_IDLE.litValue, 1)
      c.clock.step(5) // Give some cycles

      // 2. Configure Kernel Address and Size
      val kernelDim = 3
      sendCommand(c, CONFIG_KERNEL_ADDR_SIZE, 0x2000, kernelDim, 2)
      c.clock.step(1)
      expectResponse(c, STATUS_IDLE.litValue, 2)
      c.clock.step(5)

      // 3. Configure OFM Address
      sendCommand(c, CONFIG_OFM_ADDR_PARAMS, 0x3000, 0x0, 3)
      c.clock.step(1)
      expectResponse(c, STATUS_IDLE.litValue, 3)
      c.clock.step(5)

      // 4. Check Status (should be IDLE)
      sendCommand(c, GET_STATUS, 0x0, 0x0, 4)
      c.clock.step(1)
      expectResponse(c, STATUS_IDLE.litValue, 4)
      c.clock.step(5)

      // 5. Start Convolution
      sendCommand(c, START_CONVOLUTION, 0x0, 0x0, 0) // rd is not used for START
      c.io.busy_out.expect(true.B)
      // Controller should move to sDmaLoadIFM_Setup then sDmaLoadIFM_Busy
      // Expect DMA request for IFM
      c.io.dma_req_out.expectDequeuePred( (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x1000 &&
        !req.is_write_to_main_mem.peek().litToBoolean &&
        req.spad_target_is_ifm.peek().litToBoolean
      })
      // Simulate DMA finishing IFM load
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Expect DMA request for Kernel
      c.io.dma_req_out.expectDequeuePred( (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x2000 &&
        !req.is_write_to_main_mem.peek().litToBoolean &&
        req.spad_target_is_kernel.peek().litToBoolean
      })
      // Simulate DMA finishing Kernel load
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Expect Compute to start
      c.io.compute_start_out.expect(true.B)
      c.io.compute_kernel_dim_out.expect(kernelDim.U)
      c.clock.step(1)
      c.io.compute_start_out.expect(false.B) // Should be a pulse

      // Simulate Compute finishing
      c.io.compute_busy_in.poke(true.B) // Keep compute busy for a few cycles
      c.clock.step(5)
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(true.B)
      c.clock.step(1)
      c.io.compute_done_in.poke(false.B)

      // Expect DMA request for OFM writeback
       c.io.dma_req_out.expectDequeuePred( (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x3000 &&
        req.is_write_to_main_mem.peek().litToBoolean
      })
      // Simulate DMA finishing OFM writeback
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Controller should be IDLE again
      c.clock.step(5) // Allow FSM to return to Idle
      c.io.busy_out.expect(false.B)

      // Check Status (should be DONE_SUCCESS before returning to IDLE, then IDLE)
      // The status register might reflect DONE_SUCCESS briefly before sRespondDone transitions to sIdle.
      // Let's try sending GET_STATUS again. It should be IDLE now.
      sendCommand(c, GET_STATUS, 0x0, 0x0, 5)
      c.clock.step(1)
      expectResponse(c, STATUS_IDLE.litValue, 5) // Or STATUS_DONE_SUCCESS then IDLE if checked earlier
    }
  }

  it should "report config error if START is sent before full configuration" in {
    test(new CNNController(accConfig)) { c =>
      c.io.cmd_in.initSource().setSourceClock(c.clock)
      c.io.dma_req_out.initSink().setSinkClock(c.clock)

      // Mock DMA and Compute inputs
      c.io.dma_busy_in.poke(false.B)
      c.io.dma_done_in.poke(false.B)
      c.io.dma_error_in.poke(false.B)
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(false.B)
      c.io.compute_error_in.poke(false.B)

      // Send START_CONVOLUTION without any prior config
      sendCommand(c, START_CONVOLUTION, 0x0, 0x0, 1)
      c.clock.step(1)
      expectResponse(c, STATUS_ERROR_CONFIG.litValue, 1)
      c.io.busy_out.expect(false.B) // Should not become busy, just error and stay idle
    }
  }

  // Add more tests for other ISA commands, error conditions (DMA error, Compute error), etc.
}