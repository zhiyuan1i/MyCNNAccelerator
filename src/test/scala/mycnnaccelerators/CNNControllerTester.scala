package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config}
// Ensure RoCCCommand and MStatus are imported
import freechips.rocketchip.tile.RoCCCommand
import freechips.rocketchip.rocket.MStatus 
import CNNAcceleratorISA._
import chisel3.util.DecoupledIO 
// import freechips.rocketchip.rocket.HellaCacheIO // Likely not needed here, but harmless
// import AcceleratorConfig._ // This might be covered by the more specific import below
// Ensure DMARequest is in scope, e.g. by importing it if it's in the mycnnaccelerators package
// import mycnnaccelerators.DMARequest // Assuming DMARequest is defined in this package

// Your config class (looks good)
class TestConfigForCNNController extends Config((site, here, up) => {
    case MyCNNAcceleratorKey => DefaultAcceleratorConfig
})

class CNNControllerTester extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "CNNController"

  implicit val p: Parameters = new TestConfigForCNNController
  val accConfig = p(MyCNNAcceleratorKey)

  // Helper function to send a RoCC command (looks good)
  def sendCommand(dut: CNNController, funct: UInt, rs1: BigInt, rs2: BigInt, rd: Int = 0): Unit = {
    dut.io.cmd_in.valid.poke(true.B)
    dut.io.cmd_in.bits.inst.funct.poke(funct)
    dut.io.cmd_in.bits.inst.rs1.poke(rs1.U)
    dut.io.cmd_in.bits.inst.rs2.poke(rs2.U)
    dut.io.cmd_in.bits.inst.rd.poke(rd.U)
    dut.io.cmd_in.bits.status.poke(0.U.asTypeOf(new MStatus)) // Correct MStatus poke
    dut.clock.step(1)
    dut.io.cmd_in.valid.poke(false.B)
  }

  // Helper to check for response (looks good)
  def expectResponse(dut: CNNController, expectedData: BigInt, expectedRd: Int): Unit = {
    var timeout = 0
    // Wait for the DUT to assert cmd_resp_valid_out
    while (!dut.io.cmd_resp_valid_out.peek().litToBoolean && timeout < 50) {
      dut.clock.step(1)
      timeout += 1
    }
    assert(timeout < 50, "Timeout waiting for response valid")
    dut.io.cmd_resp_data_out.expect(expectedData.U)
    dut.io.cmd_resp_rd_out.expect(expectedRd.U)
    // In a real system, the CPU would signal it has taken the response, often via io.resp.ready.
    // For this test, we assume the DUT keeps resp_valid high for one cycle or the test consumes it implicitly.
  }

  // Helper to expect and dequeue a DMARequest with predicate checks
  // T_REQ is the type of the data on the DecoupledIO, assumed to have main_mem_addr, is_write_to_main_mem etc.
  def expectDMARequestPred(
      dmaChannel: DecoupledIO[DMARequest], // Pass the channel directly
      clock: Clock, // Pass the clock for stepping
      pred: DMARequest => Boolean,
      failMsg: String = "DMA Request predicate failed"
  ): DMARequest = {
      dmaChannel.ready.poke(true.B)
      var timeout = 0
      while (!dmaChannel.valid.peek().litToBoolean && timeout < 100) {
          clock.step(1)
          timeout += 1
      }
      assert(timeout < 100, s"Timeout waiting for DMA request valid. $failMsg")
      dmaChannel.valid.expect(true.B)
      val req = dmaChannel.bits.peek()
      assert(pred(req), s"$failMsg for request: $req")
      clock.step(1) // Consume the request
      dmaChannel.ready.poke(false.B) // Good practice to deassert ready
      req
  }


  it should "configure IFM, Kernel, OFM addresses and start convolution" in {
    test(new CNNController(accConfig)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      c.io.cmd_in.initSource().setSourceClock(c.clock)
      // REMOVED: c.io.cmd_resp_valid_out.poke(false.B) // Cannot poke an output of the DUT
      // If you need to model CPU not ready for response, DUT needs a resp_ready input.

      // Mock DMA and Compute inputs
      c.io.dma_busy_in.poke(false.B)
      c.io.dma_done_in.poke(false.B)
      c.io.dma_error_in.poke(false.B)
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(false.B)
      c.io.compute_error_in.poke(false.B)
      c.io.dma_req_out.initSink().setSinkClock(c.clock)
      c.io.dma_req_out.ready.poke(false.B) // Initially not ready to dequeue


      // 1. Configure IFM Address
      sendCommand(c, CONFIG_IFM_ADDR, 0x1000, 0x0, 1)
      c.clock.step(1)
      // c.io.cmd_in.ready.expect(false.B) // Controller might be busy, this is a valid check
      expectResponse(c, STATUS_IDLE.litValue, 1)
      c.clock.step(5)

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
      sendCommand(c, START_CONVOLUTION, 0x0, 0x0, 0)
      c.io.busy_out.expect(true.B)

      // Expect DMA request for IFM
      expectDMARequestPred(c.io.dma_req_out, c.clock, (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x1000 &&
        !req.is_write_to_main_mem.peek().litToBoolean &&
        req.spad_target_is_ifm.peek().litToBoolean
      }, "IFM DMA Request Mismatch")

      // Simulate DMA finishing IFM load
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Expect DMA request for Kernel
      expectDMARequestPred(c.io.dma_req_out, c.clock, (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x2000 &&
        !req.is_write_to_main_mem.peek().litToBoolean &&
        req.spad_target_is_kernel.peek().litToBoolean
      }, "Kernel DMA Request Mismatch")

      // Simulate DMA finishing Kernel load
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Expect Compute to start
      c.io.compute_start_out.expect(true.B)
      c.io.compute_kernel_dim_out.expect(kernelDim.U)
      c.clock.step(1)
      c.io.compute_start_out.expect(false.B)

      // Simulate Compute finishing
      c.io.compute_busy_in.poke(true.B)
      c.clock.step(5)
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(true.B)
      c.clock.step(1)
      c.io.compute_done_in.poke(false.B)

      // Expect DMA request for OFM writeback
      expectDMARequestPred(c.io.dma_req_out, c.clock, (req: DMARequest) => {
        req.main_mem_addr.peek().litValue == 0x3000 &&
        req.is_write_to_main_mem.peek().litToBoolean
        // Note: spad_target flags might not be relevant or could be checked if they are
      }, "OFM DMA Request Mismatch")

      // Simulate DMA finishing OFM writeback
      c.io.dma_done_in.poke(true.B)
      c.clock.step(1)
      c.io.dma_done_in.poke(false.B)

      // Controller should be IDLE again
      c.clock.step(5) // Allow FSM to return to Idle
      c.io.busy_out.expect(false.B)

      sendCommand(c, GET_STATUS, 0x0, 0x0, 5)
      c.clock.step(1)
      expectResponse(c, STATUS_IDLE.litValue, 5)
    }
  }

  it should "report config error if START is sent before full configuration" in {
    test(new CNNController(accConfig)) { c =>
      c.io.cmd_in.initSource().setSourceClock(c.clock)
      c.io.dma_req_out.initSink().setSinkClock(c.clock)
      c.io.dma_req_out.ready.poke(false.B) // Initially not expecting DMA requests

      // Mock DMA and Compute inputs
      c.io.dma_busy_in.poke(false.B)
      c.io.dma_done_in.poke(false.B)
      c.io.dma_error_in.poke(false.B)
      c.io.compute_busy_in.poke(false.B)
      c.io.compute_done_in.poke(false.B)
      c.io.compute_error_in.poke(false.B)

      sendCommand(c, START_CONVOLUTION, 0x0, 0x0, 1)
      c.clock.step(1)
      expectResponse(c, STATUS_ERROR_CONFIG.litValue, 1)
      c.io.busy_out.expect(false.B)
    }
  }
}