// In generators/mycnnaccelerators/src/test/scala/mycnnaccelerators/SimpleDMAControllerTester.scala
package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.HellaCacheReq
import freechips.rocketchip.rocket.HellaCacheResp
import freechips.rocketchip.rocket.constants.MemoryOpConstants._ // For M_XRD, M_XWR etc.
import org.chipsalliance.cde.config.Config
import org.chipsalliance.cde.config.Parameters

class TestDMAConfig extends Config(new MyCNNConfig)

// A very simple behavioral memory model for HellaCacheIO
class SimpleHellaCacheMemModel(dutIO: HellaCacheIO, clock: Clock, initialMemory: Map[Long, Long] = Map()) {
  val memory = collection.mutable.Map[Long, Long]() ++ initialMemory
  val xLen = dutIO.req.bits.data.getWidth // Typically coreDataBits

  // Continuously monitor requests
  fork {
    while (true) {
      if (dutIO.req.valid.peek().litToBoolean) {
        val addr = dutIO.req.bits.addr.peek().litValue.toLong
        val cmd = dutIO.req.bits.cmd.peek().litValue
        val size = 1 << dutIO.req.bits.size.peek().litValue.toInt // Bytes in transfer
        val dataToWrite = dutIO.req.bits.data.peek().litValue

        // Respond after a delay
        clock.step(5) // Simulate memory latency

        if (cmd == M_XRD) { // Memory Read
          var readData: BigInt = 0
          // Simple model: assumes aligned beat read for now
          // For xLen = 64 bits (8 bytes), and size typically config.tileLinkBeatBytes
          // This model reads one xLen-width word.
          // A real model would handle beats based on 'size' and cache line.
          readData = memory.getOrElse(addr, 0L) // Default to 0 if address not found
          println(s"[MemModel] Read from addr 0x${addr.toHexString}, data 0x${readData.toString(16)}")
          dutIO.resp.valid.poke(true.B)
          dutIO.resp.bits.data.poke(readData.U)
          dutIO.resp.bits.addr.poke(addr.U) // Echo address
          dutIO.resp.bits.cmd.poke(M_XRD.U) // Echo command
          dutIO.resp.bits.has_data.poke(true.B)
          dutIO.resp.bits.replay.poke(false.B)
          // Wait for DUT to accept response
          while (!dutIO.req.ready.peek().litToBoolean && dutIO.resp.valid.peek().litToBoolean) {
            clock.step(1)
          }
          clock.step(1)
          dutIO.resp.valid.poke(false.B)
        } else if (cmd == M_XWR) { // Memory Write
          println(s"[MemModel] Write to addr 0x${addr.toHexString}, data 0x${dataToWrite.toString(16)}, size ${size} bytes")
          // Simple model: assumes aligned beat write.
          memory(addr) = dataToWrite.toLong // Store the xLen-width data
          dutIO.resp.valid.poke(true.B) // Acknowledge write
          dutIO.resp.bits.cmd.poke(M_XWR.U)
          dutIO.resp.bits.addr.poke(addr.U)
          dutIO.resp.bits.has_data.poke(false.B)
          dutIO.resp.bits.replay.poke(false.B)
          // Wait for DUT to accept response
          while (!dutIO.req.ready.peek().litToBoolean && dutIO.resp.valid.peek().litToBoolean) {
            clock.step(1)
          }
          clock.step(1)
          dutIO.resp.valid.poke(false.B)
        }
      }
      clock.step(1)
    }
  }
}


class SimpleDMAControllerTester extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "SimpleDMAController"

  implicit val p: Parameters = new TestDMAConfig
  val accConfig = p(MyCNNAcceleratorKey)

  it should "transfer data from main memory to IFM scratchpad" in {
    val initialMemContents = Map(
      0x1000L -> 0x11223344AABBCCDDL,
      0x1008L -> 0xEEFF001155667788L
    ) // Assuming xLen=64, tileLinkBeatBytes=8

    test(new SimpleDMAController(accConfig)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      // Initialize memory model
      val memModel = new SimpleHellaCacheMemModel(c.io.mem, c.clock, initialMemContents)
      
      // Initialize DMA inputs
      c.io.dma_req.initSource().setSourceClock(c.clock)
      c.io.mem.req.initSink().setSinkClock(c.clock)       // DUT makes requests
      c.io.mem.resp.initSource().setSourceClock(c.clock) // Model sends responses

      // Drive Scratchpad inputs (or connect to a real Scratchpad module)
      c.io.spad_read_data.poke(0.U) // For DMA writes to memory

      // Send a DMA request: Read 2 elements (e.g. 2 * 16 bits = 4 bytes, if element is 16-bit)
      // For beat-based DMA: read 1 beat (8 bytes) if tileLinkBeatBytes = 8
      val elementBytes = accConfig.ifmDataType.dataWidth / 8
      val numElementsToRead = accConfig.tileLinkBeatBytes / elementBytes // Read one full beat
      val totalBytesToRead = numElementsToRead * elementBytes

      c.io.dma_req.enqueueNow(chiselTypeOf(c.io.dma_req.bits).Lit(
        _.main_mem_addr -> 0x1000.U,
        _.spad_base_addr -> 0.U, // Write to IFM starting at SPAD addr 0
        _.length_bytes -> totalBytesToRead.U,
        _.is_write_to_main_mem -> false.B,
        _.spad_target_is_ifm -> true.B,
        _.spad_target_is_kernel -> false.B
      ))

      // Wait for DMA to become busy
      while (!c.io.busy.peek().litToBoolean) {
        c.clock.step(1)
      }
      println("DMA is busy")

      // Wait for DMA to complete
      var timeout = 200
      while (c.io.busy.peek().litToBoolean && timeout > 0) {
        // Observe spad_write signals (optional, for debugging)
        if(c.io.spad_write_en.peek().litToBoolean) {
          println(s"SPAD Write: Addr=${c.io.spad_write_addr.peek().litValue}, Data=${c.io.spad_write_data.peek().litValue.toString(16)}")
        }
        c.clock.step(1)
        timeout -=1
      }
      assert(timeout > 0, "DMA timeout")

      c.io.done.expect(true.B)
      c.io.error.expect(false.B)
      println("DMA done")

      // Here, you would ideally check the contents of a Scratchpad model
      // For this example, we mainly checked DMA FSM progression and HellaCacheIO interaction.
    }
  }
}