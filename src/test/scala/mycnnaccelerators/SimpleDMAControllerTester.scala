// In generators/mycnnaccelerators/src/test/scala/mycnnaccelerators/SimpleDMAControllerTester.scala
package mycnnaccelerators

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config}

import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.rocket.HellaCacheReq // Req might not be explicitly used, but good for context
import freechips.rocketchip.rocket.HellaCacheResp
import freechips.rocketchip.rocket.constants.MemoryOpConstants // Import the trait
import chisel3.experimental.BundleLiterals._
import chisel3.util.log2Ceil

import mycnnaccelerators.{MyCNNAcceleratorKey, AcceleratorConfig, DefaultAcceleratorConfig}

class TestDMAConfig extends Config((site, here, up) => {
  case MyCNNAcceleratorKey => AcceleratorConfig()
})

// SimpleHellaCacheMemModel now also mixes in MemoryOpConstants
class SimpleHellaCacheMemModel(dutIO: HellaCacheIO, clock: Clock, initialMemory: Map[Long, Long] = Map()) extends MemoryOpConstants { // <<< MIX IN TRAIT HERE
  val memory = collection.mutable.Map[Long, Long]() ++ initialMemory

  fork {
    while (true) {
      dutIO.req.ready.poke(true.B)
      if (dutIO.req.valid.peek().litToBoolean) {
        val addr = dutIO.req.bits.addr.peek().litValue.toLong
        val cmd = dutIO.req.bits.cmd.peek().litValue
        val dataToWrite = dutIO.req.bits.data.peek().litValue
        val tag = dutIO.req.bits.tag.peek().litValue
        // val size = 1 << dutIO.req.bits.size.peek().litValue.toInt // For reference

        clock.step(5) // Simulate memory latency

        if (cmd == M_XRD.litValue) { // Memory Read (M_XRD is now in scope)
          var readData: BigInt = 0 // Declared as BigInt
          readData = BigInt(memory.getOrElse(addr, 0L)) // <<< CORRECTED: Convert Long to BigInt
          println(s"[MemModel] Read from addr 0x${addr.toHexString}, data 0x${readData.toString(16)}")
          dutIO.resp.valid.poke(true.B)
          dutIO.resp.bits.data.poke(readData.U)
          dutIO.resp.bits.addr.poke(addr.U)
          dutIO.resp.bits.cmd.poke(M_XRD) // Echo command (M_XRD now in scope)
          dutIO.resp.bits.tag.poke(tag.U)
          dutIO.resp.bits.has_data.poke(true.B)
          dutIO.resp.bits.replay.poke(false.B)
          // dutIO.resp.bits.nack.poke(false.B) // <<< REMOVED: nack is not a field
          // Ensure other required fields of HellaCacheResp are poked if necessary by your DUT
          // For example, size, signed if your DUT uses them from the response.
          // Standard HellaCacheResp does not require them to be driven by a simple memory like this.
          clock.step(1)
          dutIO.resp.valid.poke(false.B)
        } else if (cmd == M_XWR.litValue) { // Memory Write (M_XWR is now in scope)
          println(s"[MemModel] Write to addr 0x${addr.toHexString}, data 0x${dataToWrite.toString(16)}")
          memory(addr) = dataToWrite.toLong
          dutIO.resp.valid.poke(true.B)
          // For write responses, typically only tag, and replay are strictly needed.
          // Other fields like addr, cmd, has_data might not be checked by DUT or can be defaulted.
          dutIO.resp.bits.cmd.poke(M_XWR) // Echo command (M_XWR now in scope)
          dutIO.resp.bits.addr.poke(addr.U) // Echo address
          dutIO.resp.bits.tag.poke(tag.U)
          dutIO.resp.bits.has_data.poke(false.B) // Write ack typically doesn't have data
          dutIO.resp.bits.replay.poke(false.B)
          // dutIO.resp.bits.nack.poke(false.B) // <<< REMOVED: nack is not a field
          clock.step(1)
          dutIO.resp.valid.poke(false.B)
        }
      }
      clock.step(1)
    }
  }
}

class SimpleDMAControllerTester extends AnyFlatSpec with ChiselScalatestTester with MemoryOpConstants { // Outer class still mixes it in
  behavior of "SimpleDMAController"

  implicit val p: Parameters = new TestDMAConfig
  val accConfig = p(MyCNNAcceleratorKey)

  it should "transfer data from main memory to IFM scratchpad" in {
    val initialMemContents = Map(
      0x1000L -> 0x11223344AABBCCDDL,
      0x1008L -> 0xEEFF001155667788L
    )

    test(new SimpleDMAController(accConfig)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      val memModel = new SimpleHellaCacheMemModel(c.io.mem, c.clock, initialMemContents)
      
      c.io.dma_req.initSource().setSourceClock(c.clock)
      c.io.spad_read_data.poke(0.U)

      val elementBytes = accConfig.ifmDataType.dataWidth / 8
      val numElementsToRead = accConfig.tileLinkBeatBytes / elementBytes
      val totalBytesToRead = numElementsToRead * elementBytes

      val coreMaxAddrBitsParam: Int = accConfig.coreMaxAddrBits
      // Ensure max depth is at least 1 for log2Ceil if depths can be 0.
      // log2Ceil(0) is undefined, log2Ceil(1) is 0.
      val minDepthForLog = 1 
      val spadAddrWidthMaxParam: Int = log2Ceil(
          Seq(accConfig.ifmDepth, accConfig.kernelMaxDepth, accConfig.ofmDepth).map(d => math.max(d, minDepthForLog)).max
      )


      c.io.dma_req.enqueueNow(
        (new DMARequest(
          coreMaxAddrBits = coreMaxAddrBitsParam,
          spadAddrWidthMax = spadAddrWidthMaxParam
        )).Lit(
          _.main_mem_addr -> 0x1000.U,
          _.spad_base_addr -> 0.U,
          _.length_bytes -> totalBytesToRead.U,
          _.is_write_to_main_mem -> false.B,
          _.spad_target_is_ifm -> true.B,
          _.spad_target_is_kernel -> false.B
        )
      )

      while (!c.io.busy.peek().litToBoolean) {
        c.clock.step(1)
      }
      println("DMA is busy")

      var timeout = 200
      while (c.io.busy.peek().litToBoolean && timeout > 0) {
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
    }
  }
}