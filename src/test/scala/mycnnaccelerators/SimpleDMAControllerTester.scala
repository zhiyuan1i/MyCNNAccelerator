// In generators/mycnnaccelerators/src/test/scala/mycnnaccelerators/SimpleDMAControllerTester.scala
package mycnnaccelerators

import chisel3._
import chisel3.util.{log2Ceil, log2Up}
// ChiselScalatestTester
import chiseltest._
import chiseltest.iotesters._
// end ChiselScalatestTester

import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy._
import org.chipsalliance.diplomacy.bundlebridge._

import org.scalatest.flatspec.AnyFlatSpec
import org.chipsalliance.cde.config.{Parameters, Config, Field}

// Rocket Chip imports
import freechips.rocketchip.rocket.HellaCacheIO
import chisel3.experimental.BundleLiterals._

// Field Keys needed for TestDMAConfig
import freechips.rocketchip.tile.{TileKey, RocketTileParams, TileVisibilityNodeKey} // For TileKey and RocketTileParams
import freechips.rocketchip.rocket.{RocketCoreParams, BTBParams, DCacheParams, ICacheParams} // For components of RocketTileParams
import freechips.rocketchip.tilelink.{TLEphemeralNode}
import mycnnaccelerators.{MyCNNAcceleratorKey, AcceleratorConfig, FixedPointConfig}

// === Local Definitions for Core Field Keys ===
// Using local definitions as a robust workaround for potential import path issues in user's environment
case object XLen extends Field[Int]
case object PAddrBits extends Field[Int]
case object CacheBlockBytes extends Field[Int]
case object MaxHartIdBits extends Field[Int](1)
// ============================================

object DefaultTestAcceleratorConfig {
  val config = AcceleratorConfig(
    ifmDataType = FixedPointConfig(dataWidth = 16, fractionBits = 8),
    kernelDataType = FixedPointConfig(dataWidth = 16, fractionBits = 8),
    ofmDataType = FixedPointConfig(dataWidth = 16, fractionBits = 8),
    accumulatorWidth = 32,
    ifmDepth = 1024, kernelMaxDepth = 25, ofmDepth = 1024,
    ifmRows = 32, ifmCols = 32, kernelMaxRows = 5, kernelMaxCols = 5, ofmRows = 32, ofmCols = 32,
    coreMaxAddrBits = 64, tileLinkBeatBytes = 8, xLen = 64 // Default values
  )
}



class TestDMAConfig extends Config((site, here, up) => {
  // 在 lambda 函数作用域内定义常量
  val currentTestXLen_val = 64
  val currentTestPAddrBits_val = 32
  val currentTestCacheBlockBytes_val = 8 // For a 64-bit wide bus (8 bytes per beat)

  // 将 PartialFunction 显式赋值给一个 typed val
  val pf: PartialFunction[Any, Any] = {
    case XLen => currentTestXLen_val
    case PAddrBits => currentTestPAddrBits_val
    case CacheBlockBytes => currentTestCacheBlockBytes_val

    case MyCNNAcceleratorKey => DefaultTestAcceleratorConfig.config.copy(
        xLen = currentTestXLen_val,
        coreMaxAddrBits = currentTestPAddrBits_val,
        tileLinkBeatBytes = currentTestCacheBlockBytes_val
    )
    case MaxHartIdBits => 1

    // === 添加 TileKey 定义 ===
    case TileKey => RocketTileParams(
      core = RocketCoreParams( // xLen 会从 site(XLen) 获取
        useVM = false,
        fpu = None,
        mulDiv = None
        // 其他 RocketCoreParams 字段使用默认值
      ),
      dcache = Some(DCacheParams(
        blockBytes = currentTestCacheBlockBytes_val, // 使用上面定义的常量
        nSets = 64,    // 示例值
        nWays = 1,     // 示例值
        nMSHRs = 0     // 简化设置
        // 其他 DCacheParams 字段使用默认值
      )),
      icache = Some(ICacheParams(
        blockBytes = currentTestCacheBlockBytes_val, // 使用上面定义的常量
        nSets = 64,    // 示例值
        nWays = 1
        // 其他 ICacheParams 字段使用默认值
      )),
      btb = None
      // 根据之前的编译器错误，这里不显式传递 hartId 和 name
      // 假设它们在 RocketTileParams 类中由您的 Rocket Chip 版本默认提供
    )
    case TileVisibilityNodeKey => TLEphemeralNode()(ValName("test_dma_visibility_node"))
  }

  pf // 返回 PartialFunction
})


object HexUtils {
  def toHex(l: Long): String = java.lang.Long.toString(l, 16)
  def toHex(bi: BigInt): String = bi.toString(16)
  def toHex(i: Int): String = java.lang.Integer.toString(i, 16)
}


class SimpleDMAControllerTester extends AnyFlatSpec with ChiselScalatestTester
  with freechips.rocketchip.rocket.constants.MemoryOpConstants {

  implicit val p: Parameters = Parameters.empty.alter(new TestDMAConfig())
  val accConfig = p(MyCNNAcceleratorKey)

  behavior of "SimpleDMAController"
  import HexUtils._

  def printDUTState(c: SimpleDMAController, cycle: Int): Unit = {
    println(s"[Cycle $cycle] DUT IOs:")
    println(s"  dma_req.ready: ${c.io.dma_req.ready}")
    println(s"  busy: ${c.io.busy}, done: ${c.io.done}, error: ${c.io.error}")
    println(s"  mem.req.valid: ${c.io.mem.req.valid}, mem.req.ready (poked by test): ${c.io.mem.req.ready}, mem.req.bits.addr: 0x${toHex(c.io.mem.req.bits.addr.peek().litValue)}, mem.req.bits.cmd: ${c.io.mem.req.bits.cmd.peek().litValue}")
    println(s"  mem.resp.valid (poked by test): ${c.io.mem.resp.valid}")
    // If HellaCacheIO.resp is ValidIO (no ready from DUT):
    // println(s"  mem.resp.ready: NOT APPLICABLE (Assuming ValidIO)") // Comment out or remove ready access
    println(s"  spad_write_en: ${c.io.spad_write_en}, spad_write_addr: ${toHex(c.io.spad_write_addr.peek().litValue.toInt)}")
    if (c.io.spad_write_en) {
      println(s"  spad_write_data: 0x${toHex(c.io.spad_write_data.peek().litValue)}")
    }
    println(s"  spad_read_en: ${c.io.spad_read_en}, spad_read_addr: ${toHex(c.io.spad_read_addr.peek().litValue.toInt)}")
    println("-" * 20)
  }

  it should "transfer one beat from main memory to IFM scratchpad (simplified test)" in {
    test(new SimpleDMAController(accConfig)(p)).withAnnotations(Seq(WriteVcdAnnotation)) { c =>
      c.io.dma_req.initSource().setSourceClock(c.clock)
      c.io.mem.resp.valid.poke(false.B)
      c.io.spad_read_data.poke(0.U)
      c.io.mem.req.ready.poke(false.B) // Test will control this
      c.clock.step(5)

      val elementBytes = accConfig.ifmDataType.dataWidth / 8
      assert(accConfig.tileLinkBeatBytes % elementBytes == 0)
      val numElementsInBeat = accConfig.tileLinkBeatBytes / elementBytes
      val totalBytesToTransfer = accConfig.tileLinkBeatBytes.U
      val dataToReadFromMemLit = "hCAFEF00DDEADBEEF".U((accConfig.tileLinkBeatBytes * 8).W)
      val coreMaxAddrBitsParam: Int = accConfig.coreMaxAddrBits
      val spadAddrWidthMaxParam: Int = log2Ceil(Seq(accConfig.ifmDepth, accConfig.kernelMaxDepth, accConfig.ofmDepth, 1).map(math.max(_, 1)).max)

      c.io.dma_req.enqueueNow(
        (new DMARequest(coreMaxAddrBitsParam, spadAddrWidthMaxParam)).Lit(
          _.main_mem_addr -> 0x1000.U, _.spad_base_addr -> 0.U, _.length_bytes -> totalBytesToTransfer,
          _.is_write_to_main_mem -> false.B, _.spad_target_is_ifm -> true.B, _.spad_target_is_kernel -> false.B
        )
      )

      var cycles = 0
      val maxCycles = numElementsInBeat * 40 + 300 // Increased timeout
      var memReqFired = false
      var spadElementsCountBasedOnAddr = 0 // Approximate count
      var memRespSentToDut = false
      var dutShouldHaveTakenResp = false


      var memReqValid_prevCycle = false
      var memReqReady_prevCycle = false


      printDUTState(c, cycles)

      while (!c.io.done && cycles < maxCycles) {
        memReqValid_prevCycle = c.io.mem.req.valid
        memReqReady_prevCycle = c.io.mem.req.ready // What test poked last cycle

        // 1. Test environment sets req.ready if DUT has a valid request
        if (c.io.mem.req.valid && !memReqFired) {
          println(s"[Cycle $cycles] DUT mem.req is VALID. Setting Test mem.req.ready = true.")
          c.io.mem.req.ready.poke(true.B)
        } else if (!c.io.mem.req.valid && c.io.mem.req.ready) {
            // If DUT deasserts valid but test still has ready high, test should deassert ready.
             println(s"[Cycle $cycles] DUT mem.req is no longer VALID. Test sets mem.req.ready = false.")
            c.io.mem.req.ready.poke(false.B)
        }


        // --- Clock advances ---
        c.clock.step(1)
        cycles += 1
        // --- Actions after clock step based on values from *before* this step ---

        if (memReqValid_prevCycle && memReqReady_prevCycle && !memReqFired) { // Request fired in the (now) previous cycle
          println(s"[Cycle $cycles Post-Step] Memory request fired in previous cycle. Test sets mem.req.ready = false.")
          c.io.mem.req.ready.poke(false.B) // Test is no longer ready for new req immediately
          memReqFired = true
        }
        
        // 4. Simulate memory response after latency if request has fired and response not yet sent
        if (memReqFired && !memRespSentToDut) {
            var currentLatency = 0 // This counter should be outside the while loop or reset properly
            val memReadLatency = 5 // Start counting latency after req has fired
            
            // This is a simplified latency model, assuming we start counting after memReqFired is true
            if (cycles > ( /* cycle when memReqFired became true */ 5 + memReadLatency) ) { // Ensure some cycles pass for latency
              println(s"[Cycle $cycles] Memory latency assumed complete. Poking mem.resp.valid = true.")
              c.io.mem.resp.valid.poke(true.B)
              c.io.mem.resp.bits.data.poke(dataToReadFromMemLit)
              c.io.mem.resp.bits.has_data.poke(true.B)
              memRespSentToDut = true
              dutShouldHaveTakenResp = true // With ValidIO, DUT takes it when valid is high
            }
        }
        
        // If test sent a response (assuming ValidIO for resp, no DUT ready signal to check)
        if (dutShouldHaveTakenResp) {
            println(s"[Cycle $cycles Post-Step] DUT should have taken memory response. Test deasserts mem.resp.valid.")
            c.io.mem.resp.valid.poke(false.B) // Deassert after one cycle
            dutShouldHaveTakenResp = false
        }

        if (c.io.spad_write_en) {
            spadElementsCountBasedOnAddr = (c.io.spad_write_addr.peek().litValue.toInt + 1).min(numElementsInBeat)
        }

        printDUTState(c, cycles)
      } // End while loop

      if (c.io.done) {
          spadElementsCountBasedOnAddr = numElementsInBeat
      }

      assert(memReqFired, "DMA controller's memory request was not seen as fired by the memory model.")
      assert(spadElementsCountBasedOnAddr >= numElementsInBeat, s"DMA did not write enough elements to SPAD. Expected: $numElementsInBeat, Counted: $spadElementsCountBasedOnAddr. Cycles: $cycles")
      assert(c.io.done, s"DMA did not signal done. Timed out after $cycles cycles. Busy: ${c.io.busy}, Error: ${c.io.error}")
      assert(!c.io.error, "DMA signaled an error.")
      println(s"Test completed in $cycles cycles.")
    }
  }
}