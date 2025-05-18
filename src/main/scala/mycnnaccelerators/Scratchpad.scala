// filename: Scratchpad.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

// IO Bundle for Scratchpad Read Port
class ScratchpadReadIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  val en = Input(Bool())      // Read enable
  val addr = Input(UInt(addrWidth.W)) // Read address
  val data = Output(UInt(dataWidth.W)) // Data read from Scratchpad
}

// IO Bundle for Scratchpad Write Port
class ScratchpadWriteIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  val en = Input(Bool())      // Write enable
  val addr = Input(UInt(addrWidth.W)) // Write address
  val data = Input(UInt(dataWidth.W)) // Data to write into Scratchpad
}

class Scratchpad(val config: AcceleratorConfig) extends Module {
  val io = IO(new Bundle {
    // IFM (Input Feature Map) Memory Ports
    val ifm_write = new ScratchpadWriteIO(log2Ceil(config.ifmDepth), config.ifmDataType.dataWidth)
    val ifm_read  = new ScratchpadReadIO(log2Ceil(config.ifmDepth), config.ifmDataType.dataWidth)

    // Kernel Memory Ports
    val kernel_write = new ScratchpadWriteIO(log2Ceil(config.kernelMaxDepth), config.kernelDataType.dataWidth)
    val kernel_read  = new ScratchpadReadIO(log2Ceil(config.kernelMaxDepth), config.kernelDataType.dataWidth)

    // OFM (Output Feature Map) Memory Ports
    val ofm_write = new ScratchpadWriteIO(log2Ceil(config.ofmDepth), config.ofmDataType.dataWidth)
    val ofm_read  = new ScratchpadReadIO(log2Ceil(config.ofmDepth), config.ofmDataType.dataWidth)
  })

  // Instantiate Synchronous Read Memories for IFM, Kernel, and OFM
  // SyncReadMem: read data is available one cycle after 'en' and 'addr' are asserted
  val ifm_mem    = SyncReadMem(config.ifmDepth, UInt(config.ifmDataType.dataWidth.W))
  val kernel_mem = SyncReadMem(config.kernelMaxDepth, UInt(config.kernelDataType.dataWidth.W))
  val ofm_mem    = SyncReadMem(config.ofmDepth, UInt(config.ofmDataType.dataWidth.W))

  // IFM Memory Logic
  when(io.ifm_write.en) {
    ifm_mem.write(io.ifm_write.addr, io.ifm_write.data)
  }
  // For SyncReadMem, read is registered. Data output one cycle after en & addr are valid.
  io.ifm_read.data := ifm_mem.read(io.ifm_read.addr, io.ifm_read.en)

  // Kernel Memory Logic
  when(io.kernel_write.en) {
    kernel_mem.write(io.kernel_write.addr, io.kernel_write.data)
  }
  io.kernel_read.data := kernel_mem.read(io.kernel_read.addr, io.kernel_read.en)

  // OFM Memory Logic
  when(io.ofm_write.en) {
    ofm_mem.write(io.ofm_write.addr, io.ofm_write.data)
  }
  io.ofm_read.data := ofm_mem.read(io.ofm_read.addr, io.ofm_read.en)
}