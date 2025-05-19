// filename: MinimalBuffer.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class MinimalBufferIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  // Write port
  val write_en   = Input(Bool())
  val write_addr = Input(UInt(addrWidth.W))
  val write_data = Input(SInt(dataWidth.W))

  // Read port
  val read_addr  = Input(UInt(addrWidth.W))
  val read_data  = Output(SInt(dataWidth.W))
}

class MinimalBuffer(depth: Int, dataWidth: Int) extends Module {
  val addrWidth = log2Ceil(depth) // IO still needs address width
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  // Use a register vector (Reg of Vec) to implement storage
  val mem_reg = Reg(Vec(depth, SInt(dataWidth.W)))

  // Write operation
  when(io.write_en) {
    mem_reg(io.write_addr) := io.write_data
  }

  // Read operation
  // To maintain a 1-cycle read latency similar to SyncReadMem or Mem + RegNext:
  // mem_reg(io.read_addr) is a combinational read from the register vector
  // RegNext will output the result of this combinational read in the next cycle
  io.read_data := RegNext(mem_reg(io.read_addr))
}