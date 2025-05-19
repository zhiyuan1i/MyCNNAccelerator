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
  val addrWidth = log2Ceil(depth) // IO 仍然需要地址宽度
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  // 使用寄存器向量 (Reg of Vec) 来实现存储

  val mem_reg = Reg(Vec(depth, SInt(dataWidth.W)))

  // 写操作
  when(io.write_en) {
    mem_reg(io.write_addr) := io.write_data
  }

  // 读操作
  // 为了保持与 SyncReadMem 或 Mem + RegNext 相似的 1 周期读延迟：
  // mem_reg(io.read_addr) 是对寄存器向量的组合逻辑读
  // RegNext 会将这个组合逻辑读的结果在下一个周期输出
  io.read_data := RegNext(mem_reg(io.read_addr))

}