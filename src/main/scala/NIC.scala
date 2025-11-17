package icenet

import chisel3._
import chisel3.util._
import chisel3.experimental.{IO, DataMirror}
import freechips.rocketchip.subsystem.{BaseSubsystem, TLBusWrapperLocation, PBUS, FBUS}
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import IceNetConsts._
import freechips.rocketchip.tilelink.TLRAM
import freechips.rocketchip.tilelink.TLXbar

// Custom imports for collective
import icenet.collective._

// This is copied from testchipip to avoid dependencies
class ClockedIO[T <: Data](private val gen: T) extends Bundle {
  val clock = Output(Clock())
  val bits = DataMirror.internal.chiselTypeClone[T](gen)
}

/**
 * @inBufFlits How many flits in the input buffer(s)
 * @outBufFlits Number of flits in the output buffer
 * @nMemXacts Maximum number of transactions that the send/receive path can send to memory
 * @maxAcquireBytes Cache block size
 * @ctrlQueueDepth Depth of the MMIO control queues
 * @usePauser Hardware support for Ethernet pause frames
 * @checksumOffload TCP checksum offload engine
 * @packetMaxBytes Maximum number of bytes in a packet (header size + MTU)
 */
case class NICConfig(
  inBufFlits: Int  = 2 * ETH_STANDARD_MAX_BYTES / NET_IF_BYTES,
  outBufFlits: Int = 2 * ETH_STANDARD_MAX_BYTES / NET_IF_BYTES,
  nMemXacts: Int = 8,
  maxAcquireBytes: Int = 64,
  ctrlQueueDepth: Int = 10,
  usePauser: Boolean = false,
  checksumOffload: Boolean = false,
  packetMaxBytes: Int = ETH_STANDARD_MAX_BYTES)

case class NICAttachParams(
  masterWhere: TLBusWrapperLocation = FBUS,
  slaveWhere: TLBusWrapperLocation = PBUS
)

case object NICKey extends Field[Option[NICConfig]](None)
case object NICAttachKey extends Field[NICAttachParams](NICAttachParams())

trait HasNICParameters {
  implicit val p: Parameters
  val nicExternal = p(NICKey).get
  val inBufFlits = nicExternal.inBufFlits
  val outBufFlits = nicExternal.outBufFlits
  val nMemXacts = nicExternal.nMemXacts
  val maxAcquireBytes = nicExternal.maxAcquireBytes
  val ctrlQueueDepth = nicExternal.ctrlQueueDepth
  val usePauser = nicExternal.usePauser
  val checksumOffload = nicExternal.checksumOffload
  val packetMaxBytes = nicExternal.packetMaxBytes
}

abstract class NICLazyModule(implicit p: Parameters)
  extends LazyModule with HasNICParameters

abstract class NICModule(implicit val p: Parameters)
  extends Module with HasNICParameters

abstract class NICBundle(implicit val p: Parameters)
  extends Bundle with HasNICParameters

class PacketArbiter(arbN: Int, rr: Boolean = false)
  extends HellaPeekingArbiter(
    new StreamChannel(NET_IF_WIDTH), arbN,
    (ch: StreamChannel) => ch.last, rr = rr)

class IceNicSendIO extends Bundle {
  val req = Decoupled(UInt(NET_IF_WIDTH.W))
  val comp = Flipped(Decoupled(Bool()))
}

class IceNicRecvIO extends Bundle {
  val req = Decoupled(UInt(NET_IF_WIDTH.W))
  val comp = Flipped(Decoupled(UInt(NET_LEN_BITS.W)))
}

trait IceNicControllerBundle extends Bundle {
  val send = new IceNicSendIO
  val recv = new IceNicRecvIO
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val txcsumReq = Decoupled(new ChecksumRewriteRequest)
  val rxcsumRes = Flipped(Decoupled(new TCPChecksumOffloadResult))
  val csumEnable = Output(Bool())
}

trait IceNicControllerModule extends HasRegMap with HasNICParameters {
  implicit val p: Parameters
  val io: IceNicControllerBundle

  val sendCompDown = WireInit(false.B)

  val qDepth = ctrlQueueDepth
  require(qDepth < (1 << 8))

  def queueCount[T <: Data](qio: QueueIO[T], depth: Int): UInt =
    TwoWayCounter(qio.enq.fire, qio.deq.fire, depth)

  // hold (len, addr) of packets that we need to send out
  val sendReqQueue = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val sendReqCount = queueCount(sendReqQueue.io, qDepth)
  // hold addr of buffers we can write received packets into
  val recvReqQueue = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val recvReqCount = queueCount(recvReqQueue.io, qDepth)
  // count number of sends completed
  val sendCompCount = TwoWayCounter(io.send.comp.fire, sendCompDown, qDepth)
  // hold length of received packets
  val recvCompQueue = Module(new HellaQueue(qDepth)(UInt(NET_LEN_BITS.W)))
  val recvCompCount = queueCount(recvCompQueue.io, qDepth)

  val sendCompValid = sendCompCount > 0.U
  val intMask = RegInit(0.U(2.W))

  io.send.req <> sendReqQueue.io.deq
  io.recv.req <> recvReqQueue.io.deq
  io.send.comp.ready := sendCompCount < qDepth.U
  recvCompQueue.io.enq <> io.recv.comp

  interrupts(0) := sendCompValid && intMask(0)
  interrupts(1) := recvCompQueue.io.deq.valid && intMask(1)

  val sendReqSpace = (qDepth.U - sendReqCount)
  val recvReqSpace = (qDepth.U - recvReqCount)

  def sendCompRead = (ready: Bool) => {
    sendCompDown := sendCompValid && ready
    (sendCompValid, true.B)
  }

  val txcsumReqQueue = Module(new HellaQueue(qDepth)(UInt(49.W)))
  val rxcsumResQueue = Module(new HellaQueue(qDepth)(UInt(2.W)))
  val csumEnable = RegInit(false.B)

  io.txcsumReq.valid := txcsumReqQueue.io.deq.valid
  io.txcsumReq.bits := txcsumReqQueue.io.deq.bits.asTypeOf(new ChecksumRewriteRequest)
  txcsumReqQueue.io.deq.ready := io.txcsumReq.ready

  rxcsumResQueue.io.enq.valid := io.rxcsumRes.valid
  rxcsumResQueue.io.enq.bits := io.rxcsumRes.bits.asUInt
  io.rxcsumRes.ready := rxcsumResQueue.io.enq.ready

  io.csumEnable := csumEnable

  regmap(
    0x00 -> Seq(RegField.w(NET_IF_WIDTH, sendReqQueue.io.enq)),
    0x08 -> Seq(RegField.w(NET_IF_WIDTH, recvReqQueue.io.enq)),
    0x10 -> Seq(RegField.r(1, sendCompRead)),
    0x12 -> Seq(RegField.r(NET_LEN_BITS, recvCompQueue.io.deq)),
    0x14 -> Seq(
      RegField.r(8, sendReqSpace),
      RegField.r(8, recvReqSpace),
      RegField.r(8, sendCompCount),
      RegField.r(8, recvCompCount)),
    0x18 -> Seq(RegField.r(ETH_MAC_BITS, io.macAddr)),
    0x20 -> Seq(RegField(2, intMask)),
    0x28 -> Seq(RegField.w(49, txcsumReqQueue.io.enq)),
    0x30 -> Seq(RegField.r(2, rxcsumResQueue.io.deq)),
    0x31 -> Seq(RegField(1, csumEnable)))
}

case class IceNicControllerParams(address: BigInt, beatBytes: Int)

/*
 * Take commands from the CPU over TL2, expose as Queues
 */
class IceNicController(c: IceNicControllerParams)(implicit p: Parameters)
  extends TLRegisterRouter(
    c.address, "ice-nic", Seq("ucbbar,ice-nic"),
    interrupts = 2, beatBytes = c.beatBytes)(
      new TLRegBundle(c, _)    with IceNicControllerBundle)(
      new TLRegModule(c, _, _) with IceNicControllerModule)

class IceNicSendPath(nInputTaps: Int = 0)(implicit p: Parameters)
    extends NICLazyModule {
  val reader = LazyModule(new StreamReader(
    nMemXacts, outBufFlits, maxAcquireBytes))
  val node = reader.node

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val send = Flipped(new IceNicSendIO)
      val tap = Flipped(Vec(nInputTaps, Decoupled(new StreamChannel(NET_IF_WIDTH))))
      val out = Decoupled(new StreamChannel(NET_IF_WIDTH))
      val rlimit = Input(new RateLimiterSettings)
      val csum = checksumOffload.option(new Bundle {
        val req = Flipped(Decoupled(new ChecksumRewriteRequest))
        val enable = Input(Bool())
      })
    })

    val readreq = reader.module.io.req
    io.send.req.ready := readreq.ready
    readreq.valid := io.send.req.valid
    readreq.bits.address := io.send.req.bits(47, 0)
    readreq.bits.length  := io.send.req.bits(62, 48)
    readreq.bits.partial := io.send.req.bits(63)
    io.send.comp <> reader.module.io.resp

    val preArbOut = if (checksumOffload) {
      val readerOut = reader.module.io.out
      val arb = Module(new PacketArbiter(2))
      val bufFlits = (packetMaxBytes - 1) / NET_IF_BYTES + 1
      val rewriter = Module(new ChecksumRewrite(NET_IF_WIDTH, bufFlits))
      val enable = io.csum.get.enable

      rewriter.io.req <> io.csum.get.req

      arb.io.in(0) <> rewriter.io.stream.out
      arb.io.in(1).valid := !enable && readerOut.valid
      arb.io.in(1).bits  := readerOut.bits
      rewriter.io.stream.in.valid := enable && readerOut.valid
      rewriter.io.stream.in.bits := readerOut.bits
      readerOut.ready := Mux(enable,
        rewriter.io.stream.in.ready, arb.io.in(1).ready)

      arb.io.out
    } else { reader.module.io.out }

    val unlimitedOut = if (nInputTaps > 0) {
      val bufWords = (packetMaxBytes - 1) / NET_IF_BYTES + 1
      val inputs = (preArbOut +: io.tap).map { in =>
        // The packet collection buffer doesn't allow sending the first flit
        // of a packet until the last flit is received.
        // This ensures that we don't lock the arbiter while waiting for data
        // to arrive, which could cause deadocks.
        val buffer = Module(new PacketCollectionBuffer(bufWords))
        buffer.io.in <> in
        buffer.io.out
      }
      val arb = Module(new PacketArbiter(inputs.size, rr = true))
      arb.io.in <> inputs
      arb.io.out
    } else { preArbOut }

    val limiter = Module(new RateLimiter(new StreamChannel(NET_IF_WIDTH)))
    limiter.io.in <> unlimitedOut
    limiter.io.settings := io.rlimit
    io.out <> limiter.io.out
  }
}

class IceNicWriter(implicit p: Parameters) extends NICLazyModule {
  val writer = LazyModule(new StreamWriter(nMemXacts, maxAcquireBytes))
  val node = writer.node

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val recv = Flipped(new IceNicRecvIO)
      val in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH)))
      val length = Flipped(Valid(UInt(NET_LEN_BITS.W)))
    })

    val streaming = RegInit(false.B)
    val byteAddrBits = log2Ceil(NET_IF_BYTES)
    val helper = DecoupledHelper(
      io.recv.req.valid,
      writer.module.io.req.ready,
      io.length.valid, !streaming)

    writer.module.io.req.valid := helper.fire(writer.module.io.req.ready)
    writer.module.io.req.bits.address := io.recv.req.bits
    writer.module.io.req.bits.length := io.length.bits
    io.recv.req.ready := helper.fire(io.recv.req.valid)

    writer.module.io.in.valid := io.in.valid && streaming
    writer.module.io.in.bits := io.in.bits
    io.in.ready := writer.module.io.in.ready && streaming

    io.recv.comp <> writer.module.io.resp

    when (io.recv.req.fire) { streaming := true.B }
    when (io.in.fire && io.in.bits.last) { streaming := false.B }
  }
}

/*
 * Recv frames
 */
class IceNicRecvPath(val tapFuncs: Seq[EthernetHeader => Bool] = Nil)
    (implicit p: Parameters) extends LazyModule {
  val writer = LazyModule(new IceNicWriter)
  val node = TLIdentityNode()
  node := writer.node
  lazy val module = new IceNicRecvPathModule(this)
}

class IceNicRecvPathModule(val outer: IceNicRecvPath)
    extends LazyModuleImp(outer) with HasNICParameters {
  val io = IO(new Bundle {
    val recv = Flipped(new IceNicRecvIO)
    val in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH))) // input stream
    val tap = Vec(outer.tapFuncs.length, Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val csum = checksumOffload.option(new Bundle {
      val res = Decoupled(new TCPChecksumOffloadResult)
      val enable = Input(Bool())
    })
    val buf_free = Output(Vec(1 + outer.tapFuncs.length, UInt(8.W)))
  })

  def tapOutToDropCheck(tapOut: EthernetHeader => Bool) = {
    (header: EthernetHeader, ch: StreamChannel, update: Bool) => {
      val first = RegInit(true.B)
      val drop = tapOut(header) && first
      val dropReg = RegInit(false.B)

      when (update && first) { first := false.B; dropReg := drop }
      when (update && ch.last) { first := true.B; dropReg := false.B }

      drop || dropReg
    }
  }

  def duplicateStream(in: DecoupledIO[StreamChannel], outs: Seq[DecoupledIO[StreamChannel]]) = {
    outs.foreach { out =>
      out.valid := in.valid
      out.bits := in.bits
    }
    in.ready := outs.head.ready
    val outReadys = Cat(outs.map(_.ready))
    assert(outReadys.andR || !outReadys.orR,
      "Duplicated streams must all be ready simultaneously")
    outs
  }

  def invertCheck(check: (EthernetHeader, StreamChannel, Bool) => Bool) =
    (eth: EthernetHeader, ch: StreamChannel, up: Bool) => !check(eth, ch, up)

  val tapDropChecks = outer.tapFuncs.map(func => tapOutToDropCheck(func))
  val pauseDropCheck = if (usePauser) Some(PauseDropCheck(_, _, _)) else None
  val allDropChecks =
    // Drop checks for the primary buffer
    // Drop if the packet should be tapped out or is a pause frame
    Seq(tapDropChecks ++ pauseDropCheck.toSeq) ++
    // Drop checks for the tap buffers
    // For each tap, drop if the packet doesn't match the tap function or is a pause frame
    tapDropChecks.map(check => invertCheck(check) +: pauseDropCheck.toSeq)

  val buffers = allDropChecks.map(dropChecks =>
    Module(new NetworkPacketBuffer(
      inBufFlits,
      maxBytes = packetMaxBytes,
      dropChecks = dropChecks, dropless = usePauser)))
  duplicateStream(io.in, buffers.map(_.io.stream.in))

  io.buf_free := buffers.map(_.io.free)

  io.tap <> buffers.tail.map(_.io.stream.out)
  val bufout = buffers.head.io.stream.out
  val buflen = buffers.head.io.length

  val (csumout, recvreq) = (if (checksumOffload) {
    val offload = Module(new TCPChecksumOffload(NET_IF_WIDTH))
    val offloadReady = offload.io.in.ready || !io.csum.get.enable

    val out = Wire(Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val recvreq = Wire(Decoupled(UInt(NET_IF_WIDTH.W)))
    val reqq = Module(new Queue(UInt(NET_IF_WIDTH.W), 1))

    val enqHelper = DecoupledHelper(
      io.recv.req.valid, reqq.io.enq.ready, recvreq.ready)
    val deqHelper = DecoupledHelper(
      bufout.valid, offloadReady, out.ready, reqq.io.deq.valid)

    reqq.io.enq.valid := enqHelper.fire(reqq.io.enq.ready)
    reqq.io.enq.bits := io.recv.req.bits
    io.recv.req.ready := enqHelper.fire(io.recv.req.valid)
    recvreq.valid := enqHelper.fire(recvreq.ready)
    recvreq.bits := io.recv.req.bits

    out.valid := deqHelper.fire(out.ready)
    out.bits  := bufout.bits
    offload.io.in.valid := deqHelper.fire(offloadReady, io.csum.get.enable)
    offload.io.in.bits := bufout.bits
    bufout.ready := deqHelper.fire(bufout.valid)
    reqq.io.deq.ready := deqHelper.fire(reqq.io.deq.valid, bufout.bits.last)

    io.csum.get.res <> offload.io.result

    (out, recvreq)
  } else { (bufout, io.recv.req) })

  val writer = outer.writer.module
  writer.io.recv.req <> Queue(recvreq, 1)
  io.recv.comp <> writer.io.recv.comp
  writer.io.in <> csumout
  writer.io.length.valid := buflen.valid
  writer.io.length.bits  := buflen.bits
}

class NICIO extends StreamIO(NET_IF_WIDTH) {
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val rlimit = Input(new RateLimiterSettings)
  val pauser = Input(new PauserSettings)

}

/*
 * A simple NIC
 *
 * Expects ethernet frames (see below), but uses a custom transport
 * (see ExtBundle)
 *
 * Ethernet Frame format:
 *   2 bytes |  6 bytes  |  6 bytes    | 2 bytes  | 46-1500B
 *   Padding | Dest Addr | Source Addr | Type/Len | Data
 *
 * @address Starting address of MMIO control registers
 * @beatBytes Width of memory interface (in bytes)
 * @tapOutFuncs Sequence of functions for each output tap.
 *              Each function takes the header of an Ethernet frame
 *              and returns Bool that is true if matching and false if not.
 * @nInputTaps Number of input taps
 *
 */
class IceNIC(address: BigInt, beatBytes: Int = 8,
    tapOutFuncs: Seq[EthernetHeader => Bool] = Nil,
    nInputTaps: Int = 0)
    (implicit p: Parameters) extends NICLazyModule {

  val control = LazyModule(new IceNicController(
    IceNicControllerParams(address, beatBytes)))
  val sendPath = LazyModule(new IceNicSendPath(nInputTaps))
  val recvPath = LazyModule(new IceNicRecvPath(tapOutFuncs))

  val mmionode = TLIdentityNode()
  val dmanode = TLIdentityNode()
  val intnode = control.intnode

  control.node := TLAtomicAutomata() := mmionode
  dmanode := TLWidthWidget(NET_IF_BYTES) := sendPath.node
  dmanode := TLWidthWidget(NET_IF_BYTES) := recvPath.node

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val ext = new NICIO
      val tapOut = Vec(tapOutFuncs.length, Decoupled(new StreamChannel(NET_IF_WIDTH)))
      val tapIn = Flipped(Vec(nInputTaps, Decoupled(new StreamChannel(NET_IF_WIDTH))))
    })

    sendPath.module.io.send <> control.module.io.send
    recvPath.module.io.recv <> control.module.io.recv

    // connect externally
    if (usePauser) {
      val pauser = Module(new Pauser(inBufFlits, 1 + tapOutFuncs.length))
      pauser.io.int.out <> sendPath.module.io.out
      recvPath.module.io.in <> pauser.io.int.in
      io.ext.out <> pauser.io.ext.out
      pauser.io.ext.in <> io.ext.in
      pauser.io.in_free := recvPath.module.io.buf_free
      pauser.io.macAddr := io.ext.macAddr
      pauser.io.settings := io.ext.pauser
    } else {
      recvPath.module.io.in <> io.ext.in
      io.ext.out <> sendPath.module.io.out
    }

    control.module.io.macAddr := io.ext.macAddr
    sendPath.module.io.rlimit := io.ext.rlimit

    io.tapOut <> recvPath.module.io.tap
    sendPath.module.io.tap <> io.tapIn

    if (checksumOffload) {
      sendPath.module.io.csum.get.req <> control.module.io.txcsumReq
      sendPath.module.io.csum.get.enable := control.module.io.csumEnable
      control.module.io.rxcsumRes <> recvPath.module.io.csum.get.res
      recvPath.module.io.csum.get.enable := control.module.io.csumEnable
    } else {
      control.module.io.txcsumReq.ready := false.B
      control.module.io.rxcsumRes.valid := false.B
      control.module.io.rxcsumRes.bits := DontCare
    }
  }
}

class SimNetwork extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val net = Flipped(new NICIOvonly)
  })
  addResource("/vsrc/SimNetwork.v")
  addResource("/csrc/SimNetwork.cc")
  addResource("/csrc/device.h")
  addResource("/csrc/device.cc")
  addResource("/csrc/switch.h")
  addResource("/csrc/switch.cc")
  addResource("/csrc/packet.h")
}


class NICIOvonly extends Bundle {
  val in = Flipped(Valid(new StreamChannel(NET_IF_WIDTH)))
  val out = Valid(new StreamChannel(NET_IF_WIDTH))
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val rlimit = Input(new RateLimiterSettings)
  val pauser = Input(new PauserSettings)

}

object NICIOvonly {
  def apply(nicio: NICIO): NICIOvonly = {
    val vonly = Wire(new NICIOvonly)
    vonly.out.valid := nicio.out.valid
    vonly.out.bits  := nicio.out.bits
    nicio.out.ready := true.B
    nicio.in.valid  := vonly.in.valid
    nicio.in.bits   := vonly.in.bits
    assert(!vonly.in.valid || nicio.in.ready, "NIC input not ready for valid")
    nicio.macAddr := vonly.macAddr
    nicio.rlimit  := vonly.rlimit
    nicio.pauser  := vonly.pauser
    vonly
  }
}

object NICIO {
  def apply(vonly: NICIOvonly): NICIO = {
    val nicio = Wire(new NICIO)
    assert(!vonly.out.valid || nicio.out.ready)
    nicio.out.valid := vonly.out.valid
    nicio.out.bits  := vonly.out.bits
    vonly.in.valid  := nicio.in.valid
    vonly.in.bits   := nicio.in.bits
    nicio.in.ready  := true.B
    vonly.macAddr   := nicio.macAddr
    vonly.rlimit    := nicio.rlimit
    vonly.pauser    := nicio.pauser
    nicio
  }

}
trait CanHavePeripheryIceNIC  { this: BaseSubsystem =>
  private val address = BigInt(0x10016000)
  private val portName = "Ice-NIC"


  val icenicOpt = p(NICKey).map { params =>
    val manager = locateTLBusWrapper(p(NICAttachKey).slaveWhere)
    val client = locateTLBusWrapper(p(NICAttachKey).masterWhere)
    // TODO: currently the controller is in the clock domain of the bus which masters it
    // we assume this is same as the clock domain of the bus the controller masters
    val domain = manager.generateSynchronousDomain.suggestName("icenic_domain")

    val icenic = domain { LazyModule(new IceNIC(address, manager.beatBytes)) }

    manager.coupleTo(portName) { icenic.mmionode := TLFragmenter(manager.beatBytes, manager.blockBytes) := _ }
    client.coupleFrom(portName) { _ :=* icenic.dmanode }
    ibus.fromSync := icenic.intnode

    val inner_io = domain { InModuleBody {
      val inner_io = IO(new NICIOvonly).suggestName("nic")
      inner_io <> NICIOvonly(icenic.module.io.ext)
      inner_io
    } }

    val outer_io = InModuleBody {
      val outer_io = IO(new ClockedIO(new NICIOvonly)).suggestName("nic")
      outer_io.bits <> inner_io
      outer_io.clock := domain.module.clock
      outer_io
    }
    outer_io
  }
}


object NicLoopback {
  def connect(net: Option[NICIOvonly], nicConf: Option[NICConfig], qDepth: Int, latency: Int = 10): Unit = {
    net.foreach { netio =>
      import PauseConsts.BT_PER_QUANTA
      val packetWords = nicConf.get.packetMaxBytes / NET_IF_BYTES
      val packetQuanta = (nicConf.get.packetMaxBytes * 8) / BT_PER_QUANTA
      netio.macAddr := PlusArg("macaddr")
      netio.rlimit.inc := PlusArg("rlimit-inc", 1)
      netio.rlimit.period := PlusArg("rlimit-period", 1)
      netio.rlimit.size := PlusArg("rlimit-size", 8)
      netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency)
      netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta)
      netio.pauser.refresh := PlusArg("pauser-refresh", packetWords)

      if (nicConf.get.usePauser) {
        val pauser = Module(new PauserComplex(qDepth))
        pauser.io.ext.flipConnect(NetDelay(NICIO(netio), latency))
        pauser.io.int.out <> pauser.io.int.in
        pauser.io.macAddr := netio.macAddr + (1 << 40).U
        pauser.io.settings := netio.pauser
      } else {

        netio.in := Pipe(netio.out, latency)
      }
      netio.in.bits.keep := NET_FULL_KEEP
    }
  }

  def connect(net: NICIOvonly, nicConf: NICConfig): Unit = {
    val packetWords = nicConf.packetMaxBytes / NET_IF_BYTES
    NicLoopback.connect(Some(net), Some(nicConf), 4 * packetWords)
  }
}

object SimNetwork {
  def connect(net: Option[NICIOvonly], clock: Clock, reset: Bool) {
    net.foreach { netio =>
      val sim = Module(new SimNetwork)
      sim.io.clock := clock
      sim.io.reset := reset
      sim.io.net <> netio
    }
  }
}

object PacketModifierConnector {

  /**
   * Connects the NIC output through a PacketModifier instance back to the NIC input.
   * Attempts to mimic NicLoopback structure by using a Pipe on the output path.
   *
   * @param netio The NICIOvonly bundle from the harness perspective.
   * @param params The parameters associated with the NIC port (expected to be NICConfig).
   * @param p Implicit CDE Parameters context.
   */
  def connect(netio: NICIOvonly, params: Any)(implicit p: Parameters): Unit = {
    println("[PacketModifierConnector] Connecting NIC through PacketModifier (Queue Input + Pipe Output approach)")

    // 1. Extract NICConfig and Instantiate PacketModifier
    val nicConf = params match {
      case conf: NICConfig => conf
      case _ => throw new Exception(s"PacketModifierConnector requires NICConfig params, got ${params.getClass.getName}")
    }
    // Use latency similar to NicLoopback default
    val latency = 10
    // Ensure you are using the simplified pass-through PacketModifier for this test
    val modifier = Module(new PacketModifier(NET_IF_WIDTH))
    println(s"[PacketModifierConnector] Instantiated PacketModifier with dataWidth=${NET_IF_WIDTH}")
    println(s"[PacketModifierConnector] Using latency = ${latency} for output Pipe")

    // 2. Setup NIC Control Signals (adapted from NicLoopback)
    import PauseConsts._ // Use this if PauseConsts is properly on the classpath
    val packetWords = nicConf.packetMaxBytes / NET_IF_BYTES
    val packetQuanta = if (BT_PER_QUANTA > 0) {
                         (nicConf.packetMaxBytes * 8) / BT_PER_QUANTA
                       } else {
                         println("[PacketModifierConnector] Warning: BT_PER_QUANTA is zero, defaulting packetQuanta to 0")
                         0
                       }
    // Ensure PlusArg has access to chisel3.util functions or specify width explicitly
    netio.macAddr := PlusArg("macaddr", width = 48)
    netio.rlimit.inc := PlusArg("rlimit-inc", 1, width = 32)
    netio.rlimit.period := PlusArg("rlimit-period", 1, width = 32)
    netio.rlimit.size := PlusArg("rlimit-size", 8, width = 32)
    // Use latency value in threshold calculation like NicLoopback
    netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency, width = 32)
    netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta, width = 32)
    netio.pauser.refresh := PlusArg("pauser-refresh", packetWords, width = 32)
    println(s"[PacketModifierConnector] Configured NIC PlusArgs: macaddr, rlimit-*, pauser-*")
    // Check the usePauser flag value during elaboration
    println(s"[PacketModifierConnector] NICConfig usePauser = ${nicConf.usePauser}")


    // --- Connect Data Path: Queue for Input, Pipe for Output ---

    // 1. Adapt NIC output (Valid) -> Modifier input (Decoupled) using Queue
    // This part remains necessary because modifier.io.in is Decoupled.
    val inAdapterQueue = Module(new Queue(chiselTypeOf(netio.out.bits), entries = 2)) // Small queue is likely sufficient
    inAdapterQueue.io.enq.valid := netio.out.valid
    inAdapterQueue.io.enq.bits  := netio.out.bits
    // Connect Queue output (Decoupled) to modifier input (Decoupled)
    modifier.io.in <> inAdapterQueue.io.deq
    println("[PacketModifierConnector] Connected netio.out -> inAdapterQueue -> modifier.io.in")


    // Check if we should mimic the Pauser path - NicLoopback uses PauserComplex here if true.
    // This implementation currently *ignores* usePauser and implements the non-pauser Pipe path.
    // If usePauser is true, this configuration will differ significantly from NicLoopback.
    if (nicConf.usePauser) {
      println("[PacketModifierConnector] WARNING: usePauser is true. This connector currently implements the non-pauser Pipe path. Behavior WILL differ from NicLoopback if usePauser=true.")
    }


    // 2. Adapt Modifier output (Decoupled) -> NIC input (Valid) using Pipe
    // Instantiate the Pipe MODULE. It takes DecoupledIO (enq) and produces DecoupledIO (deq).
    val outQueuePipe = Module(new Queue(chiselTypeOf(modifier.io.out.bits), entries = 1, pipe = true))
    println(s"[PacketModifierConnector] Using Queue(entries=1, pipe=true) for output stage.")

    // Connect modifier output (Decoupled) to Pipe input (Decoupled)
    outQueuePipe.io.enq <> modifier.io.out
    println("[PacketModifierConnector] Connected modifier.io.out -> outQueuePipe.io.enq")

    // Connect Pipe output (Decoupled) to NIC input (Valid)
    netio.in.valid := outQueuePipe.io.deq.valid
    netio.in.bits  := outQueuePipe.io.deq.bits
    // Tell the Pipe's output that the ValidIO sink (netio.in) is always ready to accept.
    outQueuePipe.io.deq.ready := true.B
    println("[PacketModifierConnector] Connected outQueuePipe.io.deq -> netio.in")

    // Force keep bits (Keep this, as NicLoopback does it)
    netio.in.bits.keep := NET_FULL_KEEP
    println("[PacketModifierConnector] Forcing netio.in.bits.keep = NET_FULL_KEEP")

    println("[PacketModifierConnector] Data path connection complete.")
  }
}

object RecursiveDoublingConnector {

  /**
   * Connects the NIC output through a RecursiveDoubling instance back to the NIC input.
   * Uses Queues to adapt between ValidIO (NIC) and DecoupledIO (RecursiveDoubling module).
   *
   * @param netio The NICIOvonly bundle from the harness perspective.
   * @param nicConf The parameters associated with the NIC port (NICConfig object).
   * Changed from Any to NICConfig for type safety, assuming it's always passed.
   * @param p Implicit CDE Parameters context, used to fetch RecursiveDoublingParams.
   */
  def connect(netio: NICIOvonly, nicConf: NICConfig)(implicit p: Parameters): Unit = {
    println("[RecursiveDoublingConnector] Connecting NIC through RecursiveDoubling (Queue Input + Pipe Output approach)")

    // 1. Lookup RecursiveDoublingParams from CDE config and Instantiate Module
    // *** CHANGED: Lookup parameters using the key ***
    val rdParams = p.lift(RecursiveDoublingKey).flatten.getOrElse(
      throw new Exception("RecursiveDoublingParams not found. Did you add WithRecursiveDoubling to your Config?")
    )
    // *** CHANGED: Instantiate RecursiveDoubling with looked-up parameters ***
    val recursiveDoubler = Module(new RecursiveDoubling(rdParams))
    // Use dataWidth from the specific parameters for consistency
    println(s"[RecursiveDoublingConnector] Instantiated RecursiveDoubling with dataWidth=${rdParams.dataWidth}")


    // 2. Setup NIC Control Signals (PlusArgs for simulation)
    // --- NO CHANGES EXPECTED HERE --- (This configures the NIC itself)
    val latency = 10 // Keeping latency consistent with the previous example for pauser calculation
    println(s"[RecursiveDoublingConnector] Using latency = ${latency} for pauser threshold calculation hint")
    
    import PauseConsts._
    val packetWords = nicConf.packetMaxBytes / NET_IF_BYTES
    val packetQuanta = if (BT_PER_QUANTA > 0) {
                         (nicConf.packetMaxBytes * 8) / BT_PER_QUANTA
                       } else {
                         println("[RecursiveDoublingConnector] Warning: BT_PER_QUANTA is zero, defaulting packetQuanta to 0")
                         0
                       }
    // Ensure PlusArg has access to chisel3.util functions or specify width explicitly if needed outside harness
    netio.macAddr := PlusArg("macaddr", width = 48)
    netio.rlimit.inc := PlusArg("rlimit-inc", 1, width = 32)
    netio.rlimit.period := PlusArg("rlimit-period", 1, width = 32)
    netio.rlimit.size := PlusArg("rlimit-size", 8, width = 32)
    // Use latency value in threshold calculation like NicLoopback
    netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency, width = 32)
    netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta, width = 32)
    netio.pauser.refresh := PlusArg("pauser-refresh", packetWords, width = 32)
    println(s"[RecursiveDoublingConnector] Configured NIC PlusArgs: macaddr, rlimit-*, pauser-*")
    println(s"[RecursiveDoublingConnector] NICConfig usePauser = ${nicConf.usePauser}")


    // --- Connect Data Path: Queue for Input, Pipe for Output ---

    // 3. Adapt NIC output (Valid) -> Module input (Decoupled) using Queue
    // --- NO STRUCTURAL CHANGES HERE, just variable names ---
    // Interface types match (StreamChannel), so Queue is appropriate.
    // *** CHANGED: Connect to recursiveDoubler.io.in ***
    val inAdapterQueue = Module(new Queue(chiselTypeOf(netio.out.bits), entries = 2))
    //inAdapterQueue.io.enq <> netio.out // Directly connect ValidIO output to Queue enqueue input
    inAdapterQueue.io.enq.valid := netio.out.valid
    inAdapterQueue.io.enq.bits  := netio.out.bits

    recursiveDoubler.io.in <> inAdapterQueue.io.deq // Connect Queue output to Module input
    // *** CHANGED: Updated print statement ***
    println("[RecursiveDoublingConnector] Connected netio.out -> inAdapterQueue -> recursiveDoubler.io.in")


    // 4. Handle Pauser Warning (Logic identical to PacketModifierConnector)
    // --- NO CHANGES HERE ---
    if (nicConf.usePauser) {
      println("[RecursiveDoublingConnector] WARNING: usePauser is true. This connector currently implements the non-pauser Pipe path. Behavior WILL differ from NicLoopback if usePauser=true.")
    }


    // 5. Adapt Module output (Decoupled) -> NIC input (Valid) using Pipe (Queue with pipe=true)
    // --- NO STRUCTURAL CHANGES HERE, just variable names ---
    // Interface types match, Pipe adapts Decoupled->Decoupled, then we connect to ValidIO.
    // *** CHANGED: Connect from recursiveDoubler.io.out ***
    val outQueuePipe = Module(new Queue(chiselTypeOf(recursiveDoubler.io.out.bits), entries = 1, pipe = true))
    outQueuePipe.io.enq <> recursiveDoubler.io.out // Connect Module output to Pipe input
    // *** CHANGED: Updated print statement ***
    println(s"[RecursiveDoublingConnector] Using Queue(entries=1, pipe=true) for output stage.")
    println("[RecursiveDoublingConnector] Connected recursiveDoubler.io.out -> outQueuePipe.io.enq")

    // Connect Pipe output (Decoupled) to NIC input (Valid)
    // This part converts Decoupled back to Valid
    netio.in.valid := outQueuePipe.io.deq.valid
    netio.in.bits  := outQueuePipe.io.deq.bits
    outQueuePipe.io.deq.ready := true.B // Assume NIC's input can always accept when valid
    // *** CHANGED: Updated print statement ***
    println("[RecursiveDoublingConnector] Connected outQueuePipe.io.deq -> netio.in")


    // 6. Force keep bits (Logic identical to PacketModifierConnector)
    // --- NO CHANGES HERE ---
    // Calculate full keep based on the module's configured dataWidth
    //val fullKeep = ((1 << rdParams.bytesPerWord) - 1).U(rdParams.bytesPerWord.W)
    //netio.in.bits.keep := fullKeep
    // *** CHANGED: Updated print statement and use rdParams ***
    netio.in.bits.keep := NET_FULL_KEEP
    //println(s"[RecursiveDoublingConnector] Forcing netio.in.bits.keep = ${Hexadecimal(fullKeep)}")
    println(s"[RecursiveDoublingConnector] Forcing netio.in.bits.keep = NET_FULL_KEEP")


    // *** CHANGED: Updated final print statement ***
    println("[RecursiveDoublingConnector] Data path connection complete.")
  }
}


// #########################################################################################
// Simple DMA Connector Logic
// #########################################################################################

// Create a wrapper LazyModule that contains a simple DMA implementation
class SimpleDmaControllerWrapper(implicit p: Parameters) extends LazyModule {
  // Create a simple memory system for DMA operations
  val ram = LazyModule(new TLRAM(
    address = AddressSet(0x80000000L, 0x10000000L - 1),
    beatBytes = 8, // Use 8 bytes to match NET_IF_BYTES
    devName = Some("simple-dma-controller-ram")
  ))
  
  // Create a simple DMA controller that directly handles TileLink
  val dmaController = LazyModule(new SimpleDmaController)
  
  // Connect DMA controller to RAM
  ram.node := dmaController.node
  
  lazy val module = new SimpleDmaControllerWrapperModuleImp(this)
}

class SimpleDmaControllerWrapperModuleImp(outer: SimpleDmaControllerWrapper) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val net_in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val net_out = Decoupled(new StreamChannel(NET_IF_WIDTH))
  })
  
  val dmaControllerModule = outer.dmaController.module
  // Instantiate the RAM module to activate TileLink connections
  val _ = outer.ram.module
  
  // Connect network interface
  io.net_in <> dmaControllerModule.io.net_in
  io.net_out <> dmaControllerModule.io.net_out
}

object SimpleDmaControllerConnector {
  def connect(netio: NICIOvonly, nicConf: NICConfig)(implicit p: Parameters): Unit = {
    println("[SimpleDmaControllerConnector] Attaching Simple DMA Controller wrapper to the NIC.")

    // Create the wrapper LazyModule that contains everything
    val wrapper = LazyModule(new SimpleDmaControllerWrapper)
    val wrapperModule = Module(wrapper.module)

    // --- TARGETED PRINTS: Only print key events ---
    val prev_nic_out_valid = RegNext(netio.out.valid)
    val prev_nic_in_valid = RegNext(netio.in.valid)

    // --- Step 3: Connect Data Path ---

    // Connect the NIC output to the wrapper's network input.
    // A simple Queue helps buffer the data.
    val inQueue = Module(new Queue(chiselTypeOf(netio.out.bits), 2))
    inQueue.io.enq.valid := netio.out.valid
    inQueue.io.enq.bits  := netio.out.bits
    wrapperModule.io.net_in <> inQueue.io.deq
    println("[SimpleDmaControllerConnector] Connected netio.out -> inQueue -> wrapper.io.net_in")

    // Connect the wrapper's network output to the NIC input.
    // The wrapper's output is Decoupled, which can drive the NIC's ValidIO directly.
    netio.in.valid := wrapperModule.io.net_out.valid
    netio.in.bits  := wrapperModule.io.net_out.bits
    wrapperModule.io.net_out.ready := true.B // Assume NIC input is always ready
    println("[SimpleDmaControllerConnector] Connected wrapper.io.net_out -> netio.in")

    // Force keep bits to all 1s
    netio.in.bits.keep := NET_FULL_KEEP
    println("[SimpleDmaControllerConnector] Forcing netio.in.bits.keep to all ones.")
    
    // --- TARGETED PRINTS: Only print key events ---
    
    // 1. NIC receives packet from C test (only when it happens)
    when(netio.out.valid && !prev_nic_out_valid) {
      printf("[SimpleDmaControllerConnector] NIC_RECV: data=0x%x, last=%d\n", 
        netio.out.bits.data, netio.out.bits.last)
    }

    // 2. NIC sends packet to module (only when fired)
    when(netio.out.valid && inQueue.io.enq.ready) {
      printf("[SimpleDmaControllerConnector] NIC_TO_MODULE: data=0x%x, last=%d\n", 
        netio.out.bits.data, netio.out.bits.last)
    }
    
    // 3. NIC drops packet (only when it happens)
    when(netio.out.valid && !inQueue.io.enq.ready && 
         (netio.out.valid =/= prev_nic_out_valid || inQueue.io.enq.ready =/= RegNext(inQueue.io.enq.ready))) {
      printf("[SimpleDmaControllerConnector] NIC_DROP: module not ready\n")
    }

    // 4. Module sends packet to NIC (only when fired)
    when(wrapperModule.io.net_out.valid && wrapperModule.io.net_out.ready) {
      printf("[SimpleDmaControllerConnector] MODULE_TO_NIC: data=0x%x, last=%d\n", 
        wrapperModule.io.net_out.bits.data, wrapperModule.io.net_out.bits.last)
    }

    // --- NEW: Debug NIC receive from module ---
    when(netio.in.valid && !prev_nic_in_valid) {
      printf("[SimpleDmaControllerConnector] NIC_RECV_FROM_MODULE: data=0x%x, last=%d\n", 
        netio.in.bits.data, netio.in.bits.last)
    }

    // Add standard PlusArg connections for simulation (same as RecursiveDoublingDMA)
    netio.macAddr := PlusArg("macaddr", width = 48)
    netio.rlimit.inc := PlusArg("rlimit-inc", 1, width = 32)
    netio.rlimit.period := PlusArg("rlimit-period", 1, width = 32)
    netio.rlimit.size := PlusArg("rlimit-size", 8, width = 32)
    val latency = 10
    val packetWords = nicConf.packetMaxBytes / NET_IF_BYTES
    val packetQuanta = if (PauseConsts.BT_PER_QUANTA > 0) {
      (nicConf.packetMaxBytes * 8) / PauseConsts.BT_PER_QUANTA
    } else { 0 }
    netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency, width = 32)
    netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta, width = 32)
    netio.pauser.refresh := PlusArg("pauser-refresh", packetWords, width = 32)
    
    println("[SimpleDmaControllerConnector] Simple DMA Controller wrapper connected successfully.")
  }
}

// #########################################################################################
// RecursiveDoublingWithDMA Connector Logic
// 
// This module provides a wrapper around the RecursiveDoublingWithDMA accelerator
// that integrates it with the NIC (Network Interface Controller). It handles:
// - DMA memory operations through a dedicated RAM subsystem
// - Network packet routing between the NIC and the accelerator
// - Debug output and performance monitoring
// #########################################################################################

/**
 * Wrapper LazyModule that encapsulates the RecursiveDoublingWithDMA accelerator
 * and provides the necessary infrastructure for DMA operations and network integration.
 * 
 * This wrapper is responsible for:
 * 1. Creating and managing a dedicated RAM subsystem for DMA transfers
 * 2. Instantiating the RecursiveDoublingWithDMA accelerator module
 * 3. Connecting the accelerator to the RAM via TileLink protocol
 * 4. Providing a clean interface for network packet routing
 */
class RecursiveDoublingWithDMAWrapper(implicit p: Parameters) extends LazyModule {
  // Debug configuration: Enable/disable debug prints at elaboration time
  // This has no hardware cost when disabled - the printf statements are optimized away
  private val dbgEnabled: Boolean = p.lift(RecursiveDoublingWithDMAKey).flatten.map(_.EnableDebug).getOrElse(false)
  @inline private def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }
  
  // Dedicated RAM subsystem for DMA operations
  // This RAM is used by the RecursiveDoublingWithDMA accelerator to store:
  // - Input data received from the network
  // - Intermediate computation results
  // - Output data to be sent back to the network
  val ram = LazyModule(new TLRAM(
    address = AddressSet(0x80000000L, 0x100000L - 1), // 1MB address space starting at 2GB
    beatBytes = 8, // 8-byte transfers to match NET_IF_BYTES for efficient network data handling
    devName = Some("recursive-doubling-dma-ram")
  ))
  
  // Extract configuration parameters for the RecursiveDoublingWithDMA accelerator
  // These parameters control the accelerator's behavior and resource allocation
  val rdParams = p.lift(RecursiveDoublingWithDMAKey).flatten.getOrElse(
    throw new Exception("RecursiveDoublingWithDMAParams not found. Did you add WithRecursiveDoublingWithDMA to your Config?")
  )
  
  // Instantiate the main RecursiveDoublingWithDMA accelerator module
  val recursiveDoublingDMA = LazyModule(new RecursiveDoublingWithDMA(rdParams))
  
  // Connect the accelerator to the RAM subsystem via TileLink protocol
  // This enables the accelerator to perform DMA operations on the dedicated RAM
  ram.node := recursiveDoublingDMA.node
  
  // Create the module implementation
  lazy val module = new RecursiveDoublingWithDMAWrapperModuleImp(this)
}

/**
 * Module implementation for the RecursiveDoublingWithDMA wrapper.
 * This class provides the actual hardware implementation that connects the
 * accelerator to the network interface and manages the RAM subsystem.
 */
class RecursiveDoublingWithDMAWrapperModuleImp(outer: RecursiveDoublingWithDMAWrapper) extends LazyModuleImp(outer) {
  // Network interface I/O bundle
  // net_in: Receives network packets from the NIC (Flipped because it's an input to this module)
  // net_out: Sends processed packets back to the NIC
  // dstMacAddr: Destination MAC address for outgoing packets
  // level0SrcMac: Source MAC from Level 0 packets (for Level 4 destination)
  val io = IO(new Bundle {
    val net_in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val net_out = Decoupled(new StreamChannel(NET_IF_WIDTH))
    val dstMacAddr = Output(UInt(ETH_MAC_BITS.W))  // Destination MAC for outgoing packets
    val level0SrcMac = Input(UInt(ETH_MAC_BITS.W))  // Source MAC extracted from incoming packets
  })
  
  // Get the actual hardware module instance of the RecursiveDoublingWithDMA accelerator
  val recursiveDoublingDMAModule = outer.recursiveDoublingDMA.module
  
  // Instantiate the RAM module to activate TileLink connections
  // The underscore assignment ensures the module is instantiated but we don't need to reference it
  val _ = outer.ram.module
  
  // Connect the network interface between the wrapper and the accelerator
  // This creates the data path for packets flowing in and out of the accelerator
  io.net_in <> recursiveDoublingDMAModule.io.in
  io.net_out <> recursiveDoublingDMAModule.io.out
  io.dstMacAddr := recursiveDoublingDMAModule.io.dstMacAddr  // Pass destination MAC through
  recursiveDoublingDMAModule.io.level0SrcMac := io.level0SrcMac  // Deliver Level 0 source MAC to accelerator
}


/**
 * Ethernet Header Extractor Module
 * Extracts Ethernet headers from incoming packets and provides the source MAC address.
 * The Ethernet header words are removed from the stream so the accelerator receives payload only.
 */
class EthernetHeaderExtractor(dataWidth: Int, dbgEnabled: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new StreamChannel(dataWidth)))
    val out = Decoupled(new StreamChannel(dataWidth))
    val srcMacAddr = Output(UInt(ETH_MAC_BITS.W))
  })

  // Debug prints controlled by parent connector
  @inline def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }

  val headerWords = ETH_HEAD_BYTES * 8 / dataWidth
  require(headerWords >= 1, "Ethernet header must span at least one word on the stream interface.")

  val s_idle :: s_header :: s_data :: Nil = Enum(3)
  val state = RegInit(s_idle)
  val wordIdx = RegInit(0.U(log2Ceil(headerWords + 1).W))
  val storedHeaderWords = RegInit(VecInit(Seq.fill(headerWords)(0.U(dataWidth.W))))
  val extractedSrcMac = RegInit(0.U(ETH_MAC_BITS.W))

  io.in.ready := false.B
  io.out.valid := false.B
  io.out.bits := DontCare
  io.srcMacAddr := extractedSrcMac


  switch(state) {
    is(s_idle) {
      io.in.ready := true.B
      when(io.in.fire) {
        storedHeaderWords(0) := io.in.bits.data
        when(headerWords.U === 1.U) {
          // Single-word header: extract MAC immediately
          val headerVec = VecInit(Seq(io.in.bits.data.asUInt))
          val ethHeader = EthernetHeader(headerVec, dataWidth)
          // Only update extracted MAC if it's valid (not zero, not broadcast)
          when(ethHeader.srcmac =/= 0.U && ethHeader.srcmac =/= IceNetConsts.ETH_BCAST_MAC) {
            extractedSrcMac := ethHeader.srcmac
            dprintf(p"[EthernetHeaderExtractor] Extracted source MAC: 0x${Hexadecimal(ethHeader.srcmac)} (dst=0x${Hexadecimal(ethHeader.dstmac)}, type=0x${Hexadecimal(ethHeader.ethType)})\n")
          }.otherwise {
            dprintf(p"[EthernetHeaderExtractor] Ignoring invalid source MAC: 0x${Hexadecimal(ethHeader.srcmac)} (zero or broadcast)\n")
          }
          state := s_data
        }.otherwise {
          // Multi-word header: need to collect more words
          wordIdx := 1.U
          state := s_header
        }
      }
    }
    is(s_header) {
      io.in.ready := true.B
      when(io.in.fire) {
        // Store the current word at its position
        storedHeaderWords(wordIdx) := io.in.bits.data
        val nextIdx = wordIdx + 1.U
        when(nextIdx === headerWords.U) {
          // All header words received, extract MAC
          val headerVec = storedHeaderWords.map(_.asUInt)
          val ethHeader = EthernetHeader(headerVec, dataWidth)
          // Only update extracted MAC if it's valid (not zero, not broadcast)
          // This prevents storing garbage from uninitialized state or corrupted packets
          when(ethHeader.srcmac =/= 0.U && ethHeader.srcmac =/= IceNetConsts.ETH_BCAST_MAC) {
            extractedSrcMac := ethHeader.srcmac
            dprintf(p"[EthernetHeaderExtractor] Extracted source MAC: 0x${Hexadecimal(ethHeader.srcmac)} (dst=0x${Hexadecimal(ethHeader.dstmac)}, type=0x${Hexadecimal(ethHeader.ethType)})\n")
          }.otherwise {
            dprintf(p"[EthernetHeaderExtractor] Ignoring invalid source MAC: 0x${Hexadecimal(ethHeader.srcmac)} (zero or broadcast)\n")
          }
          wordIdx := 0.U
          state := s_data
        }.otherwise {
          wordIdx := nextIdx
        }
      }
    }
    is(s_data) {
      io.in.ready := io.out.ready
      io.out.valid := io.in.valid
      io.out.bits := io.in.bits

      when(io.in.fire && io.in.bits.last) {
        state := s_idle
        wordIdx := 0.U
      }
    }
  }
}

/**
 * Ethernet Header Prepender Module
 * Prepends Ethernet headers to packets with the specified source and destination MAC addresses.
 * This module ensures packets are routed to the correct destination instead of using broadcast.
 */
class EthernetHeaderPrepender(dataWidth: Int, dbgEnabled: Boolean = false) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new StreamChannel(dataWidth)))
    val out = Decoupled(new StreamChannel(dataWidth))
    val srcMacAddr = Input(UInt(ETH_MAC_BITS.W))
    val dstMacAddr = Input(UInt(ETH_MAC_BITS.W))
  })
  
  // Debug prints controlled by parent connector
  @inline def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }
  
  val ETH_HEAD_WORDS = ETH_HEAD_BYTES * 8 / dataWidth  // Number of words for Ethernet header
  val s_idle :: s_header :: s_data :: Nil = Enum(3)
  val state = RegInit(s_idle)
  val headerWordIdx = RegInit(0.U(log2Ceil(ETH_HEAD_WORDS + 1).W))
  
  // Capture destination MAC when packet starts (to ensure it's stable for entire packet)
  val capturedDstMac = Reg(UInt(ETH_MAC_BITS.W))
  val capturedSrcMac = Reg(UInt(ETH_MAC_BITS.W))
  
  // Ethernet header construction
  // Format: [padding(2B) | dstmac(6B) | srcmac(6B) | ethType(2B)]
  // For 64-bit words: word0 = [padding(2B) | dstmac[5:0](6B)], word1 = [srcmac(6B) | ethType(2B)]
  val ethHeader = Wire(new EthernetHeader)
  ethHeader.dstmac := capturedDstMac
  ethHeader.srcmac := capturedSrcMac
  ethHeader.ethType := 0x0800.U(ETH_TYPE_BITS.W)  // IPv4
  ethHeader.padding := 0.U(ETH_PAD_BITS.W)
  
  val headerWords = ethHeader.toWords(dataWidth)
  
  io.in.ready := false.B
  io.out.valid := false.B
  io.out.bits := DontCare
  
  switch(state) {
    is(s_idle) {
      // Wait for packet start, capture MAC addresses, then send header
      when(io.in.valid) {
        capturedDstMac := io.dstMacAddr
        capturedSrcMac := io.srcMacAddr
        dprintf(p"[EthernetHeaderPrepender] Prepending header: src=0x${Hexadecimal(io.srcMacAddr)}, dst=0x${Hexadecimal(io.dstMacAddr)}\n")
        state := s_header
        headerWordIdx := 0.U
      }
    }
    is(s_header) {
      // Send Ethernet header words
      io.out.valid := true.B
      io.out.bits.data := headerWords(headerWordIdx)
      io.out.bits.keep := NET_FULL_KEEP
      io.out.bits.last := false.B
      
      when(io.out.ready) {
        headerWordIdx := headerWordIdx + 1.U
        when(headerWordIdx === (ETH_HEAD_WORDS - 1).U) {
          state := s_data
        }
      }
    }
    is(s_data) {
      // Forward packet data
      io.in.ready := io.out.ready
      io.out.valid := io.in.valid
      io.out.bits := io.in.bits
      
      when(io.in.fire && io.in.bits.last) {
        state := s_idle
      }
    }
  }
}

/**
 * Connector object that integrates the RecursiveDoublingWithDMA accelerator with the NIC.
 * This object provides the main connection logic that wires up the accelerator to the
 * network interface controller, including packet routing, buffering, and debug monitoring.
 */
object RecursiveDoublingWithDMAConnector {
  /**
   * Main connection function that attaches the RecursiveDoublingWithDMA accelerator to the NIC.
   * 
   * @param netio The NIC I/O interface that handles network packet transmission/reception
   * @param nicConf Configuration parameters for the NIC (packet sizes, timing, etc.)
   * @param p Implicit parameters containing system configuration
   */
  def connect(netio: NICIO, nicConf: NICConfig)(implicit p: Parameters): Unit = {
    // Debug configuration: Enable/disable debug prints at elaboration time
    // This has no hardware cost when disabled - printf statements are optimized away
    val dbgEnabled: Boolean = p.lift(RecursiveDoublingWithDMAKey).flatten.map(_.EnableDebug).getOrElse(false)
    @inline def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }
    
    println("[RecursiveDoublingWithDMAConnector] Attaching RecursiveDoublingWithDMA wrapper to the NIC.")    

    // Create the wrapper LazyModule that contains the accelerator and RAM subsystem
    val wrapper = LazyModule(new RecursiveDoublingWithDMAWrapper)
    val wrapperModule = Module(wrapper.module)

    // ========================================================================================
    // Debug and Monitoring Infrastructure
    // ========================================================================================
    
    // Previous state tracking for edge detection in debug prints
    val prev_nic_out_valid = RegNext(netio.out.valid)
    val prev_nic_in_valid = RegNext(netio.in.valid)
    
    // Packet and word counters for debug output management
    // These counters help limit debug output to avoid overwhelming the simulation logs
    val inPacketCount = RegInit(0.U(16.W))   // Total number of packets received from NIC
    val outPacketCount = RegInit(0.U(16.W))  // Total number of packets sent to NIC
    val inWordCount = RegInit(0.U(8.W))      // Word count within current input packet
    val outWordCount = RegInit(0.U(8.W))     // Word count within current output packet
    val MAX_DEBUG_WORDS = 3.U                // Print first 3 and last 3 words per packet to reduce log spam

    // ========================================================================================
    // Data Path Connection and Buffering
    // ========================================================================================

    // Input path: NIC -> Ethernet Header Extractor -> Input Queue -> Accelerator
    // Extract Ethernet headers to obtain source MAC (Level 0 origin) and remove headers before accelerator
    val headerExtractor = Module(new EthernetHeaderExtractor(NET_IF_WIDTH, dbgEnabled))
    headerExtractor.io.in <> netio.out

    val inQueue = Module(new Queue(chiselTypeOf(headerExtractor.io.out.bits), 256))
    inQueue.io.enq <> headerExtractor.io.out
    wrapperModule.io.net_in <> inQueue.io.deq
    wrapperModule.io.level0SrcMac := headerExtractor.io.srcMacAddr

    println("[RecursiveDoublingWithDMAConnector] Connected netio.out -> headerExtractor -> inQueue (256 entries) -> wrapper.io.net_in")
    println("[RecursiveDoublingWithDMAConnector] Ethernet header extractor feeding Level 0 source MAC to accelerator")

    // Output path: Accelerator -> Output Queue -> Ethernet Header Prepender -> NIC
    // Create a large output queue to buffer processed packets from the accelerator
    // This allows the accelerator to continue processing while the NIC handles transmission
    // Queue size of 256 entries can handle multiple large packets (16 packets * 130 words = 2080 words)
    val outQueue = Module(new Queue(chiselTypeOf(wrapperModule.io.net_out.bits), 256))
    outQueue.io.enq <> wrapperModule.io.net_out
    println("[RecursiveDoublingWithDMAConnector] Added output queue (256 entries) after module")

    // Ethernet header prepender: Adds Ethernet headers with correct destination MAC
    // This ensures packets are routed to the correct destination instead of broadcast
    val headerPrepender = Module(new EthernetHeaderPrepender(NET_IF_WIDTH, dbgEnabled))
    headerPrepender.io.srcMacAddr := netio.macAddr
    headerPrepender.io.dstMacAddr := wrapperModule.io.dstMacAddr
    headerPrepender.io.in <> outQueue.io.deq
    
    // Connect the header prepender to the NIC input
    // This creates the final link in the data path: accelerator -> queue -> header prepender -> NIC
    netio.in <> headerPrepender.io.out
    println("[RecursiveDoublingWithDMAConnector] Connected wrapper.io.net_out -> outQueue -> headerPrepender -> netio.in")
    
    // ========================================================================================
    // NIC Input Backpressure Monitoring
    // ========================================================================================
    
    // Monitor when the NIC input buffer is full and the output queue is trying to send data
    // This helps identify potential bottlenecks in the data path
    val nicInputBlockCounter = RegInit(0.U(8.W))      // Counts consecutive cycles of backpressure
    val nicInputBlockPrintCount = RegInit(0.U(4.W))   // Limits debug output to first 5 instances
    when(outQueue.io.deq.valid && !netio.in.ready) {
      nicInputBlockCounter := nicInputBlockCounter + 1.U
      when(nicInputBlockCounter === 0.U && nicInputBlockPrintCount < 5.U) { // Print only first 5 instances
        dprintf(p"[RecursiveDoublingWithDMAConnector] NIC_INPUT_NOT_READY[${nicInputBlockPrintCount}]: NIC input buffer full\n")
        nicInputBlockPrintCount := nicInputBlockPrintCount + 1.U
      }
    }.otherwise {
      nicInputBlockCounter := 0.U
    }

    // ========================================================================================
    // Network Interface Configuration
    // ========================================================================================

    // Force keep bits to all 1s to indicate all bytes in the packet are valid
    // This is necessary because the accelerator processes complete packets and we want to
    // ensure the NIC treats all data as valid
    netio.in.bits.keep := NET_FULL_KEEP
    println("[RecursiveDoublingWithDMAConnector] Forcing netio.in.bits.keep to all ones.")
    
    // ========================================================================================
    // Debug Output and Packet Tracking
    // ========================================================================================
    
    // Track incoming packet and word counts for debug output
    // This helps monitor packet flow and identify potential issues
    when(netio.out.valid && !prev_nic_out_valid) {
      when(netio.out.bits.last) {
        inPacketCount := inPacketCount + 1.U
        inWordCount := 0.U // Reset word count for next packet
      }.otherwise {
        inWordCount := inWordCount + 1.U
      }
    }

    // Debug print: NIC receives data from external source (C test)
    // Only print first few and last few words per packet to avoid log spam
    when(netio.out.valid && !prev_nic_out_valid && 
         (inWordCount < MAX_DEBUG_WORDS || inWordCount >= 127.U || netio.out.bits.last)) { // 130 total words
      dprintf(p"[RecursiveDoublingWithDMAConnector] NIC_RECV[P${inPacketCount}W${inWordCount}]: data=0x${Hexadecimal(netio.out.bits.data)}, last=${netio.out.bits.last}\n")
    }

    // Debug print: NIC sends data to accelerator module
    // Only print first few and last few words per packet to avoid log spam
    when(netio.out.valid && inQueue.io.enq.ready && 
         (inWordCount < MAX_DEBUG_WORDS || inWordCount >= 127.U || netio.out.bits.last)) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] NIC_TO_MODULE[P${inPacketCount}W${inWordCount}]: data=0x${Hexadecimal(netio.out.bits.data)}, last=${netio.out.bits.last}\n")
    }
    
    // Debug print: Packet dropped due to input queue being full
    // This indicates the accelerator is not keeping up with incoming data
    when(netio.out.valid && !inQueue.io.enq.ready && 
         (netio.out.valid =/= prev_nic_out_valid || inQueue.io.enq.ready =/= RegNext(inQueue.io.enq.ready))) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] NIC_DROP: module not ready\n")
    }

    // Track outgoing packet and word counts for debug output
    when(wrapperModule.io.net_out.valid && outQueue.io.enq.ready) {
      when(wrapperModule.io.net_out.bits.last) {
        outPacketCount := outPacketCount + 1.U
        outWordCount := 0.U // Reset word count for next packet
      }.otherwise {
        outWordCount := outWordCount + 1.U
      }
    }

    // Debug print: Accelerator sends data to output queue
    // Only print first few and last few words per packet to avoid log spam
    when(wrapperModule.io.net_out.valid && outQueue.io.enq.ready && 
         (outWordCount < MAX_DEBUG_WORDS || outWordCount >= 127.U || wrapperModule.io.net_out.bits.last)) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] MODULE_TO_QUEUE[P${outPacketCount}W${outWordCount}]: data=0x${Hexadecimal(wrapperModule.io.net_out.bits.data)}, last=${wrapperModule.io.net_out.bits.last}\n")
    }

    // Debug print: Output queue sends complete packet to NIC
    // This provides a summary when each packet is fully transmitted
    when(outQueue.io.deq.valid && netio.in.ready && outQueue.io.deq.bits.last) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] QUEUE_TO_NIC[P${outPacketCount}]: packet complete (130 words)\n")
    }

    // ========================================================================================
    // Queue Status Monitoring and Backpressure Detection
    // ========================================================================================
    
    // Monitor output queue fullness to detect potential bottlenecks
    // Only print when queue is nearly full (>240/256) or completely empty to avoid spam
    val queueFullness = outQueue.io.count
    val prevQueueFullness = RegNext(queueFullness)
    when(queueFullness =/= prevQueueFullness && (queueFullness > 240.U || queueFullness === 0.U)) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] OUTPUT_QUEUE_STATUS: count=${queueFullness}/256\n")
    }

    // Monitor when the accelerator is blocked by a full output queue
    // This indicates the NIC is not keeping up with the accelerator's output rate
    val blockCounter = RegInit(0.U(8.W))
    when(wrapperModule.io.net_out.valid && !outQueue.io.enq.ready) {
      blockCounter := blockCounter + 1.U
      when(blockCounter === 0.U) { // Print only once every 256 cycles to avoid spam
        dprintf(p"[RecursiveDoublingWithDMAConnector] MODULE_BLOCKED: output queue full, count=${queueFullness}/256\n")
      }
    }.otherwise {
      blockCounter := 0.U
    }

    // Debug print: NIC receives complete packet from accelerator
    // This confirms successful end-to-end packet transmission
    when(netio.in.valid && !prev_nic_in_valid && netio.in.bits.last) {
      dprintf(p"[RecursiveDoublingWithDMAConnector] NIC_RECV_FROM_MODULE[P${outPacketCount}]: packet complete (130 words)\n")
    }

    // ========================================================================================
    // NIC Configuration and Simulation Parameters
    // ========================================================================================
    
    // Configure NIC with standard PlusArg connections for simulation
    // These parameters can be overridden at runtime via command-line arguments
    // Default MAC address based on node rank: 00:12:6D:00:00:XX where XX is node rank
    val nodeRank = PlusArg("node_rank", default = 0, docstring = "Node rank (0-7 for 8 nodes)", width = 8)
    val baseMac = (0x00126D000000L).U(48.W)
    val defaultMacAddr = baseMac | nodeRank(7, 0).asUInt
    val macAddrValue = Mux(PlusArg("macaddr", width = 48).orR, 
                          PlusArg("macaddr", width = 48),
                          defaultMacAddr)
    netio.macAddr := macAddrValue
    
    // Extract node rank from MAC address (last byte) and pass to module via PlusArg
    // MAC format: 00:12:6D:00:00:XX where XX is typically the node rank
    // This allows the module to calculate partner nodes for recursive doubling
    // Note: PlusArg for node_rank can override this if MAC doesn't follow pattern
    val nodeRankFromMac = macAddrValue(7, 0)  // Extract last byte as rank
    // The module will use PlusArg("node_rank") which can be set explicitly or defaults to 0
    netio.rlimit.inc := PlusArg("rlimit-inc", 1, width = 32)
    netio.rlimit.period := PlusArg("rlimit-period", 1, width = 32)
    netio.rlimit.size := PlusArg("rlimit-size", 8, width = 32)
    
    // Calculate packet flow control parameters based on NIC configuration
    val latency = 10  // Base latency in cycles
    val packetWords = nicConf.packetMaxBytes / NET_IF_BYTES  // Number of words per packet
    val packetQuanta = if (PauseConsts.BT_PER_QUANTA > 0) {
      (nicConf.packetMaxBytes * 8) / PauseConsts.BT_PER_QUANTA  // Bits per quanta for flow control
    } else { 0 }
    
    // Configure pause mechanism for flow control
    // This prevents buffer overflow by pausing transmission when buffers are nearly full
    netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency, width = 32)
    netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta, width = 32)
    netio.pauser.refresh := PlusArg("pauser-refresh", packetWords, width = 32)
    
    println("[RecursiveDoublingWithDMAConnector] RecursiveDoublingWithDMA wrapper connected successfully.")
  }
}