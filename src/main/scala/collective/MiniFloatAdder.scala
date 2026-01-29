/*
 * MiniFloatAdder.scala
 * 
 * Parameterized floating-point adder for small formats (BFloat16, DLFloat).
 * Created to avoid Hardfloat's lowMask issues with small sigWidth values.
 * 
 * Format-specific behavior:
 *   - BFloat16: IEEE 754 style (e=0 is subnormal/zero, e=255 is Inf/NaN, RNE rounding)
 *   - DLFloat:  Custom format (no subnormals, only e=63 m=511 is NaN-Inf, RNU rounding)
 * 
 * Design: Single-cycle combinational.
 */

package icenet

import chisel3._
import chisel3.util._

/**
 * Parameterized floating-point adder for mini-float formats.
 * 
 * @param expWidth   Number of exponent bits
 * @param mantWidth  Number of mantissa bits (excluding hidden bit)
 * @param isDLFloat  If true, use DLFloat semantics; if false, use BFloat16/IEEE semantics
 * @param enableDebug If true, emit debug prints to trace intermediate values
 * 
 * Format layout: [sign(1)] [exponent(expWidth)] [mantissa(mantWidth)]
 * Total bits = 1 + expWidth + mantWidth
 */
class MiniFloatAdder(expWidth: Int, mantWidth: Int, isDLFloat: Boolean = false, enableDebug: Boolean = false) extends Module {
  require(expWidth >= 4 && expWidth <= 8, "expWidth must be between 4 and 8")
  require(mantWidth >= 4 && mantWidth <= 23, "mantWidth must be between 4 and 23")
  
  // Debug print helper (no cost when disabled at elaboration time)
  // @inline private def dprintf(msg: chisel3.Printable): Unit = if (enableDebug) { printf(msg) }
  
  val totalBits = 1 + expWidth + mantWidth
  val bias = (1 << (expWidth - 1)) - 1  // 127 for expWidth=8, 31 for expWidth=6
  val maxExp = (1 << expWidth) - 1      // 255 for expWidth=8, 63 for expWidth=6
  val maxMant = (1 << mantWidth) - 1    // 127 for mantWidth=7, 511 for mantWidth=9
  
  // Extended mantissa width: hidden bit + mantissa + guard/round/sticky bits
  val extMantWidth = 1 + mantWidth + 3  // +3 for guard, round, sticky
  
  val io = IO(new Bundle {
    val a      = Input(UInt(totalBits.W))
    val b      = Input(UInt(totalBits.W))
    val result = Output(UInt(totalBits.W))
  })
  
  // ==========================================================================
  // Step 1: Unpack inputs with format-specific handling
  // ==========================================================================
  def unpack(x: UInt): (Bool, UInt, UInt, Bool) = {
    val sign = x(totalBits - 1)
    val exp  = x(totalBits - 2, mantWidth)
    val mant = x(mantWidth - 1, 0)
    
    // Determine if this is a zero value
    // DLFloat: only e=0, m=0 is zero (e=0 m≠0 is normal with hidden bit 1)
    // BFloat16: e=0 is zero/subnormal (hidden bit 0)
    val isZero = if (isDLFloat) {
      exp === 0.U && mant === 0.U
    } else {
      exp === 0.U  // For BFloat16, treat all e=0 as zero (simplified subnormal handling)
    }
    
    (sign, exp, mant, isZero)
  }
  
  val (signA, expA, mantA, isZeroA) = unpack(io.a)
  val (signB, expB, mantB, isZeroB) = unpack(io.b)
  
  // Add hidden bit
  // DLFloat: hidden bit is 1 for all non-zero values (including e=0, m≠0)
  // BFloat16: hidden bit is 0 for e=0 (subnormal/zero)
  val fullMantA = if (isDLFloat) {
    Cat(!isZeroA, mantA)
  } else {
    Cat(expA =/= 0.U, mantA)
  }
  
  val fullMantB = if (isDLFloat) {
    Cat(!isZeroB, mantB)
  } else {
    Cat(expB =/= 0.U, mantB)
  }
  
  // Effective exponent for DLFloat e=0:
  // Per DLFloat paper: Value = 2^(e-31) * 1.m for ALL non-zero values
  // For e=0 m≠0: exponent is 2^(0-31) = 2^-31, NOT 2^(1-31)
  // So effective exponent should remain 0, not be promoted to 1
  val effExpA = expA  // DLFloat and BFloat16 both use raw exponent
  val effExpB = expB
  
  // ==========================================================================
  // Step 2: Determine which operand is larger (by exponent, then mantissa)
  // ==========================================================================
  val aGtB = (effExpA > effExpB) || (effExpA === effExpB && fullMantA >= fullMantB)
  
  // Swap so that 'larger' always has the larger exponent
  val signL = Mux(aGtB, signA, signB)
  val expL  = Mux(aGtB, effExpA, effExpB)
  val mantL = Mux(aGtB, fullMantA, fullMantB)
  val signS = Mux(aGtB, signB, signA)
  val expS  = Mux(aGtB, effExpB, effExpA)
  val mantS = Mux(aGtB, fullMantB, fullMantA)
  
  // ==========================================================================
  // Step 3: Align mantissas - shift smaller mantissa right
  // ==========================================================================
  val expDiff = expL - expS
  
  // Extend mantissa with guard/round/sticky bits for precision
  val mantLExt = Cat(mantL, 0.U(3.W))
  
  // Shift smaller mantissa right, capturing sticky bits
  val maxShift = extMantWidth.U
  val shiftAmt = Mux(expDiff > maxShift, maxShift, expDiff)
  val mantSExt = Cat(mantS, 0.U(3.W))
  val mantSShifted = mantSExt >> shiftAmt
  
  // Sticky bit from shifted-out bits
  val stickyMask = (1.U << shiftAmt) - 1.U
  val stickyBit = (mantSExt & stickyMask).orR
  val mantSAligned = mantSShifted | stickyBit
  
  // ==========================================================================
  // Step 4: Add or subtract based on signs
  // ==========================================================================
  val effectiveSub = signL =/= signS
  
  // CRITICAL: Widen operands BEFORE addition to capture overflow bit!
  // Chisel addition of two N-bit values gives N-bit result (truncation).
  // We need (extMantWidth + 1) bits to detect overflow.
  val mantLExtWide = Cat(0.U(1.W), mantLExt)   // 14 bits
  val mantSAlignedWide = Cat(0.U(1.W), mantSAligned)  // 14 bits
  
  val mantSum = Wire(UInt((extMantWidth + 1).W))
  when(effectiveSub) {
    mantSum := mantLExtWide - mantSAlignedWide
  }.otherwise {
    mantSum := mantLExtWide + mantSAlignedWide
  }
  
  // Result sign: same as larger operand
  val resultSign = signL
  
  // ==========================================================================
  // Step 5: Normalize the result
  // ==========================================================================
  // Count leading zeros on the non-overflow portion (extMantWidth bits, not extMantWidth+1)
  // This ensures we get the correct shift when overflow bit is 0 but bit extMantWidth-1 is 1
  val mantSumNoOverflow = mantSum(extMantWidth - 1, 0)
  val leadingZeros = PriorityEncoder(Reverse(mantSumNoOverflow))
  
  val overflow = mantSum(extMantWidth)
  val mantNorm = Wire(UInt(extMantWidth.W))
  val expAdj = Wire(SInt((expWidth + 2).W))
  
  when(mantSum === 0.U) {
    mantNorm := 0.U
    expAdj := (-(bias + 1)).S
  }.elsewhen(overflow) {
    mantNorm := mantSum(extMantWidth, 1) | mantSum(0)
    expAdj := 1.S
  }.otherwise {
    val normalPos = (extMantWidth - 1).U
    val shiftLeft = Mux(leadingZeros > normalPos, normalPos, leadingZeros)
    mantNorm := (mantSumNoOverflow << shiftLeft)(extMantWidth - 1, 0)
    expAdj := -(shiftLeft.zext)
  }
  
  val expResult = expL.zext + expAdj
  
  // Debug Step 5: Print normalization results
  // dprintf(p"[MiniFloat] mantSum=${mantSum} overflow=${overflow} expL=${expL} expAdj=${expAdj} expResult=${expResult}\n")
  
  // ==========================================================================
  // Step 6: Round and pack result
  // ==========================================================================
  val hiddenBit = mantNorm(extMantWidth - 1)
  val finalMant = mantNorm(extMantWidth - 2, 3)
  val guardBit = mantNorm(2)
  val roundBit = mantNorm(1)
  val stickyOut = mantNorm(0)
  
  // Rounding mode:
  // DLFloat: Round-nearest-up (round up if guard bit is set)
  // BFloat16: Round-nearest-even (round up if guard && (round || sticky || LSB))
  val roundUp = if (isDLFloat) {
    guardBit  // RNU: round up if guard bit set
  } else {
    guardBit && (roundBit || stickyOut || finalMant(0))  // RNE
  }
  
  val mantRounded = finalMant + roundUp
  
  // Handle mantissa overflow from rounding
  val mantRoundOverflow = roundUp && finalMant.andR
  val expFinal = Mux(mantRoundOverflow, expResult + 1.S, expResult)
  val mantFinal = Mux(mantRoundOverflow, 0.U(mantWidth.W), mantRounded)
  
  // ==========================================================================
  // Step 7: Handle special cases and pack output
  // ==========================================================================
  val isResultZero = (mantSum === 0.U) || (expFinal <= 0.S)
  
  // Overflow handling:
  // DLFloat: e=63 m=0-510 are normal, only e=63 m=511 is NaN-Inf -> saturate to e=63 m=510
  // BFloat16: e=255 with m=0 is Infinity (per IEEE 754)
  val isResultOverflow = expFinal >= maxExp.S
  
  val resultExp = Wire(UInt(expWidth.W))
  val resultMant = Wire(UInt(mantWidth.W))
  
  // Result sign handling:
  // DLFloat paper: sign of zero is ignored (always positive zero)
  // BFloat16: preserves sign of zero per IEEE 754
  val finalSign = if (isDLFloat) {
    Mux(isResultZero, false.B, resultSign)  // DLFloat: zero is unsigned
  } else {
    resultSign  // BFloat16: preserve sign
  }
  
  when(isResultZero) {
    resultExp := 0.U
    resultMant := 0.U
  }.elsewhen(isResultOverflow) {
    if (isDLFloat) {
      // DLFloat: saturate to max normal (e=63, m=510), avoiding NaN-Inf (m=511)
      resultExp := maxExp.U
      resultMant := (maxMant - 1).U  // 510 for DLFloat
    } else {
      // BFloat16: overflow produces Infinity (e=255, m=0) per IEEE 754
      resultExp := maxExp.U
      resultMant := 0.U
    }
  }.otherwise {
    resultExp := expFinal(expWidth - 1, 0)
    resultMant := mantFinal
  }
  
  // Debug Step 7: Print final result components
  // dprintf(p"[MiniFloat] resultExp=${resultExp} resultMant=${resultMant} final=0x${Hexadecimal(io.result)}\n")
  
  io.result := Cat(finalSign, resultExp, resultMant)
}
