// ─── cmsisnn_compat.c ──────────────────────────────────────
// Re-implement the helper stubs that appear in old CMSIS–NN objects.
// Compiled with -mcpu=cortex-m7, each wrapper maps to the real SIMD op
// so there is zero runtime overhead.

#include <stdint.h>

__attribute__((naked)) int32_t __sadd16(int32_t a, int32_t b) {
  __asm__("sadd16 r0, r0, r1\n"  // r0 = a + b (pairwise, signed, saturating)
          "bx lr");
}

__attribute__((naked)) int32_t __smlald(int32_t a, int32_t b, int32_t c) {
  // r0 = (a.lo*b.lo + a.hi*b.hi) + c   (signed 16×16, add long dual)
  __asm__("smlald r0, r0, r1, r2\n"
          "bx lr");
}

__attribute__((naked)) int32_t __sxtb16(int32_t a) {
  // sign-extend the two bytes in r0 to 2×16-bit half-words
  __asm__("sxtb16 r0, r0\n"
          "bx lr");
}
