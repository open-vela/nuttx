/* SPDX-License-Identifier: Apache-2.0 */
/* Short instruction-only diagnostic. Caller must disable interrupts, ensure
 * PIE was off, and supply two valid sixteen-byte buffers. No HWLP or calls.
 */
  .text
  .align 2
  .global esp32p4_pie_dot16
  .type esp32p4_pie_dot16, @function
esp32p4_pie_dot16:
  csrwi 0x7f2, 1
  li t4, 2
  esp.movx.w.cfg t4
  mv t5, a0
  mv t6, a1
  esp.zero.xacc
  esp.vld.128.ip q0, t5, 16
  esp.vld.128.ip q1, t6, 16
  esp.vmulas.s8.xacc q0, q1
  esp.movx.r.xacc.l t5
  mv a0, t5
  csrwi 0x7f2, 0
  ret
  .size esp32p4_pie_dot16, . - esp32p4_pie_dot16
