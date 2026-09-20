/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adapted for NuttX from ESP-IDF c712a0dde385d659a1470a136251980d31a70bc1
 * FreeRTOS-Kernel/portable/riscv/portasm.S. Changes: eager frame storage,
 * fixed aligned224-byte payload, preserve a0, t2 return-link for kernel
 * macros, and writable CFG preservation. No FreeRTOS dependency.
 *
 * Contract: CFG saturation-status tracking is disabled (ESP-NN CFG=2).
 * Read-clear saturation/exception flags and PERF are NOT virtualized.
 * Only task-level PIE is supported, never PIE in an ISR.
 */
  .text
  .align 2
  .global esp32p4_pie_save_regs
  .type esp32p4_pie_save_regs, @function
esp32p4_pie_save_regs:
  mv t2, ra
  .global esp32p4_pie_save_context
esp32p4_pie_save_context:
  mv t5, a0
  esp.vst.128.ip q0, t5, 16
  esp.vst.128.ip q1, t5, 16
  esp.vst.128.ip q2, t5, 16
  esp.vst.128.ip q3, t5, 16
  esp.vst.128.ip q4, t5, 16
  esp.vst.128.ip q5, t5, 16
  esp.vst.128.ip q6, t5, 16
  esp.vst.128.ip q7, t5, 16
  esp.st.qacc.l.l.128.ip t5, 16
  esp.st.qacc.l.h.128.ip t5, 16
  esp.st.qacc.h.l.128.ip t5, 16
  esp.st.qacc.h.h.128.ip t5, 16
  esp.st.ua.state.ip t5, 16
  esp.st.u.xacc.ip t5, 0
  esp.movx.r.sar.bytes t4
  slli t3, t4, 4
  esp.movx.r.fft.bit.width t4
  or t3, t3, t4
  sb t3, 5(t5)
  esp.movx.r.sar t4
  sb t4, 6(t5)
  esp.movx.r.cfg t4
  sw t4, 8(t5)
  jr t2
  .size esp32p4_pie_save_regs, . - esp32p4_pie_save_regs

  .align 2
  .global esp32p4_pie_load_regs
  .type esp32p4_pie_load_regs, @function
esp32p4_pie_load_regs:
  mv t2, ra
  .global esp32p4_pie_load_context
esp32p4_pie_load_context:
  mv t5, a0
  esp.vld.128.ip q0, t5, 16
  esp.vld.128.ip q1, t5, 16
  esp.vld.128.ip q2, t5, 16
  esp.vld.128.ip q3, t5, 16
  esp.vld.128.ip q4, t5, 16
  esp.vld.128.ip q5, t5, 16
  esp.vld.128.ip q6, t5, 16
  esp.vld.128.ip q7, t5, 16
  esp.ld.qacc.l.l.128.ip t5, 16
  esp.ld.qacc.l.h.128.ip t5, 16
  esp.ld.qacc.h.l.128.ip t5, 16
  esp.ld.qacc.h.h.128.ip t5, 16
  esp.ld.ua.state.ip t5, 16
  esp.ld.xacc.ip t5, 0
  lbu t4, 5(t5)
  esp.movx.w.fft.bit.width t4
  srli t4, t4, 4
  esp.movx.w.sar.bytes t4
  lbu t4, 6(t5)
  esp.movx.w.sar t4
  lw t4, 8(t5)
  esp.movx.w.cfg t4
  jr t2
  .size esp32p4_pie_load_regs, . - esp32p4_pie_load_regs

  .align 2
  .global esp32p4_pie_unaligned_seed
  .type esp32p4_pie_unaligned_seed, @function
esp32p4_pie_unaligned_seed:
  mv t5, a0
  esp.vld.128.ip q0, t5, 16
  ret
  .size esp32p4_pie_unaligned_seed, . - esp32p4_pie_unaligned_seed
