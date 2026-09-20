/* SPDX-License-Identifier: Apache-2.0 */

/* P4 TRM 2.7.1: HWLP state must follow the interrupted task. The common
 * entry has already saved all integer scratch registers. Keep this macro
 * read-only: up_saveusercontext also uses it outside interrupt entry.
 */

#if !defined(CONFIG_ESP32P4_HWLP_CONTEXT)
#  error "P4 extension context requires HWLP support"
#endif
#ifdef CONFIG_ESP32P4_PIE_CONTEXT
#  ifdef CONFIG_ESP32P4_FPU_DIAGNOSTIC
#    if CONFIG_ARCH_RISCV_INTXCPT_EXTREGS != 70 || REG_INT_CTX_NDX != 32
#      error "FPU/PIE requires 412-byte integer + 132-byte FPU frame"
#    endif
#  elif CONFIG_ARCH_RISCV_INTXCPT_EXTREGS != 67 || REG_INT_CTX_NDX != 32
#    error "PIE requires aligned 400-byte flat RV32 exception frame"
#  endif
#elif CONFIG_ARCH_RISCV_INTXCPT_EXTREGS != 7
#  error "HWLP-only context requires seven extension words"
#endif

#define P4_HWLP_BASE ((REG_INT_CTX_NDX + 1) * INT_REG_SIZE)

  .macro save_extctx base
  csrr t0, 0x7f1
  REGSTORE t0, (P4_HWLP_BASE + 0)(\base)
  andi t1, t0, 3
  beqz t1, .Lhwlp_saved\@
  csrr t0, 0x7c6
  REGSTORE t0, (P4_HWLP_BASE + 4)(\base)
  csrr t0, 0x7c7
  REGSTORE t0, (P4_HWLP_BASE + 8)(\base)
  csrr t0, 0x7c8
  REGSTORE t0, (P4_HWLP_BASE + 12)(\base)
  csrr t0, 0x7c9
  REGSTORE t0, (P4_HWLP_BASE + 16)(\base)
  csrr t0, 0x7ca
  REGSTORE t0, (P4_HWLP_BASE + 20)(\base)
  csrr t0, 0x7cb
  REGSTORE t0, (P4_HWLP_BASE + 24)(\base)
.Lhwlp_saved\@:
#ifdef CONFIG_ESP32P4_PIE_CONTEXT
  csrr t0, 0x7f2
  REGSTORE t0, 160(\base)
  andi t1, t0, 3
  beqz t1, .Lpie_saved\@
  addi a0, \base, 176
  la t2, esp32p4_pie_save_context
  jalr t2, t2, 0
  addi a0, a0, -176
  csrw 0x7f2, t0
.Lpie_saved\@:
#endif
  .endm

  .macro load_extctx base
  REGLOAD t1, (P4_HWLP_BASE + 0)(\base)
  andi t0, t1, 3
  beqz t0, .Lhwlp_restore_state\@
  csrwi 0x7f1, 1
  csrwi 0x7c8, 0
  csrwi 0x7cb, 0
  REGLOAD t0, (P4_HWLP_BASE + 4)(\base)
  csrw 0x7c6, t0
  REGLOAD t0, (P4_HWLP_BASE + 8)(\base)
  csrw 0x7c7, t0
  REGLOAD t0, (P4_HWLP_BASE + 16)(\base)
  csrw 0x7c9, t0
  REGLOAD t0, (P4_HWLP_BASE + 20)(\base)
  csrw 0x7ca, t0
  REGLOAD t0, (P4_HWLP_BASE + 12)(\base)
  csrw 0x7c8, t0
  REGLOAD t0, (P4_HWLP_BASE + 24)(\base)
  csrw 0x7cb, t0
.Lhwlp_restore_state\@:
  csrw 0x7f1, t1
#ifdef CONFIG_ESP32P4_PIE_CONTEXT
  REGLOAD t0, 160(\base)
  andi t1, t0, 3
  beqz t1, .Lpie_restore_state\@
  csrwi 0x7f2, 1
  addi a0, \base, 176
  la t2, esp32p4_pie_load_context
  jalr t2, t2, 0
  addi a0, a0, -176
.Lpie_restore_state\@:
  csrw 0x7f2, t0
#endif
  .endm
