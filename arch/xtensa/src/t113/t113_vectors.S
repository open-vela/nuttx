/****************************************************************************
 * arch/xtensa/src/t113/t113_vectors.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 HiFi4 exception and window vectors.  Uses the upstream
 * NuttX-canonical exception handlers.
 *
 * Vector base (VECBASE) = 0x401000.  Hardware jumps to VECBASE + fixed
 * offset when each exception fires.  Offsets match vendor LSP and the
 * linker script:
 *
 *   VECBASE + 0x000  WindowOverflow4    (0x401000)
 *   VECBASE + 0x040  WindowUnderflow4   (0x401040)
 *   VECBASE + 0x080  WindowOverflow8    (0x401080)
 *   VECBASE + 0x0C0  WindowUnderflow8   (0x4010C0)
 *   VECBASE + 0x100  WindowOverflow12   (0x401100)
 *   VECBASE + 0x140  WindowUnderflow12  (0x401140)
 *   VECBASE + 0x17C  Level2Interrupt    (0x40117C)
 *   VECBASE + 0x19C  Level3Interrupt    (0x40119C)
 *   VECBASE + 0x1BC  DebugException     (0x4011BC)
 *   VECBASE + 0x1DC  NMIException       (0x4011DC)
 *   VECBASE + 0x1FC  KernelException    (0x4011FC)
 *   VECBASE + 0x21C  UserException      (0x40121C)
 *   VECBASE + 0x23C  DoubleException    (0x40123C)
 *
 * Each vector slot is at most 32 bytes.  The body simply spills a0 to the
 * appropriate EXCSAVE_n register and short-jumps (`j`, +/-128 KB range)
 * to the upstream NuttX handler in IRAM_HAND (0x401800..0x420000):
 *
 *   UserExceptionVector  -> wsr.excsave1 a0; j _xtensa_user_handler
 *   KernelExceptionVector-> wsr.excsave1 a0; j _xtensa_kernel_handler
 *   Level3InterruptVector-> wsr.excsave3 a0; j _xtensa_level3_handler
 *
 * Window vectors are provided by NuttX common at
 * arch/xtensa/src/common/xtensa_window_vector.S which emits canonical
 * spill/fill bodies into .window_vectors.text.  The linker script anchors
 * that section at VECBASE so HW dispatch lands directly on the canonical
 * handlers.  We do NOT define our own window vectors here.
 *
 * Level2 / Debug / NMI / Double exception vectors hang -- nothing on this
 * core targets them in this build.  (DSP_INTC fires at level 1, timer at
 * level 3.)
 ****************************************************************************/

/* -----------------------------------------------------------------------
 * Window vectors (at VECBASE = 0x00401000)
 *
 * Provided by NuttX common code at arch/xtensa/src/common/
 * xtensa_window_vector.S.  See file header above.
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * Level2InterruptVector / DebugExceptionVector / NMIExceptionVector --
 * unused on this core, hang for safety.
 * ----------------------------------------------------------------------- */

    .section .Level2InterruptVector.text, "ax"
    .global _Level2InterruptVector
_Level2InterruptVector:
    j _hang

    .section .DebugExceptionVector.text, "ax"
    .global _DebugExceptionVector
_DebugExceptionVector:
    j _hang

    .section .NMIExceptionVector.text, "ax"
    .global _NMIExceptionVector
_NMIExceptionVector:
    j _hang

/* -----------------------------------------------------------------------
 * Level3InterruptVector: VECBASE + 0x19C = 0x40119C
 *
 * Spill a0 to EXCSAVE_3 (level-3 scratch register) and short-jump to the
 * upstream level-3 handler.  CCOMPARE0 timer tick is the only level-3
 * source enabled in this build (XCHAL_INT2_LEVEL = 3,
 * XCHAL_TIMER0_INTERRUPT = 2).  The upstream handler will save full
 * context, call xtensa_int_decode(...) which we provide chip-side, then
 * restore and 'rfi 3'.
 * ----------------------------------------------------------------------- */

    .section .Level3InterruptVector.text, "ax"
    .global _Level3InterruptVector
_Level3InterruptVector:
    wsr.excsave3    a0
    j _xtensa_level3_handler

/* -----------------------------------------------------------------------
 * KernelExceptionVector: VECBASE + 0x1FC = 0x4011FC
 *
 * Synchronous exception taken in kernel mode (PS.UM = 0).  The upstream
 * `_xtensa_kernel_handler` panics for any kernel-mode exception (we never
 * intentionally trigger one) but we still wire it up for diagnostics.
 * ----------------------------------------------------------------------- */

    .section .KernelExceptionVector.text, "ax"
    .global _KernelExceptionVector
_KernelExceptionVector:
    wsr.excsave1    a0
    j _xtensa_kernel_handler

/* -----------------------------------------------------------------------
 * UserExceptionVector: VECBASE + 0x21C = 0x40121C
 *
 * Generic user-mode exception path.  The upstream `_xtensa_user_handler`
 * dispatches to the correct sub-path based on EXCCAUSE:
 *   EXCCAUSE = 1  (SYSCALL)        -> _xtensa_syscall_handler
 *   EXCCAUSE = 4  (LEVEL1INT)      -> _xtensa_level1_handler
 *   EXCCAUSE = 5  (ALLOCA)         -> _xtensa_alloca_handler
 *   anything else                  -> xtensa_user(exccause, regs) (chip)
 * ----------------------------------------------------------------------- */

    .section .UserExceptionVector.text, "ax"
    .global _UserExceptionVector
_UserExceptionVector:
    wsr.excsave1    a0
    j _xtensa_user_handler

/* -----------------------------------------------------------------------
 * DoubleExceptionVector: VECBASE + 0x23C = 0x40123C
 *
 * A double exception means the chip took an exception while EXCM=1; no
 * recovery.  Hang.
 * ----------------------------------------------------------------------- */

    .section .DoubleExceptionVector.text, "ax"
    .global _DoubleExceptionVector
_DoubleExceptionVector:
    j _hang

/* -----------------------------------------------------------------------
 * IRAM stub region: minimal helper labels (_hang) referenced by the
 * level-2 / debug / NMI / double vectors above.
 * ----------------------------------------------------------------------- */

    .section .iram_stub.text, "ax"
    .literal_position
    .align 4

    .global _hang
    .align 4
_hang:
.L_hang_loop:
    j .L_hang_loop
