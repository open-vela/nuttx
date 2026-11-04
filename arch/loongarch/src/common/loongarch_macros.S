.file "loongarch_macros.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/csr.h>
#include <arch/irq.h>

#include <sys/types.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Name: save_ctx
 *
 * Parameter:
 *   base - Pointer to where the save is performed
 *
 * Description:
 *   Save the common context registers.
 *
 ****************************************************************************/

.macro save_ctx base
  st.d    $ra,  \base, (REGSIZE * REG_R1_NDX)
  st.d    $sp,  \base, (REGSIZE * REG_R3_NDX)
  st.d    $r4,  \base, (REGSIZE * REG_R4_NDX)
  st.d    $r5,  \base, (REGSIZE * REG_R5_NDX)
  st.d    $r6,  \base, (REGSIZE * REG_R6_NDX)
  st.d    $r7,  \base, (REGSIZE * REG_R7_NDX)
  st.d    $r8,  \base, (REGSIZE * REG_R8_NDX)
  st.d    $r9,  \base, (REGSIZE * REG_R9_NDX)
  st.d    $r10, \base, (REGSIZE * REG_R10_NDX)
  st.d    $r11, \base, (REGSIZE * REG_R11_NDX)
  st.d    $r12, \base, (REGSIZE * REG_R12_NDX)
  st.d    $r13, \base, (REGSIZE * REG_R13_NDX)
  st.d    $r14, \base, (REGSIZE * REG_R14_NDX)
  st.d    $r15, \base, (REGSIZE * REG_R15_NDX)
  st.d    $r16, \base, (REGSIZE * REG_R16_NDX)
  st.d    $r17, \base, (REGSIZE * REG_R17_NDX)
  st.d    $r18, \base, (REGSIZE * REG_R18_NDX)
  st.d    $r19, \base, (REGSIZE * REG_R19_NDX)
  st.d    $r20, \base, (REGSIZE * REG_R20_NDX)
  st.d    $r21, \base, (REGSIZE * REG_R21_NDX)
  st.d    $r22, \base, (REGSIZE * REG_R22_NDX)
  st.d    $r23, \base, (REGSIZE * REG_R23_NDX)
  st.d    $r24, \base, (REGSIZE * REG_R24_NDX)
  st.d    $r25, \base, (REGSIZE * REG_R25_NDX)
  st.d    $r26, \base, (REGSIZE * REG_R26_NDX)
  st.d    $r27, \base, (REGSIZE * REG_R27_NDX)
  st.d    $r28, \base, (REGSIZE * REG_R28_NDX)
  st.d    $r29, \base, (REGSIZE * REG_R29_NDX)
  st.d    $r30, \base, (REGSIZE * REG_R30_NDX)
  st.d    $r31, \base, (REGSIZE * REG_R31_NDX)
.endm

/****************************************************************************
 * Name: load_ctx
 *
 * Parameter:
 *   base - Pointer to where the load is performed
 *
 * Description:
 *   Load the common context registers.
 *
 ****************************************************************************/

.macro load_ctx base
  ld.d    $ra,  \base, (REGSIZE * REG_R1_NDX)
  ld.d    $r4,  \base, (REGSIZE * REG_R4_NDX)
  ld.d    $r5,  \base, (REGSIZE * REG_R5_NDX)
  ld.d    $r6,  \base, (REGSIZE * REG_R6_NDX)
  ld.d    $r7,  \base, (REGSIZE * REG_R7_NDX)
  ld.d    $r8,  \base, (REGSIZE * REG_R8_NDX)
  ld.d    $r9,  \base, (REGSIZE * REG_R9_NDX)
  ld.d    $r10, \base, (REGSIZE * REG_R10_NDX)
  ld.d    $r11, \base, (REGSIZE * REG_R11_NDX)
  ld.d    $r12, \base, (REGSIZE * REG_R12_NDX)
  ld.d    $r13, \base, (REGSIZE * REG_R13_NDX)
  ld.d    $r14, \base, (REGSIZE * REG_R14_NDX)
  ld.d    $r15, \base, (REGSIZE * REG_R15_NDX)
  ld.d    $r16, \base, (REGSIZE * REG_R16_NDX)
  ld.d    $r17, \base, (REGSIZE * REG_R17_NDX)
  ld.d    $r18, \base, (REGSIZE * REG_R18_NDX)
  ld.d    $r19, \base, (REGSIZE * REG_R19_NDX)
  ld.d    $r20, \base, (REGSIZE * REG_R20_NDX)
  ld.d    $r21, \base, (REGSIZE * REG_R21_NDX)
  ld.d    $r22, \base, (REGSIZE * REG_R22_NDX)
  ld.d    $r23, \base, (REGSIZE * REG_R23_NDX)
  ld.d    $r24, \base, (REGSIZE * REG_R24_NDX)
  ld.d    $r25, \base, (REGSIZE * REG_R25_NDX)
  ld.d    $r26, \base, (REGSIZE * REG_R26_NDX)
  ld.d    $r27, \base, (REGSIZE * REG_R27_NDX)
  ld.d    $r28, \base, (REGSIZE * REG_R28_NDX)
  ld.d    $r29, \base, (REGSIZE * REG_R29_NDX)
  ld.d    $r30, \base, (REGSIZE * REG_R30_NDX)
  ld.d    $r31, \base, (REGSIZE * REG_R31_NDX)
.endm
