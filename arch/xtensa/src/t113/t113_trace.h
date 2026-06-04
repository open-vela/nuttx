/****************************************************************************
 * arch/xtensa/src/t113/t113_trace.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Lightweight breadcrumb ring in DSP non-cacheable shmem alias.  AP-side
 *   xd 0x47A04400 ...
 * reads are byte-coherent because the DSP writes through the
 * non-cacheable DDR window (0x17A04400) and the AP probes through its
 * own non-cacheable mapping.
 *
 * Each slot is 16 bytes:  { event_id, v1, v2, v3 }.
 * Header at TRACE_BASE :  { magic=0xCAFEBABE, head_idx, _, _ }.
 *
 * Each TRACE_BC call expands to ~8 instructions, no stack, no library
 * calls.  Safe to use from EXCM=1 / ISR / windowed-overflow paths.
 *
 * Callers must guarantee the trace ring is initialized once at boot via
 * t113_trace_init() before the first TRACE_BC.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_T113_T113_TRACE_H
#define __ARCH_XTENSA_SRC_T113_T113_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRACE_BASE       0x17A04400u
#define TRACE_MAGIC_OFF  0x00
#define TRACE_HEAD_OFF   0x04
#define TRACE_SLOTS_OFF  0x10
#define TRACE_SLOT_SIZE  16
#define TRACE_NSLOTS     64
#define TRACE_MAGIC      0xCAFEBABEu

/* Event IDs -- ASCII-mnemonic for easy AP-side decoder readability. */

#define EV_RESET     0x52455354u   /* "RSET"  reset stub completed */
#define EV_START     0x53544152u   /* "STAR"  __start entered */
#define EV_BSS_DONE  0x42535344u   /* "BSSD"  BSS zeroed */
#define EV_NX_PRE    0x4E58504Eu   /* "NXPN"  before nx_start */
#define EV_USEREX    0x55534552u   /* "USER"  user-exception dispatcher entered */
#define EV_KERNEX    0x4B45524Eu   /* "KERN"  kernel-exception dispatcher entered */
#define EV_L3VEC     0x4C334B45u   /* "L3KE"  level-3 vector entered (reserved) */
#define EV_L3DISP    0x4C334445u   /* "L3DE"  level-3 dispatcher body entered */
#define EV_L3RFI     0x4C335249u   /* "L3RI"  level-3 dispatcher about to rfi 3 */
#define EV_DTMR      0x44544D52u   /* "DTMR"  t113_dispatch_timer entered */
#define EV_TMR_ISR   0x544D5249u   /* "TMRI"  t113_timer_isr entered */
#define EV_TMR_RET   0x544D5252u   /* "TMRR"  t113_timer_isr returning */
#define EV_DISIRQ    0x44495351u   /* "DISQ"  up_disable_irq entered */
#define EV_ENIRQ     0x454E5351u   /* "ENSQ"  up_enable_irq entered */
#define EV_IRQINIT   0x49524951u   /* "IRQI"  up_irqinitialize entered */
#define EV_SERINI    0x53455249u   /* "SERI"  xtensa_serialinit entered */
#define EV_SEROUT    0x5345524Fu   /* "SERO"  xtensa_serialinit returning */
#define EV_REG1      0x52454731u   /* "REG1"  before uart_register("/dev/console") */
#define EV_REG2      0x52454732u   /* "REG2"  before uart_register("/dev/ttyS0") */
#define EV_REG3      0x52454733u   /* "REG3"  after both uart_registers */
#define EV_HANG      0x48414E47u   /* "HANG"  _hang reached */
#define EV_SWINT     0x53574954u   /* "SWIT"  xtensa_swint entered */
#define EV_USRX      0x55535258u   /* "USRX"  xtensa_user (non-handled exc) */
#define EV_INTENA    0x49454E41u   /* "IENA"  INTENABLE value after IRQ init */
#define EV_INTDEC    0x49444543u   /* "IDEC"  xtensa_int_decode entry, cpuints[0] */
#define EV_INTCSCAN  0x49435343u   /* "ICSC"  dsp_intc dispatcher PEND0+PEND1 */
#define EV_UISR      0x55495352u   /* "UISR"  uart2_isr entry, IIR value */
#define EV_UATTACH   0x55415454u   /* "UATT"  uart2_attach exit, IER value */

#ifndef __ASSEMBLER__

#ifdef CONFIG_T113_TRACE

static inline void t113_trace_init(void)
{
  volatile uint32_t *hdr = (volatile uint32_t *)TRACE_BASE;
  hdr[0] = TRACE_MAGIC;
  hdr[1] = 0;
}

/* C breadcrumb.  Uses no stack and no AR registers beyond GCC's choice
 * for the volatile-pointer arithmetic.  Safe in any context.
 */

static inline void t113_trace_bc(uint32_t eid, uint32_t v1,
                                       uint32_t v2, uint32_t v3)
{
  volatile uint32_t *hdr  = (volatile uint32_t *)TRACE_BASE;
  uint32_t           i    = hdr[1];
  volatile uint32_t *slot = (volatile uint32_t *)
    (TRACE_BASE + TRACE_SLOTS_OFF +
     (i & (TRACE_NSLOTS - 1)) * TRACE_SLOT_SIZE);

  slot[0] = eid;
  slot[1] = v1;
  slot[2] = v2;
  slot[3] = v3;
  hdr[1]  = i + 1;
}

#else /* !CONFIG_T113_TRACE */

static inline void t113_trace_init(void)
{
}

static inline void t113_trace_bc(uint32_t eid, uint32_t v1,
                                       uint32_t v2, uint32_t v3)
{
}

#endif /* CONFIG_T113_TRACE */

#define TRACE_BC1(eid, v1)         t113_trace_bc((eid), (v1), 0, 0)
#define TRACE_BC2(eid, v1, v2)     t113_trace_bc((eid), (v1), (v2), 0)
#define TRACE_BC3(eid, v1, v2, v3) t113_trace_bc((eid), (v1), (v2), (v3))

#else /* __ASSEMBLER__ */

/* Asm breadcrumb.  Caller passes three scratch ARs that are dead-or-
 * already-saved at the call site.  Writes one slot in ~12 instructions
 * and bumps head_idx.  Form:
 *
 *   TRACE_BC_ASM event_id_imm, val1_reg, val2_reg, scr_a, scr_b, scr_c
 *
 *   sa = TRACE_BASE
 *   sb = head_idx
 *   sc = slot ptr
 */

#ifdef CONFIG_T113_TRACE

.macro TRACE_BC_ASM eid:req, val1:req, val2:req, sa:req, sb:req, sc:req
    movi    \sa, TRACE_BASE
    l32i    \sb, \sa, TRACE_HEAD_OFF
    movi    \sc, (TRACE_NSLOTS - 1)
    and     \sc, \sb, \sc
    slli    \sc, \sc, 4              /* * 16 (slot size) */
    addi    \sc, \sc, TRACE_SLOTS_OFF
    add     \sc, \sa, \sc
    movi    \sa, \eid
    s32i    \sa, \sc, 0
    s32i    \val1, \sc, 4
    s32i    \val2, \sc, 8
    movi    \sa, 0
    s32i    \sa, \sc, 12

    /* Bump head_idx. */

    addi    \sb, \sb, 1
    movi    \sa, TRACE_BASE
    s32i    \sb, \sa, TRACE_HEAD_OFF
.endm

#else /* !CONFIG_T113_TRACE */

.macro TRACE_BC_ASM eid, val1, val2, sa, sb, sc
.endm

#endif /* CONFIG_T113_TRACE */

#endif /* __ASSEMBLER__ */

#endif /* __ARCH_XTENSA_SRC_T113_T113_TRACE_H */
