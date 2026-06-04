/****************************************************************************
 * arch/xtensa/src/t113/t113_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DSP_INTC subordinate-INTC routing for the T113-S3 HiFi4 DSP.
 *
 * The DSP-side interrupt controller occupies 0x01700800..0x01700BFF
 * (1 KB) per the T113-i UserManual V1.4 memory map.  It fans up to 88
 * SoC peripheral interrupt sources into a single Xtensa interrupt line
 * (INT 20, level 1).  When any unmasked source asserts, the controller
 * latches a bit in PEND[0..2] and pulls INT 20; the ISR walks PEND to
 * recover the source ID and dispatches into the NuttX kernel.
 *
 * NuttX IRQ map (see arch/xtensa/include/t113/irq.h):
 *
 *   IRQ 0..3   : Xtensa internal (TIMER0/TIMER1/SYSCALL/SWINT) -- managed
 *                by the xtensa core, not the DSP_INTC.  up_enable_irq()
 *                / up_disable_irq() are no-ops for these; the timer
 *                driver toggles INTENABLE directly.
 *   IRQ 4..91  : DSP_INTC sources 0..87 (offset T113_IRQ_FIRST = 4)
 *
 * The datasheet does not publish a register-offset table for this block;
 * the offsets are derived from baremetal firmware that has run on this
 * silicon.  The per-source ID table is described in irq.h.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>

#include <arch/chip/core-isa.h>

#include "chip_memory.h"
#include "t113_trace.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSP_INTC_NSOURCES   88
#define DSP_INTC_NREGS      3
#define DSP_INTC_FANIN_BIT  20    /* Xtensa INT bit driven by DSP_INTC OUT */

#define DSP_INTC_VECTOR     ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x00))
#define DSP_INTC_BASE_ADDR  ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x04))
#define DSP_INTC_NMI_CTRL   ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x0C))
#define DSP_INTC_PEND(n)    ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x10 + ((n) * 4)))
#define DSP_INTC_INT_EN(n)  ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x40 + ((n) * 4)))
#define DSP_INTC_MASK(n)    ((volatile uint32_t *)(T113_DSP_INTC_BASE + 0x50 + ((n) * 4)))

/* R_INTC fan-in is live: UART2 is R_INTC source 3, which raises Xtensa
 * INT 20.  t113_dispatch_dsp_intc() walks PEND to reach the UART2 ISR.
 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_swint_wrap(int irq, void *context, void *arg);
static uint32_t *t113_dispatch_dsp_intc(uint32_t *regs);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  unsigned int i;
  uint32_t intenable;

  TRACE_BC1(EV_IRQINIT, 0);

  /* Attach the SYSCALL handler so the kernel's first context switch goes
   * through xtensa_swint() instead of falling through to xtensa_user_panic.
   * Use a chip-side wrapper that drops a trace breadcrumb so we can
   * confirm the syscall path is exercised during early boot.
   */

  irq_attach(XTENSA_IRQ_SYSCALL, t113_swint_wrap, NULL);

  /* Disable every DSP_INTC source, mask every source, and clear every
   * pending bit.  PEND/MASK are write-1-to-clear / write-1-to-mask;
   * 0xffffffff is safe across the 88-source range because the upper
   * unimplemented bits are read-as-zero / write-ignored.
   */

  /* INT bit 20 is the fan-in line for the R_INTC sources.  enable=0,
   * mask=0 (NOT 0xffffffff), pending=0xffffffff (write-1-clear).  mask
   * is left at 0 permanently -- the controller's notion of "masked" is
   * the inverse of NuttX's; gating is performed purely through the
   * enable bitmap.  An all-ones mask would suppress every source even
   * after enable.
   */

  for (i = 0; i < DSP_INTC_NREGS; i++)
    {
      *DSP_INTC_INT_EN(i) = 0x00000000;
      *DSP_INTC_MASK(i)   = 0x00000000;
      *DSP_INTC_PEND(i)   = 0xffffffff;
    }

  *DSP_INTC_NMI_CTRL = 0x00000000;

  /* Unmask the Xtensa INT bit the DSP_INTC fan-in line drives (bit 20,
   * level 1).  Per-source enable is performed by up_enable_irq() from
   * individual peripheral drivers.  Clearing pending bits above ensures
   * the fan-in line deasserts cleanly once the correct source is acked.
   * RMW to preserve the timer bit set by up_timer_initialize.
   */

  /* The MSGBOX external interrupt (Xtensa INT bit 3) is NOT unmasked here:
   * its handler is attached later (board late init), and unmasking before
   * the handler exists could dispatch to an unregistered vector.  The
   * msgbox driver unmasks INT 3 itself once it has attached.
   */

  __asm__ volatile ("rsr.intenable %0" : "=a"(intenable));
  intenable |= (1u << DSP_INTC_FANIN_BIT);
  __asm__ volatile ("wsr.intenable %0" :: "a"(intenable));

  TRACE_BC1(EV_INTENA, intenable);
}

/****************************************************************************
 * Name: t113_swint_wrap
 *
 * Description:
 *   Trace-breadcrumb wrapper around xtensa_swint().  Lets the AP-side
 *   tracer see exactly when SYSCALL exceptions land and which command
 *   (SYS_save/restore/switch_context) is being processed.
 *
 ****************************************************************************/

static int t113_swint_wrap(int irq, void *context, void *arg)
{
  uint32_t *regs = (uint32_t *)context;

  TRACE_BC2(EV_SWINT, regs ? regs[REG_A2] : 0xffffffff,
            regs ? regs[REG_PC] : 0);

  return xtensa_swint(irq, context, arg);
}

/****************************************************************************
 * Name: t113_dispatch_dsp_intc
 *
 * Description:
 *   Walk DSP_INTC PEND[0..2] and dispatch every set-and-enabled source
 *   bit as a NuttX peripheral IRQ.  Acknowledges each handled bit by
 *   write-1-to-clear so the fan-in line on Xtensa INT 20 deasserts.
 *
 ****************************************************************************/

static uint32_t *t113_dispatch_dsp_intc(uint32_t *regs)
{
  unsigned int reg;

  for (reg = 0; reg < DSP_INTC_NREGS; reg++)
    {
      uint32_t pend = *DSP_INTC_PEND(reg) & *DSP_INTC_INT_EN(reg);

      while (pend != 0)
        {
          int      sub    = __builtin_ctz(pend);
          uint32_t bitm   = 1u << sub;
          uint32_t source = (uint32_t)(reg * 32 + sub);

          if (source < DSP_INTC_NSOURCES)
            {
              regs = xtensa_irq_dispatch((int)source + T113_IRQ_FIRST,
                                         regs);
            }

          /* Write-1-to-clear the pending bit.  Peripherals that
           * self-clear on read of their status register will already
           * have done so inside the dispatched ISR; this write is a
           * no-op in that case and an explicit ack otherwise.
           */

          *DSP_INTC_PEND(reg) = bitm;
          pend &= ~bitm;
        }
    }

  return regs;
}

/****************************************************************************
 * Name: xtensa_int_decode
 *
 * Description:
 *   Called by the upstream Xtensa level-1/2/3 handlers (in
 *   arch/xtensa/src/common/xtensa_int_handlers.S) to decode and dispatch
 *   pending CPU interrupts at a given level.
 *
 *   The dispatcher passes a pointer to a four-word buffer holding the
 *   masked-and-pending bits per interrupt-controller register; on this
 *   silicon XCHAL_NUM_INTERRUPTS = 32 so only word 0 is valid.
 *
 *   Bit-by-bit:
 *     - XCHAL_TIMER0_INTERRUPT (level 3)         -> XTENSA_IRQ_TIMER0
 *     - DSP_INTC_FANIN_BIT     (level 1, bit 20) -> walk DSP_INTC PEND
 *     - any other level-1 internal bit           -> sentinel ack
 *
 ****************************************************************************/

uint32_t *xtensa_int_decode(uint32_t *cpuints, uint32_t *regs)
{
  uint32_t pending = cpuints[0];
  int      bit;

  /* Trace the bitmap only when something other than CCOMPARE0 (the system
   * tick) is pending; otherwise we drown the ring in identical events at
   * 100 Hz and the DSP_INTC fan-in (bit 20) gets wrapped out of sight.
   */

  if (pending != (1u << XCHAL_TIMER0_INTERRUPT))
    {
      TRACE_BC1(EV_INTDEC, pending);
    }

  while (pending != 0)
    {
      bit = __builtin_ctz(pending);

      if (bit == XCHAL_TIMER0_INTERRUPT)
        {
          regs = xtensa_irq_dispatch(XTENSA_IRQ_TIMER0, regs);
        }
      else if (bit == DSP_INTC_FANIN_BIT)
        {
          regs = t113_dispatch_dsp_intc(regs);
        }
      else if (bit == T113_MSGBOX_INT)
        {
          /* MSGBOX is hard-wired to Xtensa external interrupt 3
           * (EXTERN_LEVEL, BInterrupt[1]).  Dispatch to the handler the
           * msgbox driver attached at XTENSA_IRQ_MSGBOX.
           */

          regs = xtensa_irq_dispatch(XTENSA_IRQ_MSGBOX, regs);
        }
      else
        {
          /* Other Xtensa-internal level-1 bits (TIMER1 / DEBUG /
           * SOFTWARE) and any spurious level-1 fan-in bits we have
           * not wired.  Drop with a sentinel ack -- handler-less
           * dispatch into DSP_INTC would walk PEND for nothing.
           */

          regs = xtensa_irq_dispatch(NR_IRQS, regs);
        }

      pending &= ~(1u << bit);
    }

  cpuints[0] = pending;
  return regs;
}

/****************************************************************************
 * Name: xtensa_user
 *
 * Description:
 *   Called by the upstream user-exception handler for any EXCCAUSE that is
 *   not handled by the dedicated SYSCALL / level-1 IRQ / ALLOCA paths.
 *   For now we delegate straight to xtensa_user_panic(), which dumps the
 *   register frame and calls _assert -- producing a comprehensible report
 *   on UART2.
 *
 ****************************************************************************/

uint32_t *xtensa_user(int exccause, uint32_t *regs)
{
  xtensa_user_panic(exccause, regs);

  while (1)
    {
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable an IRQ.  For Xtensa internal IRQs (TIMER0/TIMER1/SYSCALL/
 *   SWINT, IRQ 0..3) this is a no-op -- INTENABLE is owned by the
 *   xtensa-common path and the timer driver.  For DSP_INTC peripheral
 *   IRQs (IRQ 4..91) clear the per-source enable bit and assert the
 *   per-source mask.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uint32_t source;
  uint32_t reg;
  uint32_t bit;

  TRACE_BC2(EV_DISIRQ, (uint32_t)irq, 0);

  if (irq < T113_IRQ_FIRST || irq > T113_IRQ_DSP_INTC_LAST)
    {
      return;
    }

  source = (uint32_t)(irq - T113_IRQ_FIRST);
  reg    = source >> 5;
  bit    = 1u << (source & 0x1f);

  *DSP_INTC_INT_EN(reg) &= ~bit;
  *DSP_INTC_MASK(reg)   |=  bit;
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable an IRQ.  Mirrors up_disable_irq -- no-op for Xtensa internal
 *   IRQs, drop mask and set enable for DSP_INTC peripheral IRQs.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uint32_t source;
  uint32_t reg;
  uint32_t bit;

  TRACE_BC2(EV_ENIRQ, (uint32_t)irq, 0);

  if (irq < T113_IRQ_FIRST || irq > T113_IRQ_DSP_INTC_LAST)
    {
      return;
    }

  source = (uint32_t)(irq - T113_IRQ_FIRST);
  reg    = source >> 5;
  bit    = 1u << (source & 0x1f);

  *DSP_INTC_MASK(reg)   &= ~bit;
  *DSP_INTC_INT_EN(reg) |=  bit;
}
