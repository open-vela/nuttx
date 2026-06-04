/****************************************************************************
 * arch/xtensa/src/t113/t113_timerisr.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LX7 CCOUNT / CCOMPARE0 system tick.  Per silicon core-isa.h:
 *   XCHAL_TIMER0_INTERRUPT = 2
 *   CCOMPARE0 raises interrupt 2 when CCOUNT == CCOMPARE0.
 *
 * Note: T113_IRQ_TIMER0 is an Xtensa-internal IRQ (0..3 range), not
 * a DSP_INTC source.  Task 2.C's up_enable_irq() is a no-op for this
 * range, so we unmask the timer interrupt by writing INTENABLE directly
 * (read-modify-write to preserve other unmasked bits).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>
#include <arch/xtensa/core.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCOUNT increments at the DSP core clock.  The board header provides
 * BOARD_DSP_CLOCK_FREQUENCY (drag in via <arch/board/board.h> through
 * the chip headers).
 */

#ifndef BOARD_DSP_CLOCK_FREQUENCY
#  error "board.h must define BOARD_DSP_CLOCK_FREQUENCY"
#endif

#define CCOUNT_PER_TICK \
  (BOARD_DSP_CLOCK_FREQUENCY / 1000000 * CONFIG_USEC_PER_TICK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_timer_isr
 *
 * Description:
 *   CCOMPARE0 interrupt handler.  Re-arm CCOMPARE0 for the next tick and
 *   advance the NuttX system tick counter.
 *
 ****************************************************************************/

static int t113_timer_isr(int irq, void *context, void *arg)
{
  uint32_t now;
  uint32_t next;
  uint32_t ccompare0_pre;

  __asm__ volatile ("rsr.ccount %0" : "=a"(now));
  __asm__ volatile ("rsr.ccompare0 %0" : "=a"(ccompare0_pre));

  next = now + CCOUNT_PER_TICK;
  __asm__ volatile ("wsr.ccompare0 %0" :: "a"(next));

  nxsched_process_timer();

  /* Timer-tick trace is intentionally omitted: each tick would write two
   * ring slots (TMRI/TMRR) at the system-tick rate, crowding out
   * lower-frequency events of interest.
   */

  (void)now;
  (void)ccompare0_pre;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Attach the CCOMPARE0 handler, schedule the first tick, and unmask the
 *   Xtensa-internal timer interrupt via INTENABLE RMW.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t now;
  uint32_t intenable;

  irq_attach(T113_IRQ_TIMER0, t113_timer_isr, NULL);

  /* Schedule first tick. */

  __asm__ volatile ("rsr.ccount %0" : "=a"(now));
  __asm__ volatile ("wsr.ccompare0 %0" :: "a"(now + CCOUNT_PER_TICK));

  /* Unmask CCOMPARE0 interrupt (Xtensa internal, bit
   * XCHAL_TIMER0_INTERRUPT).  RMW so we don't clobber other bits.
   */

  __asm__ volatile ("rsr.intenable %0" : "=a"(intenable));
  intenable |= (1u << XCHAL_TIMER0_INTERRUPT);
  __asm__ volatile ("wsr.intenable %0" :: "a"(intenable));
}
