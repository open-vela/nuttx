/****************************************************************************
 * arch/xtensa/src/t113/t113_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DDR-resident NuttX entry.  Called from t113_head.S after PS,
 * VECBASE, IntEnable, CPENABLE, MEMCTL are set and SP points to top of DDR.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/init.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <sys/stat.h>

#include "xtensa.h"
#include "t113_trace.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Idle thread stack.  xtensa_initialstate() copies this address into the
 * idle TCB.  Place in .noinit so the C runtime does not zero it before
 * we are running on it.  16-byte aligned per Xtensa stack ABI.
 */

uint32_t g_idlestack[IDLETHREAD_STACKWORDS]
  aligned_data(16) locate_data(".noinit");

void __start(void) __attribute__((noreturn, section(".text.__start")));

/* g_readytorun.head is the first word of the dq_queue_t (declared in
 * <nuttx/sched.h>, pulled in via <sys/stat.h>).  We do not call any queue
 * ops -- just take the address and read the head pointer at offset 0.
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * uart2_trace
 *
 * Direct UART2 LSR-poll putc.  Bypasses NuttX serial driver so we can emit
 * single-char checkpoints from very early init (BSS not yet zeroed, no IRQ,
 * no scheduler).  UART2 was already brought up by the IRAM stub in
 * t113_head.S (LCR/DLL/FCR programmed before 'NUTTX1' was emitted).
 *
 * UART2 = 0x02500800.  THR @ +0x00, LSR @ +0x14, TX-empty bit 5.
 ****************************************************************************/

static void __attribute__((noinline, section(".text.__start")))
uart2_trace(const char *s)
{
  volatile uint32_t *thr = (volatile uint32_t *)0x02500800;
  volatile uint32_t *lsr = (volatile uint32_t *)0x02500814;

  while (*s)
    {
      while (!(*lsr & (1u << 5)))
        ;
      *thr = (uint32_t)((unsigned char)*s++);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __start(void)
{
  uint32_t *p;

  /* Initialize the breadcrumb trace ring in non-cacheable shmem
   * (TRACE_BASE = 0x17A04400).  The ring lives outside .bss, so it is
   * unaffected by the BSS-zero loop below.  Init it BEFORE the first
   * TRACE_BC so subsequent breadcrumbs land in a well-formed ring.
   */

  t113_trace_init();
  TRACE_BC1(EV_START, (uint32_t)(uintptr_t)_sbss);

  /* Trace before BSS-zero: 'S' = start.  The string literal lives in
   * .rodata (DDR) which is already valid before BSS-zero.  uart2_trace
   * itself is in .text.__start (DDR) and uses no statics.
   */

  uart2_trace("S");

  /* Zero BSS. */

  for (p = (uint32_t *)_sbss; p < (uint32_t *)_ebss; p++)
    {
      *p = 0;
    }

  /* Trace after BSS-zero: 'B' = bss done. */

  uart2_trace("B");
  TRACE_BC1(EV_BSS_DONE, (uint32_t)(uintptr_t)_ebss);

  /* Bring up the console UART (baud/FIFO + isconsole) before nx_start so
   * early boot output and NSH have a working /dev/console.  The shared
   * t113_serial driver splits init the upstream way: earlyserialinit
   * configures the console port, serialinit (called from up_initialize
   * inside nx_start) registers the device nodes.  Mirrors esp32_start.c.
   */

  xtensa_earlyserialinit();

  /* Hand control to NuttX. */

  TRACE_BC1(EV_NX_PRE, 0);
  nx_start();

  /* nx_start() never returns. */

  uart2_trace("X");

  for (; ; )
    {
      __asm__ volatile ("waiti 0");
    }
}
