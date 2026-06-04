/****************************************************************************
 * boards/xtensa/t113/t113-evb-dsp/src/t113_intc_probe.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Boot-time DSP_INTC source ID probe.
 *
 * The T113-i UserManual V1.4 does not publish a per-peripheral source ID
 * table for the DSP-side interrupt controller; only the GIC and PLIC
 * tables are listed.  This probe enables all 88 DSP_INTC sources
 * simultaneously, attaches a recording handler to each, and waits up to
 * CONFIG_T113_INTC_PROBE_TIMEOUT seconds for the first source to
 * fire.  The captured source ID is logged via syslog so the operator
 * can wire the right number into the per-peripheral driver.
 *
 * Trigger: send a byte to UART2 within the timeout window.  The first
 * source asserted on the fan-in line wins.
 *
 * The probe runs only when CONFIG_T113_INTC_PROBE=y.  Disable for
 * normal builds.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <unistd.h>

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROBE_LO   0
#define PROBE_HI   87

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern void up_putc(int ch);

static volatile int g_probed_irq = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void probe_puts(const char *s)
{
  while (*s != '\0')
    {
      up_putc((int)(unsigned char)*s++);
    }
}

static void probe_putd(int n)
{
  char     buf[12];
  unsigned u;
  int      i = 0;

  if (n < 0)
    {
      up_putc('-');
      u = (unsigned)(-n);
    }
  else
    {
      u = (unsigned)n;
    }

  if (u == 0)
    {
      up_putc('0');
      return;
    }

  while (u != 0)
    {
      buf[i++] = (char)('0' + (u % 10));
      u /= 10;
    }

  while (i-- > 0)
    {
      up_putc((int)(unsigned char)buf[i]);
    }
}

static int probe_isr(int irq, void *context, void *arg)
{
  int i;

  if (g_probed_irq < 0)
    {
      g_probed_irq = irq;
    }

  /* Disable every probe source so the fan-in line drops and we don't
   * re-enter for any other source that asserts in the same window.
   * Subsequent probe-ISR entries (for sources that latched between the
   * first ack and full disable) just fall through to the disable loop.
   */

  for (i = PROBE_LO; i <= PROBE_HI; i++)
    {
      up_disable_irq(i + T113_IRQ_FIRST);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_intc_probe(void)
{
  int i;
  int waited;

  for (i = PROBE_LO; i <= PROBE_HI; i++)
    {
      irq_attach(i + T113_IRQ_FIRST, probe_isr, NULL);
      up_enable_irq(i + T113_IRQ_FIRST);
    }

  probe_puts("\r\nINTC probe armed [");
  probe_putd(PROBE_LO);
  probe_puts("..");
  probe_putd(PROBE_HI);
  probe_puts("], waiting ");
  probe_putd(CONFIG_T113_INTC_PROBE_TIMEOUT);
  probe_puts(" s for trigger\r\n");

  for (waited = 0; waited < CONFIG_T113_INTC_PROBE_TIMEOUT; waited++)
    {
      if (g_probed_irq >= 0)
        {
          break;
        }

      sleep(1);
    }

  if (g_probed_irq >= 0)
    {
      int source = g_probed_irq - T113_IRQ_FIRST;
      probe_puts("INTC probe CAUGHT source=");
      probe_putd(source);
      probe_puts(" (irq=");
      probe_putd(g_probed_irq);
      probe_puts(")\r\n");
    }
  else
    {
      probe_puts("INTC probe TIMEOUT - no source fired\r\n");
    }

  /* Detach handlers so subsequent normal IRQ wiring is not poisoned. */

  for (i = PROBE_LO; i <= PROBE_HI; i++)
    {
      up_disable_irq(i + T113_IRQ_FIRST);
      irq_attach(i + T113_IRQ_FIRST, NULL, NULL);
    }
}
