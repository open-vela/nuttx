/****************************************************************************
 * arch/arm/src/bk7258/bk7258_vectors.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

extern void __start(void);
extern void exception_common(void);
extern void exception_direct(void);
extern uint8_t __idle_stack_top[];
extern uint8_t __cpu1_stack_top[];

typedef void (*vector_t)(void);

/* OpenVela currently runs NuttX on the primary AP core. Keep the secondary
 * core in a harmless wait loop until a SMP port is added.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bk7258_cpu1_idle(void)
{
  for (; ; )
    {
      __asm__ volatile ("wfi");
    }
}

/* CPU0 starts CPU1 at the AP image origin, so this table must remain at raw
 * image offset zero. The BK legacy words at 0x100 make it unsuitable as the
 * runtime vector table.
 */

const vector_t g_bk7258_boot_vectors[NR_IRQS]
  __attribute__((used, section(".bk_boot_vectors"), aligned(512))) =
{
  [0]       = (vector_t)__idle_stack_top,
  [1]       = __start,
  [2 ... 14] = exception_common,
  [15 ... 63] = exception_direct,
  [64]      = (vector_t)0x32374b42u,
  [65]      = (vector_t)0x00003633u,
  [66 ... (NR_IRQS - 1)] = exception_direct,
};

/* The BK7258 CP loader also reads a CPU1 vector table at raw AP-image
 * offset 0x200. Keep this table separate from the AP runtime vectors.
 */

const vector_t g_bk7258_cpu1_vectors[NR_IRQS]
  __attribute__((used, section(".bk_cpu1_vectors"), aligned(512))) =
{
  [0]       = (vector_t)__cpu1_stack_top,
  [1]       = bk7258_cpu1_idle,
  [2 ... 14] = exception_common,
  [15 ... 63] = exception_direct,
  [64]      = (vector_t)0x32374b42u,
  [65]      = (vector_t)0x00003633u,
  [66 ... (NR_IRQS - 1)] = exception_direct,
};

/* NuttX exceptions enter through the common ARMv8-M assembly dispatchers.
 * This complete table is copied to AP SRAM before interrupts are enabled.
 */

const vector_t _vectors[NR_IRQS]
  __attribute__((used, section(".ram_vectors"), aligned(512))) =
{
  [0]       = (vector_t)__idle_stack_top,
  [1]       = __start,
  [2 ... 14] = exception_common,
  [15 ... (NR_IRQS - 1)] = exception_direct,
};
