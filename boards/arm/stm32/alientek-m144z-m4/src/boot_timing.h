/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/boot_timing.h
 *
 * Lightweight boot-path instrumentation for the Alientek M144Z-M4.  Records
 * cycle-counter snapshots at well-known boot landmarks (board init, bring-
 * up sub-systems, ...) into a small RAM array and dumps them through
 * syslog at the end of stm32_bringup().
 *
 * Time source: ARMv7-M DWT cycle counter (CYCCNT).  CYCCNT is enabled
 * inside boot_timing_init() and is fed by the CPU clock, so 1 cycle = 1 /
 * STM32_SYSCLK_FREQUENCY seconds (~5.95 ns at 168 MHz).  The 32-bit
 * counter wraps every ~25.5 s at 168 MHz, well above any sane boot time.
 *
 * Disabled by default; gate with CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING.
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_BOOT_TIMING_H
#define __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_BOOT_TIMING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING

#define BOOT_MARK_MAX 32

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct boot_mark_s
{
  const char *name;
  uint32_t    cyc;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct boot_mark_s g_boot_marks[BOOT_MARK_MAX];
extern unsigned int       g_boot_mark_n;
extern uint32_t           g_boot_cyc_per_us;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Enable DWT CYCCNT, zero the cycle counter and the mark table. */

void boot_timing_init(void);

/* Append a {name, cyc} entry to the mark table.  Silently drops entries
 * past BOOT_MARK_MAX.  Safe to call from any context where DWT is alive
 * (i.e. anywhere after boot_timing_init()).
 */

void boot_mark(const char *name);

/* Pretty-print all collected marks through syslog().  Format:
 *   boot-timing: <N> marks, <fcpu> MHz
 *     idx  cyc        delta_us  abs_us   name
 *     ...
 */

void boot_timing_dump(void);

#else /* !CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING */

/* Compile-time no-ops so call sites stay clean even when disabled. */

#  define boot_timing_init() do { } while (0)
#  define boot_mark(name)    do { (void)(name); } while (0)
#  define boot_timing_dump() do { } while (0)

#endif /* CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING */

#endif /* __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_BOOT_TIMING_H */
