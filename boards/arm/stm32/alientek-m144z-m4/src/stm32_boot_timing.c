/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_boot_timing.c
 *
 * Implementation of the lightweight boot-path tracer declared in
 * boot_timing.h.  See the header for design notes.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING

#include <stdint.h>
#include <syslog.h>

#include "arch/board/board.h"   /* STM32_SYSCLK_FREQUENCY */

#include "boot_timing.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARMv7-M Debug Exception and Monitor Control Register */

#define DEMCR              (*(volatile uint32_t *)0xe000edfcul)
#define DEMCR_TRCENA       (1u << 24)

/* ARMv7-M DWT (Data Watchpoint and Trace) cycle counter */

#define DWT_CTRL           (*(volatile uint32_t *)0xe0001000ul)
#define DWT_CYCCNT         (*(volatile uint32_t *)0xe0001004ul)
#define DWT_CTRL_CYCCNTENA (1u << 0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct boot_mark_s g_boot_marks[BOOT_MARK_MAX];
unsigned int       g_boot_mark_n     = 0;

/* Cycles per microsecond at the configured SYSCLK.  Set by boot_timing_init
 * once the timing trace is armed.  Defaults to 168 to keep the dump output
 * sane even if init() is somehow skipped.
 */

uint32_t g_boot_cyc_per_us = STM32_SYSCLK_FREQUENCY / 1000000ul;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: boot_timing_init
 ****************************************************************************/

void boot_timing_init(void)
{
  /* Enable trace + DWT, then zero and start the cycle counter.  Idempotent
   * if it has already been enabled by another consumer (debugger, ETM,
   * up_perf_init when CONFIG_ARCH_PERF_EVENTS=y, ...).  We always rezero
   * CYCCNT here so the first mark sits at "approximately t=0".
   */

  DEMCR             |= DEMCR_TRCENA;
  DWT_CTRL          |= DWT_CTRL_CYCCNTENA;
  DWT_CYCCNT         = 0;

  g_boot_cyc_per_us  = STM32_SYSCLK_FREQUENCY / 1000000ul;
  g_boot_mark_n      = 0;
}

/****************************************************************************
 * Name: boot_mark
 ****************************************************************************/

void boot_mark(const char *name)
{
  uint32_t cyc = DWT_CYCCNT;

  if (g_boot_mark_n < BOOT_MARK_MAX)
    {
      g_boot_marks[g_boot_mark_n].name = name;
      g_boot_marks[g_boot_mark_n].cyc  = cyc;
      g_boot_mark_n++;
    }
}

/****************************************************************************
 * Name: boot_timing_dump
 ****************************************************************************/

void boot_timing_dump(void)
{
  uint32_t prev_cyc = 0;
  unsigned int i;

  /* Avoid a divide-by-zero if init() somehow never ran. */

  uint32_t cpu = (g_boot_cyc_per_us != 0) ? g_boot_cyc_per_us : 1;

  syslog(LOG_INFO,
         "boot-timing: %u marks, %lu MHz CPU clock\n",
         g_boot_mark_n, (unsigned long)cpu);
  syslog(LOG_INFO,
         "  idx  cyc          delta_us  abs_us   name\n");

  for (i = 0; i < g_boot_mark_n; i++)
    {
      uint32_t cyc  = g_boot_marks[i].cyc;
      uint32_t dcyc = cyc - prev_cyc;     /* unsigned wrap is fine */

      syslog(LOG_INFO,
             "  %2u   %10lu  %8lu  %8lu  %s\n",
             i,
             (unsigned long)cyc,
             (unsigned long)(dcyc / cpu),
             (unsigned long)(cyc  / cpu),
             g_boot_marks[i].name ? g_boot_marks[i].name : "?");

      prev_cyc = cyc;
    }
}

#endif /* CONFIG_ALIENTEK_M144Z_M4_BOOT_TIMING */
