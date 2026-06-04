/****************************************************************************
 * arch/arm/src/t113/t113_systemreset.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <sys/boardctl.h>

#include "arm_internal.h"
#include "hardware/t113_rtc.h"
#include "hardware/t113_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Unified boot mode protocol using RTC GP_DATA_REG[0] (0x07090100).
 * VDD_RTC power domain - retains value across WDT warm resets.
 *
 * Format: [31:16] = 0x5AA5 (magic), [15:0] = mode
 *   0x5AA50000 = software-initiated USER reboot (NSH `reboot`)
 *   0x5AA50001 = enter FEL mode  (reboot 99 / JLink)
 *   0x00000000 = cold POR, silent WDT, ASSERT/PANIC, or any other
 *                non-USER reset path - treated as "abnormal" by the
 *                reboot-loop guard.
 *
 * The lower 16 bits encode the mode; only USER (0) and FEL (1) are
 * given a magic.  Every other softreset subreason (ASSERT, PANIC,
 * BOOTLOADER, RECOVERY, ...) leaves GP_DATA[0] = 0, which
 * t113_fel_rescue() interprets as abnormal and counts.
 *
 * boot0 only acts on the FEL magic; otherwise it leaves the register
 * intact so NuttX board_reset_cause() can read and clear it.
 */

#define T113_BOOT_MAGIC         0x5AA50000
#define T113_BOOT_MAGIC_MASK    0xFFFF0000
#define T113_BOOT_MODE_USER     0x0000
#define T113_BOOT_MODE_FEL      0x0001

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   Get the cause of the last reset by reading the boot mode marker
 *   in RTC GP_DATA_REG[0].  boot0 only clears this register when
 *   entering FEL mode; for normal boot the value is left intact.
 *   We always clear after reading to prepare for the next cycle.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
int board_reset_cause(FAR struct boardioc_reset_cause_s *cause)
{
  uint32_t marker;

  marker = getreg32(T113_RTC_GP_DATA(0));

  /* Clear marker for next boot cycle */

  putreg32(0, T113_RTC_GP_DATA(0));

  if ((marker & T113_BOOT_MAGIC_MASK) == T113_BOOT_MAGIC)
    {
      cause->cause = BOARDIOC_RESETCAUSE_CPU_SOFT;
    }
  else
    {
      cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
    }

  cause->flag = 0;

  sinfo("reset cause: %d (marker=0x%08" PRIx32 ")\n",
        cause->cause, marker);
  return 0;
}
#endif

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Encode the reset reason as a marker in RTC
 *   GP_DATA_REG[0], disable the watchdog (T113 User Manual 3.6.6.11),
 *   then trigger a watchdog soft reset.
 *
 *   status == 99
 *     Enter FEL download mode.  boot0 detects MODE_FEL on the next
 *     boot and jumps to BROM FEL.  Also clears the reboot-loop
 *     counter - entering FEL is itself a recovery action.
 *
 *   status == BOARDIOC_SOFTRESETCAUSE_USER_REBOOT (0)
 *     User-initiated reboot (NSH `reboot`, application boardctl).
 *     Marker tells t113_fel_rescue this is NOT abnormal so
 *     the counter is not advanced.
 *
 *   any other status (ASSERT=1, PANIC=2, ENTER_BOOTLOADER, ...)
 *     Treated as an abnormal reset path.  No magic is written, so
 *     the next boot's t113_fel_rescue increments the counter.
 *
 ****************************************************************************/

int board_reset(int status)
{
  uint32_t val;

  if (status == 99)
    {
      val = T113_BOOT_MAGIC | T113_BOOT_MODE_FEL;

      /* Clear the reboot-loop counter: entering FEL is a deliberate
       * recovery, the device is no longer in a crash loop.
       */

      putreg32(0, T113_RTC_GP_DATA(1));
    }
  else if (status == BOARDIOC_SOFTRESETCAUSE_USER_REBOOT)
    {
      val = T113_BOOT_MAGIC | T113_BOOT_MODE_USER;
    }
  else
    {
      /* ASSERT / PANIC / BOOTLOADER / RECOVERY / ... - leave marker
       * cleared so the next boot's reboot-loop guard counts this as
       * an abnormal reset.
       */

      val = 0;
    }

  putreg32(val, T113_RTC_GP_DATA(0));

  up_irq_disable();

  /* boot0 runs with caches disabled, so no cache teardown is needed
   * before the WDT-triggered hardware reset.
   */

  /* Disable watchdog - WDOG_SOFT_RST requires WDT disabled first
   * (T113 User Manual section 3.6.6.11).
   */

  putreg32(T113_WDOG_MODE_KEY, T113_WDOG_MODE_REG);

  /* Trigger watchdog module soft reset - instant full-chip reset */

  putreg32(T113_WDOG_SOFT_RST_KEY | 1, T113_WDOG_SOFT_RST_REG);

  /* Wait for reset */

  for (; ; );
  return 0;
}
