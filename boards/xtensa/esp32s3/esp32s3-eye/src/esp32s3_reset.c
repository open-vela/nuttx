/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_reset.c
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

#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "esp32s3_reset_reasons.h"

#ifdef CONFIG_BOARDCTL_RESET_CAUSE

/****************************************************************************
 * Name: board_reset_cause
 ****************************************************************************/

int board_reset_cause(FAR struct boardioc_reset_cause_s *cause)
{
  soc_reset_reason_t reason;

  if (cause == NULL)
    {
      return -EINVAL;
    }

  reason = esp32s3_reset_reasons(0);
  cause->flag = 0;

  switch (reason)
    {
      case RESET_REASON_CHIP_POWER_ON:
        cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
        break;

      case RESET_REASON_CORE_SW:
        cause->cause = BOARDIOC_RESETCAUSE_CORE_SOFT;
        break;

      case RESET_REASON_CORE_DEEP_SLEEP:
        cause->cause = BOARDIOC_RESETCAUSE_CORE_DPSP;
        break;

      case RESET_REASON_CORE_MWDT0:
        cause->cause = BOARDIOC_RESETCAUSE_CORE_MWDT;
        break;

      case RESET_REASON_CORE_MWDT1:
        cause->cause = BOARDIOC_RESETCAUSE_CORE_MWDT;
        cause->flag = 1;
        break;

      case RESET_REASON_CORE_RTC_WDT:
        cause->cause = BOARDIOC_RESETCAUSE_CORE_RWDT;
        cause->flag = 2;
        break;

      case RESET_REASON_CPU0_MWDT0:
        cause->cause = BOARDIOC_RESETCAUSE_CPU_MWDT;
        break;

      case RESET_REASON_CPU0_SW:
        cause->cause = BOARDIOC_RESETCAUSE_CPU_SOFT;
        break;

      case RESET_REASON_CPU0_RTC_WDT:
        cause->cause = BOARDIOC_RESETCAUSE_CPU_RWDT;
        cause->flag = 2;
        break;

      case RESET_REASON_SYS_BROWN_OUT:
        cause->cause = BOARDIOC_RESETCAUSE_SYS_BOR;
        break;

      case RESET_REASON_SYS_RTC_WDT:
        cause->cause = BOARDIOC_RESETCAUSE_SYS_RWDT;
        cause->flag = 2;
        break;

      case RESET_REASON_CPU0_MWDT1:
        cause->cause = BOARDIOC_RESETCAUSE_CPU_MWDT;
        cause->flag = 1;
        break;

      default:
        cause->cause = BOARDIOC_RESETCAUSE_UNKOWN;
        break;
    }

  return OK;
}

#endif /* CONFIG_BOARDCTL_RESET_CAUSE */

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value in this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_reset(int status)
{
  up_systemreset();

  return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */
