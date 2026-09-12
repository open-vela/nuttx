/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_p4_sched_compat.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/signal.h>

#include "esp_p4_sched_compat.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nxsched_usleep(useconds_t usec)
{
  /* The older baseline has the signal-aware sleep primitive.  HAL delays
   * do not consume its status, matching the newer void scheduler helper.
   */

  nxsig_usleep(usec);
}
