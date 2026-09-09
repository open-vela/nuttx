/****************************************************************************
 * arch/arm/src/bk7258/bk7258_lowputc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "hardware/bk7258_mbox.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_lowputc(char ch)
{
  /* Mailbox output needs interrupts and is therefore best-effort here. */

  (void)bk7258_mbox_uart_write((const uint8_t *)&ch, 1);
}
