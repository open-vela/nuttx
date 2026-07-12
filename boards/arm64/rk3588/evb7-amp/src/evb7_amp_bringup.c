/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_bringup.c
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
#include <nuttx/kthread.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <stdint.h>
#include "evb7_amp.h"

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMP_SHMEM_MAGIC (*(volatile uint32_t *)0x31000000UL)
#define AMP_SHMEM_COUNT (*(volatile uint32_t *)0x31000004UL)  /* timer/usleep */
#define AMP_BUSY_COUNT  (*(volatile uint32_t *)0x31000008UL)  /* busy-loop */
#define AMP_MAGIC_VALUE 0x414d5033u  /* "AMP3" */

/* Direct polled write to the SHARED UART2 (0xfeb50000). Does NOT go through the
 * NuttX serial driver or any IRQ, so it keeps printing even if the interrupt
 * path is disturbed by the Linux GIC takeover. Borrow-only: poll LSR.THRE +
 * write THR, never reconfigure. Output interleaves with the Linux log.
 */

#define UART2_THR (*(volatile uint32_t *)0xfeb50000UL)
#define UART2_LSR (*(volatile uint32_t *)0xfeb50014UL)
#define UART_LSR_THRE (1u << 5)

/* RK3588 mailbox0 @0xfec60000 (V1 regs). cpu_l3(=B) writes B2A_CMD(chan) to
 * ring Linux(=A); Linux's rockchip-mailbox driver already attached the B2A
 * IRQs (SPI 61-64) and rpmsg enabled mailbox0. Min test: kick channel 0 each
 * tick; Linux sees the mailbox IRQ count climb in /proc/interrupts.
 */

#define MBOX0_BASE     0xfec60000UL
#define MBOX0_B2A_CMD0 (*(volatile uint32_t *)(MBOX0_BASE + 0x30UL))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void amp_putc(char c)
{
  unsigned int spin = 0;
  while (!(UART2_LSR & UART_LSR_THRE) && ++spin < 200000)
    {
    }

  UART2_THR = (uint32_t)c;
}

static void amp_puts(const char *s)
{
  while (*s)
    {
      if (*s == '\n')
        {
          amp_putc('\r');
        }

      amp_putc(*s++);
    }
}

static void amp_putu(unsigned int v)
{
  char buf[12];
  int  i = 0;

  if (v == 0)
    {
      amp_putc('0');
      return;
    }

  while (v && i < (int)sizeof(buf))
    {
      buf[i++] = (char)('0' + (v % 10u));
      v /= 10u;
    }

  while (i > 0)
    {
      amp_putc(buf[--i]);
    }
}

/* Heartbeat driven by usleep() -> depends on the arch timer tick. */

static int amp_heartbeat(int argc, char *argv[])
{
  AMP_SHMEM_MAGIC = AMP_MAGIC_VALUE;
  AMP_SHMEM_COUNT = 0;
  amp_puts("[AMP] hb start\n");

  for (; ; )
    {
      AMP_SHMEM_COUNT = AMP_SHMEM_COUNT + 1;

      /* Ring Linux via mailbox0 B2A channel 0 (doorbell min test) */

      MBOX0_B2A_CMD0 = 0xa5a50000u | (AMP_SHMEM_COUNT & 0xffffu);

      amp_puts("[AMP] tick ");
      amp_putu(AMP_SHMEM_COUNT);
      amp_puts(" (kick mbox)\n");
      usleep(1000000);
    }

  return 0;
}

/* Timer-independent busy-loop. Keeps printing as long as the core executes,
 * even if the timer tick is gone. Last "[AMP] busy N" on the wire marks the
 * exact instant cpu_l3 stops executing.
 */

static int amp_busy(int argc, char *argv[])
{
  volatile uint32_t d;

  AMP_BUSY_COUNT = 0;

  for (; ; )
    {
      for (d = 0; d < 40000000; d++)
        {
        }

      AMP_BUSY_COUNT = AMP_BUSY_COUNT + 1;
      amp_puts("[AMP] busy ");
      amp_putu(AMP_BUSY_COUNT);
      amp_puts("\n");
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int evb7_amp_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  amp_puts("[AMP] hello from NuttX on cpu_l3 (RK3588 EVB7 V11 AMP)\n");

  /* Timer-based + timer-independent liveness threads, both print to UART2 */

  kthread_create("amp_hb", 100, 2048, amp_heartbeat, NULL);
  kthread_create("amp_busy", 50, 2048, amp_busy, NULL);

  UNUSED(ret);
  return OK;
}
