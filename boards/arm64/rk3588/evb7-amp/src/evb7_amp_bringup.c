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

/* Cross-core heartbeat in amp-shmem@31000000 (outside NuttX RAM and Linux
 * no-map). Linux reads it via /dev/mem: busybox devmem 0x31000004 32.
 */

#define AMP_SHMEM_MAGIC (*(volatile uint32_t *)0x31000000UL)
#define AMP_SHMEM_COUNT (*(volatile uint32_t *)0x31000004UL)
#define AMP_MAGIC_VALUE 0x414d5033u  /* "AMP3" */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int amp_heartbeat(int argc, char *argv[])
{
  AMP_SHMEM_MAGIC = AMP_MAGIC_VALUE;
  AMP_SHMEM_COUNT = 0;
  syslog(LOG_INFO, "[AMP] NuttX heartbeat running @0x31000000\n");

  for (; ; )
    {
      AMP_SHMEM_COUNT = AMP_SHMEM_COUNT + 1;
      usleep(1000000);
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

  syslog(LOG_INFO,
         "[AMP] hello from NuttX on cpu_l3 (RK3588 EVB7 V11 AMP)\n");

  /* Start the cross-core heartbeat so Linux can confirm this core is alive */

  kthread_create("amp_hb", 100, 2048, amp_heartbeat, NULL);

  UNUSED(ret);
  return OK;
}
