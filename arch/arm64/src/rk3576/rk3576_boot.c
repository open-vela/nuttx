/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_boot.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/chip/chip.h>

#ifdef CONFIG_SMP
#include "arm64_smp.h"
#endif

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "rk3576_boot.h"
#include "rk3576_serial.h"
#ifdef CONFIG_RK3576_GPIO
#  include "rk3576_gpio.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct arm_mmu_region g_mmu_regions[] =
{
  MMU_REGION_FLAT_ENTRY("DEVICE_REGION",
                        CONFIG_DEVICEIO_BASEADDR, CONFIG_DEVICEIO_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("DRAM0_S0",
                        CONFIG_RAMBANK1_ADDR, CONFIG_RAMBANK1_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),
};

const struct arm_mmu_config g_mmu_config =
{
  .num_regions = nitems(g_mmu_regions),
  .mmu_regions = g_mmu_regions,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_el_init
 *
 * Description:
 *   The function called from arm64_head.S at very early stage for these
 * platform, it's use to:
 *   - Handling special hardware initialize routine which is need to
 *     run at high ELs
 *   - Initialize system software such as hypervisor or security firmware
 *     which is need to run at high ELs
 *
 ****************************************************************************/

void arm64_el_init(void)
{
  /* TODO: RK3576 set init sys clock */
}

/****************************************************************************
 * Name: arm64_chip_boot
 *
 * Description:
 *   Complete boot operations started in arm64_head.S
 *
 ****************************************************************************/

void arm64_chip_boot(void)
{
  /* MAP IO and DRAM, enable MMU. */

  arm64_mmu_init(true);

#if defined(CONFIG_ARM64_PSCI)
  arm64_psci_init("smc");

#endif

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

#ifdef CONFIG_RK3576_GPIO
  rk3576_gpio_init();
#endif

  rk3576_board_initialize();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm64_earlyserialinit();
#endif
}

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm64_netinitialize(void)
{
  /* TODO: Support net initialize */
}
#endif

/****************************************************************************
 * Name: arm64_addregion
 *
 * Description:
 *   Add additional memory regions to the heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm64_addregion(void)
{
  /* Additional memory regions can be added here if needed */
}
#endif

/****************************************************************************
 * Name: arm64_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void arm64_usbinitialize(void)
{
  /* USB initialization is handled by rk3576_usb driver */
}
#endif

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the RTC hardware.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  /* RTC is in PMU domain, initialized by bootloader */
  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Return the current RTC time.
 *
 ****************************************************************************/

time_t up_rtc_time(void)
{
  /* Return current time from system clock.
   * A proper implementation would read the RTC registers.
   */

  return 0;
}

/****************************************************************************
 * Name: rk3576_board_initialize
 *
 * Description:
 *   Board-specific initialization called from arm64_chip_boot().
 *
 ****************************************************************************/

void rk3576_board_initialize(void)
{
  /* Board-specific initialization can be added here.
   * This is called before serial init and after MMU setup.
   */
}
