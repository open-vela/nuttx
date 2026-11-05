/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rng.c
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

#if defined(CONFIG_STM32H7_RNG) && \
    (defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH))

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/stm32_memorymap.h"
#include "hardware/stm32_rcc.h"
#include "stm32_rng_core.h"

static mutex_t g_rng_lock = NXMUTEX_INITIALIZER;
static struct stm32_rng_state_s g_rng_state;

static const uintptr_t g_rng_registers[] =
{
  STM32_RCC_CR,
  STM32_RCC_D2CCIP2R,
  STM32_RCC_AHB2ENR,
  STM32_RCC_AHB2RSTR,
  STM32_RNG_BASE + 0,
  STM32_RNG_BASE + 4,
  STM32_RNG_BASE + 8
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t rng_read_reg(void *ctx, enum stm32_rng_reg_e reg)
{
  (void)ctx;
  return getreg32(g_rng_registers[reg]);
}

static void rng_write_reg(void *ctx, enum stm32_rng_reg_e reg,
                          uint32_t value)
{
  (void)ctx;
  putreg32(value, g_rng_registers[reg]);
}

static void rng_update_reg(void *ctx, enum stm32_rng_reg_e reg,
                           uint32_t clear, uint32_t set)
{
  irqstate_t flags;

  (void)ctx;
  flags = enter_critical_section();
  modifyreg32(g_rng_registers[reg], clear, set);
  leave_critical_section(flags);
}

static uint32_t rng_now_ms(void *ctx)
{
  (void)ctx;
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

static void rng_wait(void *ctx)
{
  (void)ctx;
  up_udelay(10);
}

static const struct stm32_rng_io_s g_rng_io =
{
  rng_read_reg, rng_write_reg, rng_update_reg, rng_now_ms, rng_wait, NULL
};

static ssize_t rng_read(struct file *filep, char *buffer, size_t length)
{
  int ret;

  (void)filep;
  ret = nxmutex_timedlock(&g_rng_lock, 100);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_rng_core_read(&g_rng_io, &g_rng_state,
                            (uint8_t *)buffer, length);
  nxmutex_unlock(&g_rng_lock);
  return ret;
}

static const struct file_operations g_rng_ops =
{
  .read = rng_read
};

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  register_driver("/dev/random", &g_rng_ops, 0444, NULL);
}
#endif

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  register_driver("/dev/urandom", &g_rng_ops, 0444, NULL);
}
#endif
#endif
