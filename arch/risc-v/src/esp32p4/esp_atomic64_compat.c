/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_atomic64_compat.c
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

/* The ESP32-P4 is an RV32IMAC core: the A extension does not cover 64-bit
 * atomic memory operations, so GCC emits calls into libatomic for
 * stdatomic accesses on uint64_t (e.g. esp_gpio_reserve.c from esp-hal).
 * The Espressif riscv32-esp-elf toolchain ships no libatomic, so provide
 * critical-section based implementations for the helpers in use.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint64_t __atomic_fetch_or_8(volatile uint64_t *ptr, uint64_t val,
                             int mem_model)
{
  irqstate_t flags;
  uint64_t old;

  flags = up_irq_save();
  old = *ptr;
  *ptr = old | val;
  up_irq_restore(flags);

  return old;
}

uint64_t __atomic_fetch_and_8(volatile uint64_t *ptr, uint64_t val,
                              int mem_model)
{
  irqstate_t flags;
  uint64_t old;

  flags = up_irq_save();
  old = *ptr;
  *ptr = old & val;
  up_irq_restore(flags);

  return old;
}

uint64_t __atomic_load_8(const volatile uint64_t *ptr, int mem_model)
{
  irqstate_t flags;
  uint64_t val;

  flags = up_irq_save();
  val = *ptr;
  up_irq_restore(flags);

  return val;
}

void __atomic_store_8(volatile uint64_t *ptr, uint64_t val, int mem_model)
{
  irqstate_t flags;

  flags = up_irq_save();
  *ptr = val;
  up_irq_restore(flags);
}
