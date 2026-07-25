/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_addrenv.c
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
#include <nuttx/arch.h>

#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_pa_to_va / up_addrenv_va_to_pa
 *
 * Description:
 *   OpenAMP and rpmsg_virtio use these to translate between the addresses
 *   stored in the shared vrings and resource table and the addresses this core
 *   can actually dereference.
 *
 *   On this core the translation is NOT the identity, unlike the cpu_l3 port
 *   where the MMU maps DRAM flat. Linux is the virtio driver: it fills
 *   descriptors with PHYSICAL addresses inside its rpmsg carveout (0x07b00000
 *   and up). The M0 reaches that same memory through its off-chip DDR window,
 *   which the boot firmware aimed at the carveout base, so the same bytes
 *   appear at RK3588M0_EXSRAM_BASE. Translating means rebasing between the two
 *   views.
 *
 *   Addresses outside the shared window are passed through unchanged: those are
 *   this core's own pointers (the code window is flat), and OpenAMP hands both
 *   kinds through these hooks.
 *
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  if (pa >= RK3588M0_RPMSG_PHYS &&
      pa <  RK3588M0_RPMSG_PHYS + RK3588M0_RPMSG_WINDOW_SIZE)
    {
      return (FAR void *)(pa - RK3588M0_RPMSG_PHYS + RK3588M0_EXSRAM_BASE);
    }

  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  uintptr_t addr = (uintptr_t)va;

  if (addr >= RK3588M0_EXSRAM_BASE &&
      addr <  RK3588M0_EXSRAM_BASE + RK3588M0_RPMSG_WINDOW_SIZE)
    {
      return addr - RK3588M0_EXSRAM_BASE + RK3588M0_RPMSG_PHYS;
    }

  return addr;
}
