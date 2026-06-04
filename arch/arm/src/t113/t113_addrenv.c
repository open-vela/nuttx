/****************************************************************************
 * arch/arm/src/t113/t113_addrenv.c
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

#include <nuttx/arch.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Identity physical<->virtual translation for the T113 AP.
 *
 * libmetal / rpmsg_virtio (OpenAMP) call up_addrenv_pa_to_va /
 * up_addrenv_va_to_pa to map the AP<->DSP shared-memory carveout.  The
 * full arm_a_r implementation (arm_virtpgaddr.c) is only built with
 * CONFIG_ARCH_ADDRENV + CONFIG_MM_PGALLOC, which the flat-address DSP
 * loader config does not use.  Every region the AP touches for rptun (the
 * SP1 peripheral window and the DDR shared carveout) is identity-mapped by
 * the board MMU table, so these are plain pass-throughs.
 */

#ifndef CONFIG_ARCH_ADDRENV

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  return (uintptr_t)va;
}

#endif /* CONFIG_ARCH_ADDRENV */
