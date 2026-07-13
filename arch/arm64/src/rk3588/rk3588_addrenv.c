/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_addrenv.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_pa_to_va / up_addrenv_va_to_pa
 *
 * Description:
 *   OpenAMP / rpmsg_virtio use these to translate between the physical
 *   addresses stored in the shared vrings/resource table and the virtual
 *   addresses used by the CPU.  On this RK3588 AMP remote core the MMU uses
 *   a flat identity mapping for DRAM and the AMP shared-memory / rpmsg
 *   carveouts (see rk3588_boot.c), so the translation is the identity.
 *
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  return (uintptr_t)va;
}
