/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_addrenv.c
 *
 * Identity physical<->virtual translation for the RK3576 AMP slave.
 *
 * The rptun / rpmsg-virtio layer translates the shared-memory addresses it
 * finds in the resource table through up_addrenv_pa_to_va() /
 * up_addrenv_va_to_pa().  This image runs MMU-enabled with a flat identity
 * map (see rk3576_boot.c), so both are the identity function.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  return (uintptr_t)va;
}
