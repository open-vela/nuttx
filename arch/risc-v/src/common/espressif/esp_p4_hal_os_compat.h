/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_p4_hal_os_compat.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_HAL_OS_COMPAT_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_HAL_OS_COMPAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>
#include <spawn.h>

#include <nuttx/sched.h>

#include "esp_irq_p4.h"
#include "esp_p4_sched_compat.h"

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Adapt the newer NuttX task initializer used by esp-hal-3rdparty to the
 * spawn-attribute based initializer present in the openvela baseline.
 */

static inline int esp_p4_nxtask_init_compat(
  FAR struct tcb_s *tcb, FAR const char *name, int priority,
  FAR void *stack, uint32_t stack_size, main_t entry,
  FAR char * const argv[], FAR char * const envp[],
  FAR const posix_spawn_file_actions_t *actions)
{
  posix_spawnattr_t attr;
  int ret;

  posix_spawnattr_init(&attr);
  posix_spawnattr_setpriority(&attr, priority);
  posix_spawnattr_setstackaddr(&attr, stack);
  posix_spawnattr_setstacksize(&attr, stack_size);

  ret = nxtask_init(tcb, name, entry, actions, &attr, argv, envp);
  posix_spawnattr_destroy(&attr);
  return ret;
}

/* This header is force-included only while compiling the HAL OS adapter. */

#define nxtask_init(t,n,p,s,z,e,a,v,f) \
  esp_p4_nxtask_init_compat(t,n,p,s,z,e,a,v,f)

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_HAL_OS_COMPAT_H */
