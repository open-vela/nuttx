/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_p4_sched_compat.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_SCHED_COMPAT_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_SCHED_COMPAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* esp-hal-3rdparty uses this scheduler-only delay helper.  The openvela
 * contest baseline predates that NuttX API, so provide it in the P4 port.
 */

void nxsched_usleep(useconds_t usec);

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_P4_SCHED_COMPAT_H */
