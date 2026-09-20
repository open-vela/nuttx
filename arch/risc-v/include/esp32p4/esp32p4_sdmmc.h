/****************************************************************************
 * arch/risc-v/include/esp32p4/esp32p4_sdmmc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_SDMMC_H
#define __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

FAR struct sdio_dev_s *esp32p4_sdmmc_sdio_initialize(int slotno);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_SDMMC_H */
