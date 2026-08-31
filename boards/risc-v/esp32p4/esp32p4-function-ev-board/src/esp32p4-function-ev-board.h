/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4-function-ev-board.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RMT gpio */

#define RMT_RXCHANNEL       4
#define RMT_TXCHANNEL       0

#ifdef CONFIG_RMT_LOOP_TEST_MODE
#  define RMT_INPUT_PIN     0
#  define RMT_OUTPUT_PIN    0
#else
#  define RMT_INPUT_PIN     2
#  define RMT_OUTPUT_PIN    8
#endif

/* SDIO link to the on-board ESP32-C6 (esp-hosted, Stage 1) *****************/

#ifdef CONFIG_ESP32P4_SDMMC

/* The C6 is on SDMMC slot 1, routed through the GPIO matrix.  The SDIO bus
 * pins (CLK/CMD/D0..D3) are configured by the arch driver from the
 * CONFIG_ESP32P4_SDMMC_* symbols (defaults: CLK=18 CMD=19 D0=14 D1=15
 * D2=16 D3=17).
 */

#  define BOARD_SDMMC_SLOT       CONFIG_ESP32P4_SDMMC_SLOT

/* ESP32-C6 reset line: ACTIVE-HIGH (drive high to assert reset, low to
 * run).  Shared with the esp-hosted menu if that symbol is configured.
 */

#  ifdef CONFIG_ESPRESSIF_ESP_HOSTED_SDIO_RESET
#    define BOARD_C6_RESET_GPIO  CONFIG_ESPRESSIF_ESP_HOSTED_SDIO_RESET
#  else
#    define BOARD_C6_RESET_GPIO  54
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_bringup(void);

/****************************************************************************
 * Name: board_twai_setup
 *
 * Description:
 *  Initialize TWAI and register the TWAI device
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_TWAI
int board_twai_setup(int port);
#endif

/****************************************************************************
 * Name: esp_gpio_init
 *
 * Description:
 *   Configure the GPIO driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_emac_init
 *
 * Description:
 *   Bring up the ESP32-P4 Ethernet MAC driver (esp_eth backed).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_EMAC
int board_emac_init(void);
#endif

/****************************************************************************
 * Name: board_sdmmc_init
 *
 * Description:
 *   Stage-1 esp-hosted bring-up: reset the on-board ESP32-C6, bring up the
 *   SDMMC host on slot 1 as an SDIO host, probe the C6 as an SDIO card and
 *   log its CCCR revision and CIS vendor ID.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC
int board_sdmmc_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H */
