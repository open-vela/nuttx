/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_psram.h
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

/* Register definitions for the BK7258 PSRAM controller.
 *
 * Source: Armino psram_ll_macro_def.h + psram_hal.h, Beken, Apache-2.0
 */

#ifndef __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_PSRAM_H
#define __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_PSRAM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PSRAM controller register base *******************************************/

#define PSRAM_LL_REG_BASE      0x46080000

/* PSRAM controller registers ***********************************************/

#define PSRAM_LL_REG0          (PSRAM_LL_REG_BASE + 0x00)  /* Status */
#define PSRAM_LL_REG2          (PSRAM_LL_REG_BASE + 0x08)  /* SF reset */
#define PSRAM_LL_REG4          (PSRAM_LL_REG_BASE + 0x10)  /* Mode/wrap */
#define PSRAM_LL_REG5          (PSRAM_LL_REG_BASE + 0x14)  /* Drive str */
#define PSRAM_LL_REG8          (PSRAM_LL_REG_BASE + 0x20)  /* Cmd trigger */
#define PSRAM_LL_REG9          (PSRAM_LL_REG_BASE + 0x24)  /* Cmd address */
#define PSRAM_LL_REGA          (PSRAM_LL_REG_BASE + 0x28)  /* Cmd write data */
#define PSRAM_LL_REGB          (PSRAM_LL_REG_BASE + 0x2C)  /* Cmd read data */

/* REG2 bit definitions *****************************************************/

#define PSRAM_SF_RESET_BIT     (1 << 0)  /* 1 = release SF reset */

/* REG8 command bits ********************************************************/

#define PSRAM_CMD_WRITE_TRIG   (1 << 0)  /* Write cmd (auto-clear) */
#define PSRAM_CMD_READ_TRIG    (1 << 1)  /* Read cmd (poll until 0) */
#define PSRAM_CMD_RESET        (1 << 2)  /* Reset command */

/* System analog register addresses *****************************************/

#define SYS_ANA_REG13          0x44010134  /* PSRAM LDO + voltage */
#define SYS_CPU_CLK_DIV2       0x44010024  /* PSRAM clock sel/div */

/* ana_reg13 bit definitions ************************************************/

#define ANA_REG13_ENPSRAM      (1 << 31)  /* PSRAM LDO enable */
#define ANA_REG13_VPSRAMSEL_S  29         /* Voltage select [30:29] */
#define ANA_REG13_VPSRAMSEL_M  (0x3 << 29)
#define ANA_REG13_PSLDO_SWB    (1 << 28)  /* 1=low-voltage range */

/* cpu_clk_div_mode2 bit definitions ****************************************/

#define CLK_DIV2_CKDIV_PSRAM   (1 << 4)   /* Clock divider */
#define CLK_DIV2_CKSEL_PSRAM   (1 << 5)   /* Clock source sel */

/* Voltage presets (ana_reg13 fields) ***************************************/

#define PSRAM_VOLTAGE_1_95V    (ANA_REG13_PSLDO_SWB | \
                                (3 << ANA_REG13_VPSRAMSEL_S))
#define PSRAM_VOLTAGE_LDO_EN   (ANA_REG13_ENPSRAM | PSRAM_VOLTAGE_1_95V)

/* Known PSRAM chip IDs *****************************************************/

#define PSRAM_ID_APS6408L      0x8d09  /* AP Memory 8MB */
#define PSRAM_ID_APS128XXO     0x8d08  /* AP Memory 16MB */
#define PSRAM_ID_W955D8MKY     0x1c8f  /* Winbond 4MB (BK7258) */

/* PSRAM mode register values (written to REG4) *****************************/

#define PSRAM_MODE_APS6408L    0xd8054041
#define PSRAM_MODE_APS128XXO   0xd8054049
#define PSRAM_MODE_W955D8MKY   0xb0054045

/* PSRAM drive strength (written to REG5) ***********************************/

#define PSRAM_DRV_APS6408L     0x380
#define PSRAM_DRV_APS128XXO    0x380
#define PSRAM_DRV_W955D8MKY    0x292

#endif /* __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_PSRAM_H */
