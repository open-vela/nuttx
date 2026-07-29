/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_lpadc.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPADC_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPADC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N947_LPADC_VERID_OFFSET          0x0000
#define N947_LPADC_PARAM_OFFSET          0x0004
#define N947_LPADC_CTRL_OFFSET           0x0010
#define N947_LPADC_STAT_OFFSET           0x0014
#define N947_LPADC_IE_OFFSET             0x0018
#define N947_LPADC_DE_OFFSET             0x001c
#define N947_LPADC_CFG_OFFSET            0x0020
#define N947_LPADC_PAUSE_OFFSET          0x0024
#define N947_LPADC_SWTRIG_OFFSET         0x0034
#define N947_LPADC_TSTAT_OFFSET          0x0038
#define N947_LPADC_OFSTRIM_OFFSET        0x0040
#define N947_LPADC_TCTRL_OFFSET(n)       (0x00a0 + ((n) << 2))
#define N947_LPADC_FCTRL_OFFSET(n)       (0x00e0 + ((n) << 2))
#define N947_LPADC_GCC_OFFSET(n)         (0x00f0 + ((n) << 2))
#define N947_LPADC_GCR_OFFSET(n)         (0x00f8 + ((n) << 2))
#define N947_LPADC_CMDL_OFFSET(n)        (0x0100 + ((n) << 3))
#define N947_LPADC_CMDH_OFFSET(n)        (0x0104 + ((n) << 3))
#define N947_LPADC_RESFIFO_OFFSET(n)     (0x0300 + ((n) << 2))

/* Register Bitfield Definitions ********************************************/

/* CTRL */

#define LPADC_CTRL_ADCEN                 (1 << 0)
#define LPADC_CTRL_RST                   (1 << 1)
#define LPADC_CTRL_DOZEN                 (1 << 2)
#define LPADC_CTRL_CAL_REQ               (1 << 3)
#define LPADC_CTRL_CALOFS                (1 << 4)
#define LPADC_CTRL_RSTFIFO0              (1 << 8)
#define LPADC_CTRL_RSTFIFO1              (1 << 9)
#define LPADC_CTRL_CAL_AVGS_SHIFT        16
#define LPADC_CTRL_CAL_AVGS_MASK         (15 << LPADC_CTRL_CAL_AVGS_SHIFT)
#define LPADC_CTRL_CAL_AVGS(n)           ((uint32_t)(n) << \
                                           LPADC_CTRL_CAL_AVGS_SHIFT)

/* STAT */

#define LPADC_STAT_RDY0                  (1 << 0)
#define LPADC_STAT_FOF0                  (1 << 1)
#define LPADC_STAT_RDY1                  (1 << 2)
#define LPADC_STAT_FOF1                  (1 << 3)
#define LPADC_STAT_TEXC_INT              (1 << 8)
#define LPADC_STAT_TCOMP_INT             (1 << 9)
#define LPADC_STAT_CAL_RDY               (1 << 10)
#define LPADC_STAT_ADC_ACTIVE            (1 << 11)
#define LPADC_STAT_TRGACT_SHIFT          16
#define LPADC_STAT_TRGACT_MASK           (3 << LPADC_STAT_TRGACT_SHIFT)
#define LPADC_STAT_CMDACT_SHIFT          24
#define LPADC_STAT_CMDACT_MASK           (15 << LPADC_STAT_CMDACT_SHIFT)

#define LPADC_STAT_W1C_MASK              (LPADC_STAT_FOF0 | \
                                           LPADC_STAT_FOF1 | \
                                           LPADC_STAT_TEXC_INT | \
                                           LPADC_STAT_TCOMP_INT | \
                                           LPADC_STAT_CAL_RDY)

/* IE */

#define LPADC_IE_FWMIE0                  (1 << 0)
#define LPADC_IE_FOFIE0                  (1 << 1)
#define LPADC_IE_FWMIE1                  (1 << 2)
#define LPADC_IE_FOFIE1                  (1 << 3)
#define LPADC_IE_TEXC_IE                 (1 << 8)
#define LPADC_IE_TCOMP_IE_SHIFT          16
#define LPADC_IE_TCOMP_IE_MASK           (15 << LPADC_IE_TCOMP_IE_SHIFT)

/* CFG */

#define LPADC_CFG_TPRICTRL_SHIFT         0
#define LPADC_CFG_TPRICTRL_MASK          (3 << LPADC_CFG_TPRICTRL_SHIFT)
#define LPADC_CFG_TPRICTRL(n)            ((uint32_t)(n) << \
                                           LPADC_CFG_TPRICTRL_SHIFT)
#define LPADC_CFG_PWRSEL_SHIFT           4
#define LPADC_CFG_PWRSEL_MASK            (3 << LPADC_CFG_PWRSEL_SHIFT)
#define LPADC_CFG_PWRSEL(n)              ((uint32_t)(n) << \
                                           LPADC_CFG_PWRSEL_SHIFT)
#define LPADC_CFG_REFSEL_SHIFT           6
#define LPADC_CFG_REFSEL_MASK            (3 << LPADC_CFG_REFSEL_SHIFT)
#define LPADC_CFG_REFSEL(n)              ((uint32_t)(n) << \
                                           LPADC_CFG_REFSEL_SHIFT)
#define LPADC_CFG_TRES                   (1 << 8)
#define LPADC_CFG_TCMDRES                (1 << 9)
#define LPADC_CFG_HPT_EXDI               (1 << 10)
#define LPADC_CFG_PUDLY_SHIFT            16
#define LPADC_CFG_PUDLY_MASK             (0xff << LPADC_CFG_PUDLY_SHIFT)
#define LPADC_CFG_PUDLY(n)               ((uint32_t)(n) << \
                                           LPADC_CFG_PUDLY_SHIFT)
#define LPADC_CFG_PWREN                  (1 << 28)

/* SWTRIG */

#define LPADC_SWTRIG_SWT(n)              (1 << (n))

/* TSTAT */

#define LPADC_TSTAT_TEXC_NUM_SHIFT       0
#define LPADC_TSTAT_TEXC_NUM_MASK        (15 << LPADC_TSTAT_TEXC_NUM_SHIFT)
#define LPADC_TSTAT_TCOMP_FLAG_SHIFT     16
#define LPADC_TSTAT_TCOMP_FLAG_MASK      (15 << LPADC_TSTAT_TCOMP_FLAG_SHIFT)
#define LPADC_TSTAT_W1C_MASK             (LPADC_TSTAT_TEXC_NUM_MASK | \
                                           LPADC_TSTAT_TCOMP_FLAG_MASK)

/* TCTRL */

#define LPADC_TCTRL_HTEN                 (1 << 0)
#define LPADC_TCTRL_FIFO_SEL_A           (1 << 1)
#define LPADC_TCTRL_FIFO_SEL_B           (1 << 2)
#define LPADC_TCTRL_TPRI_SHIFT           8
#define LPADC_TCTRL_TPRI_MASK            (3 << LPADC_TCTRL_TPRI_SHIFT)
#define LPADC_TCTRL_TPRI(n)              ((uint32_t)(n) << \
                                           LPADC_TCTRL_TPRI_SHIFT)
#define LPADC_TCTRL_RSYNC                (1 << 15)
#define LPADC_TCTRL_TDLY_SHIFT           16
#define LPADC_TCTRL_TDLY_MASK            (15 << LPADC_TCTRL_TDLY_SHIFT)
#define LPADC_TCTRL_TDLY(n)              ((uint32_t)(n) << \
                                           LPADC_TCTRL_TDLY_SHIFT)
#define LPADC_TCTRL_TCMD_SHIFT           24
#define LPADC_TCTRL_TCMD_MASK            (15 << LPADC_TCTRL_TCMD_SHIFT)
#define LPADC_TCTRL_TCMD(n)              ((uint32_t)(n) << \
                                           LPADC_TCTRL_TCMD_SHIFT)

/* FCTRL */

#define LPADC_FCTRL_FCOUNT_SHIFT         0
#define LPADC_FCTRL_FCOUNT_MASK          (31 << LPADC_FCTRL_FCOUNT_SHIFT)
#define LPADC_FCTRL_FCOUNT(n)            (((n) & LPADC_FCTRL_FCOUNT_MASK) >> \
                                           LPADC_FCTRL_FCOUNT_SHIFT)
#define LPADC_FCTRL_FWMARK_SHIFT         16
#define LPADC_FCTRL_FWMARK_MASK          (15 << LPADC_FCTRL_FWMARK_SHIFT)
#define LPADC_FCTRL_FWMARK(n)            ((uint32_t)(n) << \
                                           LPADC_FCTRL_FWMARK_SHIFT)

/* CMDL */

#define LPADC_CMDL_ADCH_SHIFT            0
#define LPADC_CMDL_ADCH_MASK             (31 << LPADC_CMDL_ADCH_SHIFT)
#define LPADC_CMDL_ADCH(n)               ((uint32_t)(n) << \
                                           LPADC_CMDL_ADCH_SHIFT)
#define LPADC_CMDL_CTYPE_SHIFT           5
#define LPADC_CMDL_CTYPE_MASK            (3 << LPADC_CMDL_CTYPE_SHIFT)
#define LPADC_CMDL_CTYPE(n)              ((uint32_t)(n) << \
                                           LPADC_CMDL_CTYPE_SHIFT)
#define LPADC_CMDL_MODE                  (1 << 7)
#define LPADC_CMDL_ALTB_ADCH_SHIFT       16
#define LPADC_CMDL_ALTB_ADCH_MASK        (31 << LPADC_CMDL_ALTB_ADCH_SHIFT)
#define LPADC_CMDL_ALTB_ADCH(n)          ((uint32_t)(n) << \
                                           LPADC_CMDL_ALTB_ADCH_SHIFT)
#define LPADC_CMDL_ALTBEN                (1 << 21)

/* CMDH */

#define LPADC_CMDH_CMPEN_SHIFT           0
#define LPADC_CMDH_CMPEN_MASK            (3 << LPADC_CMDH_CMPEN_SHIFT)
#define LPADC_CMDH_CMPEN(n)              ((uint32_t)(n) << \
                                           LPADC_CMDH_CMPEN_SHIFT)
#define LPADC_CMDH_WAIT_TRIG             (1 << 2)
#define LPADC_CMDH_LWI                   (1 << 7)
#define LPADC_CMDH_STS_SHIFT             8
#define LPADC_CMDH_STS_MASK              (7 << LPADC_CMDH_STS_SHIFT)
#define LPADC_CMDH_STS(n)                ((uint32_t)(n) << \
                                           LPADC_CMDH_STS_SHIFT)
#define LPADC_CMDH_AVGS_SHIFT            12
#define LPADC_CMDH_AVGS_MASK             (15 << LPADC_CMDH_AVGS_SHIFT)
#define LPADC_CMDH_AVGS(n)               ((uint32_t)(n) << \
                                           LPADC_CMDH_AVGS_SHIFT)
#define LPADC_CMDH_LOOP_SHIFT            16
#define LPADC_CMDH_LOOP_MASK             (15 << LPADC_CMDH_LOOP_SHIFT)
#define LPADC_CMDH_LOOP(n)               ((uint32_t)(n) << \
                                           LPADC_CMDH_LOOP_SHIFT)
#define LPADC_CMDH_NEXT_SHIFT            24
#define LPADC_CMDH_NEXT_MASK             (15 << LPADC_CMDH_NEXT_SHIFT)
#define LPADC_CMDH_NEXT(n)               ((uint32_t)(n) << \
                                           LPADC_CMDH_NEXT_SHIFT)

/* RESFIFO */

#define LPADC_RESFIFO_D_SHIFT            0
#define LPADC_RESFIFO_D_MASK             (0xffff << LPADC_RESFIFO_D_SHIFT)
#define LPADC_RESFIFO_TSRC_SHIFT         16
#define LPADC_RESFIFO_TSRC_MASK          (3 << LPADC_RESFIFO_TSRC_SHIFT)
#define LPADC_RESFIFO_LOOPCNT_SHIFT      20
#define LPADC_RESFIFO_LOOPCNT_MASK       (15 << LPADC_RESFIFO_LOOPCNT_SHIFT)
#define LPADC_RESFIFO_CMDSRC_SHIFT       24
#define LPADC_RESFIFO_CMDSRC_MASK        (15 << LPADC_RESFIFO_CMDSRC_SHIFT)
#define LPADC_RESFIFO_VALID              (1u << 31)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPADC_H */
