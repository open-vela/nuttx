/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_adc.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_ADC_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_ADC_H

#define LS2K0300_ADC_SR              0x00
#define LS2K0300_ADC_CR1             0x04
#define LS2K0300_ADC_CR2             0x08
#define LS2K0300_ADC_SMPR1           0x0c
#define LS2K0300_ADC_SMPR2           0x10
#define LS2K0300_ADC_JOFR1           0x14
#define LS2K0300_ADC_JOFR2           0x18
#define LS2K0300_ADC_JOFR3           0x1c
#define LS2K0300_ADC_JOFR4           0x20
#define LS2K0300_ADC_HTR             0x24
#define LS2K0300_ADC_LTR             0x28
#define LS2K0300_ADC_SQR1            0x2c
#define LS2K0300_ADC_SQR2            0x30
#define LS2K0300_ADC_SQR3            0x34
#define LS2K0300_ADC_JSQR            0x38
#define LS2K0300_ADC_JDR1            0x3c
#define LS2K0300_ADC_JDR2            0x40
#define LS2K0300_ADC_JDR3            0x44
#define LS2K0300_ADC_JDR4            0x48
#define LS2K0300_ADC_DR              0x4c

#define ADC_SR_EOC                  (1 << 1)
#define ADC_SR_JEOC                 (1 << 2)

#define ADC_CR1_SCAN                (1 << 8)
#define ADC_CR1_EOC_IE              (1 << 5)
#define ADC_CR1_JEOC_IE             (1 << 7)
#define ADC_CR1_DIFFMOD             (1 << 20)
#define ADC_CR1_CLKDIV_SHIFT        24
#define ADC_CR1_CLKDIV_MASK         (0x3f << ADC_CR1_CLKDIV_SHIFT)
#define ADC_CR1_OUTPHASESEL         (1 << 30)
#define ADC_CR1_CLEAR_MASK          0xc0f0feff

#define ADC_CR2_ADON                (1 << 0)
#define ADC_CR2_CONT                (1 << 1)
#define ADC_CR2_CAL                 (1 << 2)
#define ADC_CR2_RSTCAL              (1 << 3)
#define ADC_CR2_ALIGN               (1 << 11)
#define ADC_CR2_JEXTTRIG            (1 << 15)
#define ADC_CR2_EXTTRIG             (1 << 20)
#define ADC_CR2_EXTSEL_SHIFT        17
#define ADC_CR2_EXTSEL_MASK         (0x7 << ADC_CR2_EXTSEL_SHIFT)
#define ADC_CR2_SWSTART             (1 << 22)
#define ADC_CR2_JSWSTART            (1 << 21)
#define ADC_CR2_JTRIGMOD_SHIFT      24
#define ADC_CR2_CLKDIV_HI_SHIFT     26
#define ADC_CR2_CLKDIV_HI_MASK      (0xf << ADC_CR2_CLKDIV_HI_SHIFT)
#define ADC_CR2_ADCEDGE             (1 << 30)
#define ADC_CR2_CLKMASK             (1 << 31)
#define ADC_CR2_CLEAR_MASK          0x0301f7fd

#define ADC_EXTSEL_SWSTART          (0x7 << ADC_CR2_EXTSEL_SHIFT)

#define ADC_SQR1_L_SHIFT            20
#define ADC_SQR1_L_MASK             (0xf << ADC_SQR1_L_SHIFT)
#define ADC_SQR1_CLEAR_MASK         0xff0fffff

#define ADC_SQR3_SQ_MASK            0x1f
#define ADC_SQR2_SQ_MASK            0x1f
#define ADC_SQR1_SQ_MASK            0x1f

#define ADC_SMPR_SMP_MASK           0x7

#define ADC_DATA_MASK               0x0fff

#define ADC_CHANNEL_COUNT           8

#define ADC_SAMPLETIME_64CYCLES     6

#define ADC_MAX_VOLTAGE_MV          1800
#define ADC_SCALE                   4096

#endif
