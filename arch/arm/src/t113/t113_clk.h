/****************************************************************************
 * arch/arm/src/t113/t113_clk.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_CLK_H
#define __ARCH_ARM_SRC_T113_T113_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock IDs for t113_clk_enable() / t113_clk_disable() */

enum t113_clk_id_e
{
  T113_CLK_DE,              /* DE0 functional clock        CCU 0x0600 bit31 */
  T113_CLK_DE_BUS,          /* DE0 bus clock gate          CCU 0x060C bit0  */
  T113_CLK_DPSS_TOP,        /* DPSS_TOP bus clock gate     CCU 0x0ABC bit0  */
  T113_CLK_DSI,             /* MIPI DSI functional clock   CCU 0x0B24 bit31 */
  T113_CLK_DSI_BUS,         /* MIPI DSI bus clock gate     CCU 0x0B4C bit0  */
  T113_CLK_TCON_LCD0,       /* TCON-LCD0 functional clock  CCU 0x0B60 bit31 */
  T113_CLK_TCON_LCD0_BUS,   /* TCON-LCD0 bus clock gate    CCU 0x0B7C bit0  */
};

/* Reset IDs for t113_clk_reset_assert() / t113_clk_reset_deassert() */

enum t113_rst_id_e
{
  T113_RST_DE,              /* DE0 reset              CCU 0x060C bit16 */
  T113_RST_DPSS_TOP,        /* DPSS_TOP reset         CCU 0x0ABC bit16 */
  T113_RST_DSI,             /* MIPI DSI reset         CCU 0x0B4C bit16 */
  T113_RST_TCON_LCD0,       /* TCON-LCD0 reset        CCU 0x0B7C bit16 */
};

/* Audio clock consumers (PLL_AUDIO0 sharing) */

enum t113_audio_consumer_e
{
  T113_AUDIO_CONSUMER_DMIC = 0,
  T113_AUDIO_CONSUMER_CODEC_ADC,
  T113_AUDIO_CONSUMER_CODEC_DAC,
  T113_AUDIO_CONSUMER_NUM
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void t113_clk_init(void);

/* Return the live APB1 bus-clock frequency (Hz) decoded from APB1_CLK.
 * Used when a foreign master owns the shared clock tree (Linux remoteproc)
 * so the UART baud divisor tracks the real rate instead of assuming the
 * compile-time T113_APB1_FREQUENCY.
 */

uint32_t t113_apb1_freq(void);

void t113_clk_enable(enum t113_clk_id_e id);
void t113_clk_disable(enum t113_clk_id_e id);

void t113_clk_reset_assert(enum t113_rst_id_e id);
void t113_clk_reset_deassert(enum t113_rst_id_e id);

/* SMHC bus/clock control.
 *
 * t113_smhc_clk_enable(bus, freq_hz):
 *   Ungate the SMHCn bus clock, de-assert its reset, select PLL_PERI(1X)
 *   as the module clock source, and program the divider so SDCLK equals
 *   freq_hz.  Bus is 0..2 (SMHC0/1/2).  Returns 0 on success or a
 *   negated errno on invalid argument.  For Phase 1 only 25 MHz is
 *   validated (the mmcsd layer raises to 25 MHz after CMD0/CMD8
 *   initialisation at 400 kHz).
 *
 * t113_smhc_clk_disable(bus):
 *   Gate the SMHCn bus clock and hold the module in reset.  Intended
 *   for driver shutdown / suspend.
 */

int  t113_smhc_clk_enable(int bus, uint32_t freq_hz);
void t113_smhc_clk_disable(int bus);

/* Audio clock provider.
 *
 *   t113_audio_clk_request(consumer, target_rate):
 *     Refcount-managed PLL_AUDIO0 enable + per-consumer divider configure
 *     for one of {DMIC, codec ADC, codec DAC}.  PLL_AUDIO0 is brought up
 *     on the first request and torn down when the last consumer releases.
 *     target_rate is the audio sample rate in Hz (e.g. 16000, 48000).
 *     Returns 0 on success or a negated errno:
 *       -EBUSY  : duplicate request from same consumer
 *       -EINVAL : unsupported sample rate, or codec ADC/DAC requesting a
 *                 rate that disagrees with the other codec consumer
 *                 already active (the codec analog block cannot run two
 *                 different rates simultaneously)
 *       -ETIMEDOUT : PLL_AUDIO0 failed to lock within 10ms
 *
 *   t113_audio_clk_release(consumer):
 *     Reverse of request: gate the consumer divider, decrement refcount,
 *     and disable PLL_AUDIO0 once it reaches zero.  Returns 0 on success
 *     or -EINVAL if the consumer was not active.
 *
 *   This API is not safe to call from IRQ context (uses nxmutex).
 */

int t113_audio_clk_request(enum t113_audio_consumer_e consumer,
                           uint32_t target_rate);
int t113_audio_clk_release(enum t113_audio_consumer_e consumer);

#endif /* __ARCH_ARM_SRC_T113_T113_CLK_H */
