/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_i2s.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_I2S_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S mode definitions */

#define RK3576_I2S_MODE_TX          0
#define RK3576_I2S_MODE_RX          1
#define RK3576_I2S_MODE_BOTH        2

/* I2S role definitions */

#define RK3576_I2S_ROLE_MASTER      0
#define RK3576_I2S_ROLE_SLAVE       1

/* I2S format definitions */

#define RK3576_I2S_FMT_I2S          0   /* Standard I2S */
#define RK3576_I2S_FMT_LEFT_J       1   /* Left justified */
#define RK3576_I2S_FMT_RIGHT_J      2   /* Right justified */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_i2s_init(int bus);

/* Configuration */

int  rk3576_i2s_set_samplerate(int bus, uint32_t rate);
int  rk3576_i2s_set_bits(int bus, int bits);
int  rk3576_i2s_set_channels(int bus, int channels);
int  rk3576_i2s_set_mode(int bus, int mode);
int  rk3576_i2s_set_role(int bus, int role);

/* Data transfer (blocking) */

int  rk3576_i2s_send(int bus, const uint16_t *data, int len);
int  rk3576_i2s_recv(int bus, uint16_t *data, int len);

/* DMA control */

void rk3576_i2s_dma_enable(int bus, bool tx, bool rx);
void rk3576_i2s_dma_disable(int bus, bool tx, bool rx);

/* Control */

void rk3576_i2s_start(int bus);
void rk3576_i2s_stop(int bus);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_I2S_H */
