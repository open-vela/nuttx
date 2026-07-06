/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_i2c.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_I2C_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C speed modes */

#define RK3576_I2C_SPEED_STANDARD   100000   /* 100 kHz */
#define RK3576_I2C_SPEED_FAST       400000   /* 400 kHz */
#define RK3576_I2C_SPEED_FAST_PLUS  1000000  /* 1 MHz */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_i2c_init(int bus);

/* Configuration */

int  rk3576_i2c_set_speed(int bus, uint32_t speed);
int  rk3576_i2c_set_address(int bus, uint16_t addr);

/* Data transfer */

int  rk3576_i2c_write(int bus, const uint8_t *data, int len);
int  rk3576_i2c_read(int bus, uint8_t *data, int len);
int  rk3576_i2c_write_reg(int bus, uint8_t reg, const uint8_t *data, int len);
int  rk3576_i2c_read_reg(int bus, uint8_t reg, uint8_t *data, int len);

/* Combined write-then-read (common for register reads) */

int  rk3576_i2c_writeread(int bus, const uint8_t *wbuf, int wlen,
                          uint8_t *rbuf, int rlen);

/* Bus control */

int  rk3576_i2c_reset(int bus);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_I2C_H */
