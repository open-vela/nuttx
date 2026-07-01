/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_spi.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SPI_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI mode definitions */

#define RK3576_SPI_MODE_0           0   /* CPOL=0, CPHA=0 */
#define RK3576_SPI_MODE_1           1   /* CPOL=0, CPHA=1 */
#define RK3576_SPI_MODE_2           2   /* CPOL=1, CPHA=0 */
#define RK3576_SPI_MODE_3           3   /* CPOL=1, CPHA=1 */

/* SPI bit order */

#define RK3576_SPI_MSB_FIRST       0
#define RK3576_SPI_LSB_FIRST       1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_spi_init(int bus);

/* Configuration */

int  rk3576_spi_set_mode(int bus, int mode);
int  rk3576_spi_set_bits(int bus, int bits);
int  rk3576_spi_set_frequency(int bus, uint32_t freq);

/* Data transfer (full duplex) */

int  rk3576_spi_exchange(int bus, const uint8_t *txbuf,
                         uint8_t *rxbuf, int len);

/* Convenience wrappers */

int  rk3576_spi_send(int bus, const uint8_t *data, int len);
int  rk3576_spi_recv(int bus, uint8_t *data, int len);
uint8_t rk3576_spi_sendrecv(int bus, uint8_t data);

/* Bus control */

int  rk3576_spi_reset(int bus);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_SPI_H */
