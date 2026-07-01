/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_eth.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_ETH_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_ETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethernet interface types */

#define RK3576_ETH_IF_MII          0
#define RK3576_ETH_IF_RMII         1
#define RK3576_ETH_IF_RGMII        2

/* Duplex modes */

#define RK3576_ETH_HALF_DUPLEX     0
#define RK3576_ETH_FULL_DUPLEX     1

/* Speed modes */

#define RK3576_ETH_SPEED_10M       0
#define RK3576_ETH_SPEED_100M      1
#define RK3576_ETH_SPEED_1000M     2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_eth_init(int port);

/* Configuration */

int  rk3576_eth_set_interface(int port, int iface);
int  rk3576_eth_set_speed(int port, int speed);
int  rk3576_eth_set_duplex(int port, int duplex);
int  rk3576_eth_set_macaddr(int port, const uint8_t *mac);

/* MDIO (PHY access) */

int  rk3576_eth_mdio_read(int port, int phy_addr, int reg_addr);
int  rk3576_eth_mdio_write(int port, int phy_addr, int reg_addr, uint16_t val);

/* Control */

void rk3576_eth_start(int port);
void rk3576_eth_stop(int port);

/* Status */

int  rk3576_eth_is_link_up(int port);
int  rk3576_eth_get_speed(int port);
int  rk3576_eth_get_duplex(int port);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_ETH_H */
