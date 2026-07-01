/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_usb.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_USB_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_USB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB mode definitions */

#define RK3576_USB_MODE_HOST        0
#define RK3576_USB_MODE_DEVICE      1
#define RK3576_USB_MODE_OTG         2

/* USB speed definitions */

#define RK3576_USB_SPEED_HIGH       0
#define RK3576_USB_SPEED_FULL       1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_usb_init(int port);
void rk3576_usb_deinit(int port);

/* Mode control */

int  rk3576_usb_set_mode(int port, int mode);

/* Host mode operations */

int  rk3576_usb_host_reset(int port);
int  rk3576_usb_host_get_speed(int port);
int  rk3576_usb_host_connect(int port);
int  rk3576_usb_host_disconnect(int port);

/* Device mode operations */

int  rk3576_usb_device_set_address(int port, int addr);
int  rk3576_usb_device_connect(int port);
int  rk3576_usb_device_disconnect(int port);

/* Utility */

int  rk3576_usb_get_status(int port);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_USB_H */
