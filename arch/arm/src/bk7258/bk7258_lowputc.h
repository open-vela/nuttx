/****************************************************************************
 * arch/arm/src/bk7258/bk7258_lowputc.h
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

#ifndef __ARCH_ARM_SRC_BK7258_BK7258_LOWPUTC_H
#define __ARCH_ARM_SRC_BK7258_BK7258_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_lowsetup
 *
 * Description:
 *   Configure the console UART (UART0) to 115200 8N1 and enable TX.
 *   Called once by the chip start-up path (bk7258_start) before
 *   arm_lowputc() becomes usable.
 *
 ****************************************************************************/

void bk7258_lowsetup(void);

#endif /* __ARCH_ARM_SRC_BK7258_BK7258_LOWPUTC_H */
