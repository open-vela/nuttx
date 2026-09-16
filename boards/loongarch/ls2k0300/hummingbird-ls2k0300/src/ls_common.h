/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/src/ls_common.h
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

#ifndef __LS_COMMON_H__
#define __LS_COMMON_H__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MHZ_TO_HZ       1000000  /* MHz to Hz */

#define __arch_getb(a) (*(volatile unsigned char *)(a))
#define __arch_getw(a) (*(volatile unsigned short *)(a))
#define __arch_getl(a) (*(volatile unsigned int *)(a))
#define __arch_getq(a) (*(volatile unsigned long long *)(a))

#define __arch_putb(v, a) (*(volatile unsigned char *)(a) = (v))
#define __arch_putw(v, a) (*(volatile unsigned short *)(a) = (v))
#define __arch_putl(v, a) (*(volatile unsigned int *)(a) = (v))
#define __arch_putq(v, a) (*(volatile unsigned long long *)(a) = (v))

#define writeb(v, a) __arch_putb(v, a)
#define writew(v, a) __arch_putw(v, a)
#define writel(v, a) __arch_putl(v, a)
#define writeq(v, a) __arch_putq(v, a)

#define readb(a) __arch_getb(a)
#define readw(a) __arch_getw(a)
#define readl(a) __arch_getl(a)
#define readq(a) __arch_getq(a)

#endif /* __LS_COMMON_H__ */
