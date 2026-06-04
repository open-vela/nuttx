/****************************************************************************
 * arch/arm/src/t113/t113_hstimer.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_HSTIMER_H
#define __ARCH_ARM_SRC_T113_T113_HSTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/timer.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: t113_hstimer_initialize
 *
 * Description:
 *   Initialize a T113 High-Speed Timer channel and return the timer
 *   lower-half driver instance.
 *
 * Input Parameters:
 *   timer - Timer channel number (0 or 1).
 *
 * Returned Value:
 *   A pointer to the timer lower-half driver instance on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct timer_lowerhalf_s *t113_hstimer_initialize(int timer);

#endif /* __ARCH_ARM_SRC_T113_T113_HSTIMER_H */
