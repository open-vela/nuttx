/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/frdm-mcxn947.h
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

#ifndef __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_SRC_FRDM_MCXN947_H
#define __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_SRC_FRDM_MCXN947_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: n94x_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int n94x_bringup(void);
#endif

/****************************************************************************
 * Name: n94x_spidev_initialize
 *
 * Description:
 *   Initialize/register the board SPI device interface.
 *
 ****************************************************************************/

int n94x_spidev_initialize(void);

/****************************************************************************
 * Name: n94x_adc_initialize
 *
 * Description:
 *   Register LPADC as /dev/adc0.
 *
 ****************************************************************************/

int n94x_adc_initialize(void);

/****************************************************************************
 * Name: n94x_pwm_initialize
 *
 * Description:
 *   Register configured CTIMER PWM devices.
 *
 ****************************************************************************/

int n94x_pwm_initialize(void);

/****************************************************************************
 * Name: n94x_airquality_initialize
 *
 * Description:
 *   Initialize LPI2C0 on P0_12/P0_13 and register the combined ENS160 and
 *   AHT21 air-quality module with the Sensor Framework.
 *
 ****************************************************************************/

#ifdef CONFIG_N94X_ENS160_AHT21
int n94x_airquality_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_SRC_FRDM_MCXN947_H */
