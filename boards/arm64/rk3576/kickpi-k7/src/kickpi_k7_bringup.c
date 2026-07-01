/****************************************************************************
 * boards/arm64/rk3576/kickpi-k7/src/kickpi_k7_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the upper-half driver
 *   initialize logic.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  syslog(LOG_INFO, "KICKPI-K7 (RK3576) board initialized\n");
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   If CONFIG_CUSTOM_APP_INITIALIZE is selected, an application-specific
 *   initialization call will be made from NSH.
 *
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APP_INITIALIZE
int board_app_initialize(int argc, char *argv[])
{
  syslog(LOG_INFO, "KICKPI-K7 application initialized\n");
  return OK;
}
#endif
