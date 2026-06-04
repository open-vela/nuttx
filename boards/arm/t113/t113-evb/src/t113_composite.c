/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_composite.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/adb.h>
#include <nuttx/usb/composite.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *board_composite0_connect(void)
{
  /* Max 2 sub-devices: CDC-ACM + ADB */

  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;
  int dev_idx  = 0;
  int epin     = 1;
  int epout    = 1;

#ifdef CONFIG_CDCACM_COMPOSITE
  cdcacm_get_composite_devdesc(&dev[dev_idx]);
  dev[dev_idx].classobject  = cdcacm_classobject;
  dev[dev_idx].uninitialize = cdcacm_uninitialize;
  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;
  dev[dev_idx].devinfo.strbase = strbase;
  dev[dev_idx].devinfo.epno[CDCACM_EP_INTIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = epout++;

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;
  dev_idx++;
#endif

#ifdef CONFIG_USBADB_COMPOSITE
  usbdev_adb_get_composite_devdesc(&dev[dev_idx]);
  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;
  dev[dev_idx].devinfo.strbase = strbase;
  dev[dev_idx].devinfo.epno[USBADB_EP_BULKIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[USBADB_EP_BULKOUT_IDX] = epout++;

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;
  dev_idx++;
#endif

  DEBUGASSERT(dev_idx > 0 && dev_idx <= 2);
  if (dev_idx == 0)
    {
      return NULL;
    }

  return composite_initialize(composite_getdevdescs(), dev, dev_idx);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_composite_initialize(int port)
{
  return OK;
}

void *board_composite_connect(int port, int configid)
{
  if (configid == 0)
    {
      return board_composite0_connect();
    }

  return NULL;
}
