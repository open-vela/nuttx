/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_wifi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 AP-side Wi-Fi front-end.
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_WIFI_H
#define __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_WIFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_WIFI_REASON_NO_AP_FOUND    257u
#define BK7258_WIFI_REASON_WRONG_PASSWORD 258u

int bk7258_wifi_initialize(void);
int bk7258_wifi_random_block(void *buffer, size_t length);

enum bk7258_wifi_event_e
{
  BK7258_WIFI_EVENT_STA_CONNECTED = 0,
  BK7258_WIFI_EVENT_STA_DISCONNECTED,
  BK7258_WIFI_EVENT_AP_STARTED,
  BK7258_WIFI_EVENT_AP_STOPPED,
  BK7258_WIFI_EVENT_AP_CLIENTS_CHANGED
};

typedef void (*bk7258_wifi_event_cb_t)(enum bk7258_wifi_event_e event,
                                       unsigned int value, void *arg);

int bk7258_wifi_register_event_callback(bk7258_wifi_event_cb_t callback,
                                        void *arg);
void bk7258_wifi_unregister_event_callback(bk7258_wifi_event_cb_t callback,
                                           void *arg);

#endif
