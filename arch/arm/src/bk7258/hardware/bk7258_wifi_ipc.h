/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_wifi_ipc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 controller-interface Wi-Fi wire ABI.
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_WIFI_IPC_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_WIFI_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_WIFI_CMD_TX_CHANNEL       0x14u
#define BK7258_WIFI_DATA_TX_CHANNEL      0x15u
#define BK7258_WIFI_CMD_RX_CHANNEL       0x44u
#define BK7258_WIFI_DATA_RX_CHANNEL      0x45u

#define BK7258_WIFI_CMD_CONNECT          2u
#define BK7258_WIFI_CMD_DISCONNECT       3u
#define BK7258_WIFI_CMD_GET_WLAN_STATUS  7u
#define BK7258_WIFI_CMD_SET_AUTO_RECONNECT 10u
#define BK7258_WIFI_CMD_SCAN_WIFI         1u
#define BK7258_WIFI_CMD_GET_MAC_ADDR      0x202u
#define BK7258_WIFI_CMD_OPENVELA_SCAN_PAGE 0x20du
#define BK7258_WIFI_CMD_OPENVELA_COUNTRY_GET 0x20eu
#define BK7258_WIFI_CMD_OPENVELA_COUNTRY_SET 0x20fu
#define BK7258_WIFI_CMD_OPENVELA_AP_START 0x210u
#define BK7258_WIFI_CMD_OPENVELA_AP_STOP 0x211u
#define BK7258_WIFI_CMD_OPENVELA_AP_STATUS 0x212u
#define BK7258_WIFI_CFM_OFFSET            0x8000u

#define BK7258_WIFI_EVT_IPV4_IND          1u
#ifdef CONFIG_BK7258_WIFI_CP_EXTENSIONS
#define BK7258_WIFI_EVT_DISCONNECT_IND    3u
#define BK7258_WIFI_EVT_START_AP_IND      4u
#define BK7258_WIFI_EVT_ASSOC_AP_IND      5u
#define BK7258_WIFI_EVT_DISASSOC_AP_IND   6u
#define BK7258_WIFI_EVT_STOP_AP_IND       7u
#define BK7258_WIFI_EVT_SCAN_WIFI_IND     8u
#else
#define BK7258_WIFI_EVT_DISCONNECT_IND    2u
#define BK7258_WIFI_EVT_START_AP_IND      3u
#define BK7258_WIFI_EVT_ASSOC_AP_IND      4u
#define BK7258_WIFI_EVT_DISASSOC_AP_IND    5u
#define BK7258_WIFI_EVT_STOP_AP_IND       6u
#define BK7258_WIFI_EVT_SCAN_WIFI_IND     7u
#endif
#define BK7258_WIFI_EVT_WIFI_EVENT_IND    0x12u
#define BK7258_WIFI_EVENT_CONNECTED       2u
#define BK7258_WIFI_EVENT_DISCONNECTED    4u

#define BK7258_WIFI_MAX_FRAME             1514u
#define BK7258_WIFI_MAX_LIST              60u
#define BK7258_WIFI_CMD_SLOTS             8u
#define BK7258_WIFI_CMD_SLOT_SIZE         800u
#define BK7258_WIFI_CMD_PATTERN_FREE      0xf3eef3eeu
#define BK7258_WIFI_CMD_PATTERN_BUSY      0xcafebabeu
#define BK7258_WIFI_TX_HEADROOM           708u
#define BK7258_WIFI_SCAN_PAGE_RECORDS      4u
#define BK7258_WIFI_WIRE_VIF_STA          0u
#define BK7258_WIFI_WIRE_VIF_SOFTAP       1u
#define BK7258_WIFI_CPDU_VIF_SHIFT        2u
#define BK7258_WIFI_CPDU_VIF_MASK         0x0cu
#define BK7258_WIFI_AP_ABI_VERSION        1u
#define BK7258_WIFI_AP_SECURITY_OPEN      0u
#define BK7258_WIFI_AP_SECURITY_WPA2      1u
#define BK7258_WIFI_AP_MAX_CLIENTS        1u

struct bk7258_wifi_ap_start_request
{
  uint8_t version;
  uint8_t channel;
  uint8_t security;
  uint8_t hidden;
  uint8_t max_clients;
  uint8_t ssid_length;
  uint8_t password_length;
  uint8_t reserved;
  uint8_t ssid[32];
  uint8_t password[64];
};

struct bk7258_wifi_ap_status_response
{
  int32_t status;
  uint8_t started;
  uint8_t channel;
  uint8_t security;
  uint8_t client_count;
  uint8_t mac[6];
  uint8_t reserved[2];
};

static inline uint8_t bk7258_wifi_cpdu_get_vif(uint8_t flags)
{
  return (flags & BK7258_WIFI_CPDU_VIF_MASK) >> BK7258_WIFI_CPDU_VIF_SHIFT;
}

static inline uint8_t bk7258_wifi_cpdu_set_vif(uint8_t flags, uint8_t vif)
{
  return (flags & (uint8_t)~BK7258_WIFI_CPDU_VIF_MASK) |
         ((vif << BK7258_WIFI_CPDU_VIF_SHIFT) & BK7258_WIFI_CPDU_VIF_MASK);
}

struct bk7258_wifi_scan_page_request
{
  uint16_t start;
  uint16_t max_records;
};

struct bk7258_wifi_scan_record
{
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t security;
  uint8_t reserved[6];
};

struct bk7258_wifi_scan_page
{
  uint16_t total;
  uint16_t start;
  uint8_t count;
  uint8_t more;
  uint16_t reserved;
  struct bk7258_wifi_scan_record records[BK7258_WIFI_SCAN_PAGE_RECORDS];
};

struct bk7258_wifi_scan_page_response
{
  int32_t status;
  struct bk7258_wifi_scan_page page;
};

struct bk7258_wifi_country
{
  char cc[3];
  uint8_t start_channel;
  uint8_t channel_count;
  int8_t max_tx_power;
  uint8_t policy;
  uint8_t reserved;
};

struct bk7258_wifi_country_response
{
  int32_t status;
  struct bk7258_wifi_country country;
};

/* This node overlays bk7258_mb_wire_message at offsets 0, 4, 8 and 12. */

struct bk7258_wifi_ipc_node
{
  uint32_t ipc_hdr;
  uint32_t head;
  uint32_t tail;
  uint8_t channel;
  uint8_t num;
  uint16_t reserved;
};

struct bk7258_wifi_cpdu
{
  uint32_t next;
  uint16_t length;
  uint8_t type_dst;
  uint8_t flags;
};

struct bk7258_wifi_pbuf
{
  uint32_t next;
  uint32_t payload;
  uint16_t tot_len;
  uint16_t len;
  uint8_t type_internal;
  uint8_t flags;
  uint8_t ref;
  uint8_t if_idx;
};

struct bk7258_wifi_msg_hdr
{
  uint32_t reserved0;
  uint16_t cmd_id;
  uint16_t cmd_sn;
  uint16_t reserved1;
  uint16_t length;
};

struct bk7258_wifi_event_hdr
{
  uint16_t id;
  uint16_t sequence;
  uint16_t reserved;
  uint16_t length;
  uint32_t pattern;
};

static_assert(sizeof(struct bk7258_wifi_ipc_node) == 16,
               "Wi-Fi IPC node ABI must be 16 bytes");
static_assert(offsetof(struct bk7258_wifi_ipc_node, head) == 4,
               "Wi-Fi IPC node head offset");
static_assert(offsetof(struct bk7258_wifi_ipc_node, tail) == 8,
               "Wi-Fi IPC node tail offset");
static_assert(offsetof(struct bk7258_wifi_ipc_node, channel) == 12,
               "Wi-Fi IPC node channel offset");
static_assert(sizeof(struct bk7258_wifi_cpdu) == 8,
               "Wi-Fi cpdu ABI must be 8 bytes");
static_assert(sizeof(struct bk7258_wifi_pbuf) == 16,
               "Wi-Fi pbuf ABI must be 16 bytes");
static_assert(offsetof(struct bk7258_wifi_pbuf, payload) == 4,
               "Wi-Fi pbuf payload offset");
static_assert(offsetof(struct bk7258_wifi_pbuf, ref) == 14,
               "Wi-Fi pbuf ref offset");
static_assert(sizeof(struct bk7258_wifi_msg_hdr) == 12,
               "Wi-Fi command header ABI must be 12 bytes");
static_assert(offsetof(struct bk7258_wifi_msg_hdr, cmd_id) == 4,
               "Wi-Fi command id offset");
static_assert(sizeof(struct bk7258_wifi_event_hdr) == 12,
               "Wi-Fi event header ABI must be 12 bytes");
static_assert(offsetof(struct bk7258_wifi_event_hdr, pattern) == 8,
               "Wi-Fi event pattern offset");
static_assert(sizeof(struct bk7258_wifi_scan_page_request) == 4,
               "Wi-Fi scan page request ABI must be 4 bytes");
static_assert(sizeof(struct bk7258_wifi_scan_record) == 48,
               "Wi-Fi scan record ABI must be 48 bytes");
static_assert(sizeof(struct bk7258_wifi_scan_page) == 200,
               "Wi-Fi scan page ABI must be 200 bytes");
static_assert(sizeof(struct bk7258_wifi_scan_page_response) == 204,
               "Wi-Fi scan response ABI must be 204 bytes");
static_assert(sizeof(struct bk7258_wifi_country) == 8,
               "Wi-Fi country ABI must be 8 bytes");
static_assert(sizeof(struct bk7258_wifi_country_response) == 12,
               "Wi-Fi country response ABI must be 12 bytes");
static_assert(sizeof(struct bk7258_wifi_ap_start_request) == 104,
               "Wi-Fi AP start request ABI must be 104 bytes");
static_assert(offsetof(struct bk7258_wifi_ap_start_request, ssid) == 8,
               "Wi-Fi AP SSID offset");
static_assert(offsetof(struct bk7258_wifi_ap_start_request, password) == 40,
               "Wi-Fi AP password offset");
static_assert(sizeof(struct bk7258_wifi_ap_status_response) == 16,
               "Wi-Fi AP status response ABI must be 16 bytes");
static_assert(offsetof(struct bk7258_wifi_ap_status_response, mac) == 8,
               "Wi-Fi AP MAC offset");

#endif
