/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_hci_rx_snapshot.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_HCI_RX_SNAPSHOT_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_HCI_RX_SNAPSHOT_H

#include <stdint.h>

struct bt_hci_rx_ring_snapshot_s
{
  uint32_t ring_read_mirror;
  uint32_t ring_write_mirror;
  uint32_t local_read_mirror;
  uint32_t mailbox_count;
  uint32_t worker_count;
  uint32_t drain_count;
  uint32_t copied_bytes;
  uint32_t callback_count;
  uint32_t complete_h4_count;
  uint32_t forwarded_h4_count;
  uint32_t sequence;
  uint16_t last_opcode;
  uint8_t last_h4_type;
  uint8_t last_event;
};

int bt_hci_rx_ring_snapshot(struct bt_hci_rx_ring_snapshot_s *snapshot);

#endif
