/****************************************************************************
 * arch/arm/src/bk7258/bk7258_wifi_probe.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* SPDX-License-Identifier: Apache-2.0
 * Read-only controller-interface bring-up. Wire layout and buffer ownership
 * follow Armino app_ab and open-vela contest2026_264, commit 1d3549d4a1ca.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* The full driver owns this channel and supplies the probe entry point. */

#ifndef CONFIG_BK7258_WIFI
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hardware/bk7258_mbox.h"
#include "hardware/bk7258_wifi_probe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROBE_SLOTS 4
#define PROBE_TIMEOUT MSEC2TICK(2000)

struct aligned_data(32) probe_slot_s
{
  volatile uint32_t pattern;
  struct wifi_probe_cpdu_s cpdu;
  struct wifi_probe_request_s request;
  bool pending;
};

static struct probe_slot_s g_slots[PROBE_SLOTS];
static mutex_t g_lock = NXMUTEX_INITIALIZER;
static sem_t g_done = SEM_INITIALIZER(0);
static bool g_initialized;
static uint16_t g_sequence;
static uint16_t g_waiting;
static uint16_t g_expected;
static int g_result;
static uint8_t g_response[sizeof(struct wifi_probe_status_s)];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool probe_range(uint32_t address, size_t size)
{
  return address >= BK7258_CP_RAM_START &&
         address <= BK7258_CP_RAM_END &&
         size <= BK7258_CP_RAM_END - address;
}

static int probe_rx(const struct bk7258_mb_wire_message *message,
                    uint8_t *ack_flags, void *arg)
{
  struct wifi_probe_node_s node;
  struct wifi_probe_cpdu_s cpdu;
  struct wifi_probe_event_s event;
  uint32_t addresses[WIFI_PROBE_MAX_LIST];
  uint32_t address;
  unsigned int i;
  unsigned int j;
  bool wake = false;
  irqstate_t flags;

  (void)ack_flags;
  (void)arg;
  memcpy(&node, message, sizeof(node));
  if (node.channel != 0 || node.count == 0 ||
      node.count > WIFI_PROBE_MAX_LIST)
    {
      return -EPROTO;
    }

  flags = enter_critical_section();
  address = node.head;

  /* Validate the entire CP-owned chain before releasing any node. The CP
   * retains ownership until this callback returns its transport ACK.
   */

  for (i = 0; i < node.count; i++)
    {
      if ((address & 3u) != 0 || address < BK7258_CP_RAM_START + 4 ||
          !probe_range(address - 4, 4 + sizeof(cpdu) + sizeof(event)))
        {
          goto malformed;
        }

      for (j = 0; j < i; j++)
        {
          if (addresses[j] == address)
            {
              goto malformed;
            }
        }

      memcpy(&cpdu, (const void *)(uintptr_t)address, sizeof(cpdu));
      memcpy(&event, (const void *)(uintptr_t)(address + sizeof(cpdu)),
             sizeof(event));
      if (cpdu.length < sizeof(cpdu) + sizeof(event) ||
          !probe_range(address, cpdu.length) ||
          event.length > cpdu.length - sizeof(cpdu) - sizeof(event) ||
          ((i + 1 == node.count) != (address == node.tail)))
        {
          goto malformed;
        }

      addresses[i] = address;
      address = cpdu.next;
    }

  if (address != 0)
    {
      goto malformed;
    }

  for (i = 0; i < node.count; i++)
    {
      address = addresses[i];
      memcpy(&event, (const void *)(uintptr_t)(address + sizeof(cpdu)),
             sizeof(event));
      if (g_waiting != 0 && event.id == g_waiting &&
          event.sequence == g_sequence)
        {
          g_result = -EPROTO;
          if (event.length == g_expected)
            {
              memcpy(g_response, (const void *)(uintptr_t)
                     (address + sizeof(cpdu) + sizeof(event)), g_expected);
              g_result = OK;
            }

          g_waiting = 0;
          wake = true;
        }

      *(volatile uint32_t *)(uintptr_t)(address - 4) = WIFI_PROBE_FREE;
    }

  __atomic_thread_fence(__ATOMIC_SEQ_CST);
  leave_critical_section(flags);
  if (wake)
    {
      nxsem_post(&g_done);
    }

  return OK;

malformed:
  leave_critical_section(flags);
  return -EPROTO;
}

static void probe_tx_done(const struct bk7258_mb_wire_message *ack,
                          int result, void *arg)
{
  struct probe_slot_s *slot = arg;
  irqstate_t flags = enter_critical_section();
  bool wake = false;

  (void)ack;
  slot->pending = false;
  if (result == -EREMOTEIO)
    {
      slot->pattern = WIFI_PROBE_FREE;
    }

  if (result < 0 && g_waiting ==
      (slot->request.command | WIFI_PROBE_CONFIRM) &&
      g_sequence == slot->request.sequence)
    {
      g_result = result;
      g_waiting = 0;
      wake = true;
    }

  leave_critical_section(flags);
  if (wake)
    {
      nxsem_post(&g_done);
    }
}

static int probe_request(uint16_t command, void *response, size_t length)
{
  struct bk7258_mb_wire_message message;
  struct wifi_probe_node_s node =
    {
      0
    };

  struct probe_slot_s *slot = NULL;
  irqstate_t flags;
  unsigned int i;
  int ret;

  if (!bk7258_mailbox_link_ready())
    {
      return -ENOLINK;
    }

  if (g_sequence == UINT16_MAX || length > sizeof(g_response))
    {
      return -EOVERFLOW;
    }

  while (nxsem_trywait(&g_done) == OK)
    {
    }

  flags = enter_critical_section();
  for (i = 0; i < PROBE_SLOTS; i++)
    {
      if (!g_slots[i].pending && g_slots[i].pattern == WIFI_PROBE_FREE)
        {
          slot = &g_slots[i];
          break;
        }
    }

  if (slot == NULL)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  slot->pattern = WIFI_PROBE_BUSY;
  slot->pending = true;
  slot->cpdu.next = 0;
  slot->cpdu.length = sizeof(slot->cpdu) + sizeof(slot->request);
  memset(&slot->request, 0, sizeof(slot->request));
  slot->request.command = command;
  slot->request.sequence = ++g_sequence;
  g_waiting = command | WIFI_PROBE_CONFIRM;
  g_expected = length;
  g_result = -ETIMEDOUT;
  node.head = (uint32_t)(uintptr_t)&slot->cpdu;
  node.tail = node.head;
  node.count = 1;
  memcpy(&message, &node, sizeof(message));
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
  ret = bk7258_mailbox_send_wire(BK7258_MB_CHAN_WIFI_CMD_TX, &message,
                                 probe_tx_done, slot);
  if (ret < 0)
    {
      slot->pending = false;
      slot->pattern = WIFI_PROBE_FREE;
      g_waiting = 0;
    }

  leave_critical_section(flags);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_tickwait_uninterruptible(&g_done, PROBE_TIMEOUT);
  flags = enter_critical_section();
  if (ret >= 0)
    {
      ret = g_result;
      if (ret == OK)
        {
          memcpy(response, g_response, length);
        }
    }

  /* A timeout does not prove CP released this request. Keep the static
   * slot quarantined until both transport completion and CP FREE appear.
   */

  g_waiting = 0;
  leave_critical_section(flags);
  return ret;
}

int bk7258_wifi_probe(void)
{
  struct wifi_probe_status_s status;
  uint8_t mac[6];
  unsigned int i;
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!g_initialized)
    {
      for (i = 0; i < PROBE_SLOTS; i++)
        {
          g_slots[i].pattern = WIFI_PROBE_FREE;
        }

      ret = bk7258_mailbox_register_rx(BK7258_MB_CHAN_WIFI_CMD_RX,
                                       probe_rx, NULL);
      if (ret < 0)
        {
          goto out;
        }

      g_initialized = true;
    }

  printf("Wi-Fi probe: v14 CP command channel (read-only)\n");
  ret = probe_request(WIFI_PROBE_GET_MAC, mac, sizeof(mac));
  if (ret < 0)
    {
      printf("Wi-Fi GET_MAC failed: %d\n", ret);
      goto out;
    }

  if ((mac[0] & 1) != 0 ||
      (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]) == 0)
    {
      ret = -EPROTO;
      printf("Wi-Fi GET_MAC returned an invalid address\n");
      goto out;
    }

  printf("CP MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ret = probe_request(WIFI_PROBE_GET_STATUS, &status, sizeof(status));
  if (ret < 0)
    {
      printf("Wi-Fi GET_STATUS failed: %d\n", ret);
      goto out;
    }

  printf("CP state=%u RSSI=%d SSID=%.33s\n", status.state,
         status.rssi, status.ssid);
  printf("CP IPv4=%.16s mask=%.16s gateway=%.16s DNS=%.16s\n",
         status.ip, status.mask, status.gateway, status.dns);
  printf("Wi-Fi command probe passed; AP netdev/DHCP not implemented\n");

out:
  nxmutex_unlock(&g_lock);
  return ret;
}

#endif /* !CONFIG_BK7258_WIFI */
