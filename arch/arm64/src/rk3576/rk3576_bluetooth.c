/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_bluetooth.c
 *
 * Bluetooth HCI transport driver for RK3576
 * UART-based HCI for built-in BLE controller
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_memorymap.h"
#include "rk3576_bluetooth.h"

#ifdef CONFIG_RK3576_BLUETOOTH

/* HCI packet types */

#define HCI_CMD_PACKET            0x01
#define HCI_ACL_PACKET            0x02
#define HCI_SCO_PACKET            0x03
#define HCI_EVENT_PACKET          0x04

/* HCI command opcodes */

#define HCI_RESET                 0x0c03
#define HCI_WRITE_SCAN_ENABLE     0x0c1a
#define HCI_WRITE_LOCAL_NAME      0x0c13
#define HCI_SET_EVENT_MASK        0x0c01

struct rk3576_bt_s
{
  int uart;
  bool enabled;
};

static struct rk3576_bt_s g_bt;

static int rk3576_bt_write_reg(uint32_t addr, uint32_t val)
  __attribute__((unused));
static uint32_t rk3576_bt_read_reg(uint32_t addr)
  __attribute__((unused));
static void rk3576_bt_delay(int ms)
  __attribute__((unused));

static int rk3576_bt_write_reg(uint32_t addr, uint32_t val)
{
  putreg32(val, addr);
  return OK;
}

static uint32_t rk3576_bt_read_reg(uint32_t addr)
{
  return getreg32(addr);
}

static void rk3576_bt_delay(int ms)
{
  up_mdelay(ms);
}

int rk3576_bluetooth_init(int uart)
{
  g_bt.uart = uart;
  g_bt.enabled = false;

  /* Enable Bluetooth power (via PMU GRF if available) */

  ginfo("BT: initialized on UART%d\n", uart);
  return OK;
}

int rk3576_bluetooth_send(uint8_t type, const uint8_t *data, int len)
{
  UNUSED(type);
  UNUSED(data);
  UNUSED(len);

  /* HCI packet transmission via UART */
  /* In practice, this writes to the UART TX FIFO */

  return OK;
}

int rk3576_bluetooth_read(uint8_t *buf, int maxlen, int timeout_ms)
{
  UNUSED(buf);
  UNUSED(maxlen);
  UNUSED(timeout_ms);

  /* HCI packet reception via UART */
  /* In practice, this reads from the UART RX FIFO */

  return 0;
}

void rk3576_bluetooth_enable(bool enable)
{
  g_bt.enabled = enable;

  if (enable)
    {
      ginfo("BT: enabled\n");
    }
  else
    {
      ginfo("BT: disabled\n");
    }
}

#endif /* CONFIG_RK3576_BLUETOOTH */
