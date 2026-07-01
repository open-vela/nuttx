/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_cec.c
 *
 * HDMI CEC (Consumer Electronics Control) driver for RK3576
 * Supports CEC 1.4 and HDMI 2.0 CEC features
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_memorymap.h"
#include "rk3576_cec.h"

#ifdef CONFIG_RK3576_CEC

#define CEC_BASE                (RK3576_CEC_ADDR + 0x400)

#define CEC_CTRL                0x0000
#define CEC_STATUS              0x0004
#define CEC_INT_MASK            0x0008
#define CEC_INT_CLR             0x000c
#define CEC_TX_HEADER           0x0010
#define CEC_TX_DATA             0x0014
#define CEC_TX_CTRL             0x0018
#define CEC_RX_HEADER           0x0020
#define CEC_RX_DATA             0x0024
#define CEC_RX_STATUS           0x0028
#define CEC_LOG_ADDR            0x0030

/* CEC_CTRL bits */

#define CEC_CTRL_ENABLE         (1 << 0)
#define CEC_CTRL_TX_START       (1 << 1)
#define CEC_CTRL_RX_START       (1 << 2)

/* CEC_TX_CTRL bits */

#define CEC_TX_CTRL_LEN_SHIFT   0
#define CEC_TX_CTRL_LEN_MASK    (0xf << 0)
#define CEC_TX_CTRL_RETRY_SHIFT 4
#define CEC_TX_CTRL_RETRY_MASK  (0x3 << 4)

/* CEC_INT bits */

#define CEC_INT_TX_DONE         (1 << 0)
#define CEC_INT_TX_ERROR        (1 << 1)
#define CEC_INT_RX_DONE         (1 << 2)
#define CEC_INT_RX_ERROR        (1 << 3)

struct rk3576_cec_s
{
  bool enabled;
  uint8_t logical_addr;
};

static struct rk3576_cec_s g_cec;

int rk3576_cec_init(void)
{
  /* Disable CEC */

  putreg32(0, CEC_BASE + CEC_CTRL);

  /* Clear all interrupts */

  putreg32(0xffffffff, CEC_BASE + CEC_INT_CLR);

  /* Set default logical address (0 = TV) */

  putreg32(0x00, CEC_BASE + CEC_LOG_ADDR);

  memset(&g_cec, 0, sizeof(g_cec));
  g_cec.logical_addr = 0;

  ginfo("CEC: initialized (logical addr=0)\n");
  return OK;
}

int rk3576_cec_enable(void)
{
  putreg32(CEC_CTRL_ENABLE, CEC_BASE + CEC_CTRL);

  g_cec.enabled = true;

  ginfo("CEC: enabled\n");
  return OK;
}

int rk3576_cec_disable(void)
{
  putreg32(0, CEC_BASE + CEC_CTRL);

  g_cec.enabled = false;

  ginfo("CEC: disabled\n");
  return OK;
}

int rk3576_cec_send(const uint8_t *data, int len)
{
  uint32_t status;
  int timeout;

  if (!g_cec.enabled || data == NULL || len < 1 || len > 16)
    {
      return -EINVAL;
    }

  /* Set header (initiator address | destination) */

  putreg32(data[0], CEC_BASE + CEC_TX_HEADER);

  /* Write payload data */

  for (int i = 1; i < len; i++)
    {
      putreg32(data[i], CEC_BASE + CEC_TX_DATA + ((i - 1) * 4));
    }

  /* Set transfer length and start transmission */

  putreg32(((len - 1) << CEC_TX_CTRL_LEN_SHIFT) |
           (3 << CEC_TX_CTRL_RETRY_SHIFT),
           CEC_BASE + CEC_TX_CTRL);

  putreg32(CEC_CTRL_ENABLE | CEC_CTRL_TX_START, CEC_BASE + CEC_CTRL);

  /* Wait for transmission complete */

  for (timeout = 0; timeout < 100; timeout++)
    {
      status = getreg32(CEC_BASE + CEC_STATUS);
      if (status & CEC_INT_TX_DONE)
        {
          putreg32(CEC_INT_TX_DONE, CEC_BASE + CEC_INT_CLR);
          return OK;
        }

      if (status & CEC_INT_TX_ERROR)
        {
          putreg32(CEC_INT_TX_ERROR, CEC_BASE + CEC_INT_CLR);
          return -EIO;
        }

      up_udelay(100);
    }

  return -ETIMEDOUT;
}

int rk3576_cec_receive(uint8_t *data, int maxlen)
{
  uint32_t status;
  int len;

  if (!g_cec.enabled || data == NULL || maxlen < 1)
    {
      return -EINVAL;
    }

  /* Check if data is available */

  status = getreg32(CEC_BASE + CEC_STATUS);
  if (!(status & CEC_INT_RX_DONE))
    {
      return -EAGAIN;
    }

  /* Read header */

  data[0] = (uint8_t)(getreg32(CEC_BASE + CEC_RX_HEADER) & 0xff);

  /* Read payload */

  len = (getreg32(CEC_BASE + CEC_RX_STATUS) & 0xf) + 1;
  if (len > maxlen)
    {
      len = maxlen;
    }

  for (int i = 1; i < len; i++)
    {
      data[i] = (uint8_t)(getreg32(CEC_BASE + CEC_RX_DATA +
                            ((i - 1) * 4)) & 0xff);
    }

  /* Clear RX done interrupt */

  putreg32(CEC_INT_RX_DONE, CEC_BASE + CEC_INT_CLR);

  return len;
}

#endif /* CONFIG_RK3576_CEC */
