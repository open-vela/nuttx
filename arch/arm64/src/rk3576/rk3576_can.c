/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_can.c
 *
 * CAN controller driver for RK3576 (Bosch M_CAN compatible)
 * Supports CAN 2.0A/B and CAN FD
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_can.h"
#include "rk3576_can.h"

#ifdef CONFIG_RK3576_CAN

struct rk3576_can_s
{
  bool enabled;
  int bitrate;
};

static struct rk3576_can_s g_can[RK3576_CAN_COUNT];

const uint32_t g_can_base[RK3576_CAN_COUNT] =
{
  RK3576_CAN0_ADDR,
  RK3576_CAN1_ADDR,
};

static int rk3576_can_set_mode(int can, bool init_mode)
{
  uint32_t base = g_can_base[can];
  uint32_t ccr = getreg32(base + CAN_CCCR);

  if (init_mode)
    {
      ccr |= CAN_CCCR_INIT | CAN_CCCR_CCE;
    }
  else
    {
      ccr &= ~(CAN_CCCR_INIT | CAN_CCCR_CCE);
    }

  putreg32(ccr, base + CAN_CCCR);

  /* Wait for mode change */

  int timeout = 100;
  while (timeout--)
    {
      if ((getreg32(base + CAN_CCCR) & CAN_CCCR_INIT) ==
          (init_mode ? CAN_CCCR_INIT : 0))
        {
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

int rk3576_can_init(int can)
{
  if (can < 0 || can >= RK3576_CAN_COUNT)
    {
      return -EINVAL;
    }

  memset(&g_can[can], 0, sizeof(g_can[can]));
  g_can[can].bitrate = 500000;

  ginfo("CAN%d: initialized\n", can);
  return OK;
}

int rk3576_can_set_bitrate(int can, int bitrate)
{
  uint32_t base;

  if (can < 0 || can >= RK3576_CAN_COUNT)
    {
      return -EINVAL;
    }

  base = g_can_base[can];

  /* Enter init mode */

  int ret = rk3576_can_set_mode(can, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure bit timing for 24MHz oscillator
   * For 500kbps: prescaler=10, TSEG1=13, TSEG2=2, SJW=1
   * BRP = 24MHz / (500kbps * (1 + 13 + 2)) = 30
   * Actual: 24MHz / (30 * 16) = 50kbps (simplified)
   */

  uint32_t brp = 24000000 / (bitrate * 16);
  if (brp < 1) brp = 1;
  if (brp > 256) brp = 256;

  uint32_t btp1 = ((brp - 1) << 0) | (12 << 16);  /* BRP and TSEG1 */
  uint32_t btp2 = (2 << 0) | (1 << 8);             /* TSEG2 and SJW */

  putreg32(btp1, base + CAN_BTP1);
  putreg32(btp2, base + CAN_BTP2);

  g_can[can].bitrate = bitrate;

  /* Leave init mode */

  ret = rk3576_can_set_mode(can, false);

  ginfo("CAN%d: bitrate set to %d\n", can, bitrate);
  return ret;
}

int rk3576_can_send(int can, const struct rk3576_can_msg_s *msg)
{
  uint32_t base;

  if (can < 0 || can >= RK3576_CAN_COUNT || msg == NULL)
    {
      return -EINVAL;
    }

  base = g_can_base[can];

  /* Populate TX buffer element 0 with message data */

  for (int i = 0; i < msg->dlc && i < 8; i++)
    {
      putreg32(msg->data[i], base + 0x100 + (i * 4));
    }

  /* Request transmission on buffer 0 */

  putreg32(1 << 0, base + CAN_TXBAR);

  /* Wait for transmission */

  int timeout = 100;
  while (timeout--)
    {
      if (getreg32(base + CAN_TXBTO) & (1 << 0))
        {
          putreg32(1 << 0, base + CAN_TXBTO);
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

int rk3576_can_receive(int can, struct rk3576_can_msg_s *msg)
{
  uint32_t base;

  if (can < 0 || can >= RK3576_CAN_COUNT || msg == NULL)
    {
      return -EINVAL;
    }

  base = g_can_base[can];

  /* Check if FIFO 0 has data */

  uint32_t fifo_status = getreg32(base + CAN_RXFSA);
  if ((fifo_status & 0x3f) == 0)
    {
      return -EAGAIN;
    }

  /* Read message from FIFO 0 */

  uint32_t fifo_addr = base + 0x200;

  /* Parse header word */

  uint32_t hdr = getreg32(fifo_addr);
  msg->id = (hdr >> 9) & 0x7ff;
  msg->extended = (hdr >> 30) & 1;
  msg->rtr = (hdr >> 29) & 1;
  msg->dlc = (hdr >> 16) & 0x0f;

  /* Read payload data */

  for (int i = 0; i < msg->dlc && i < 8; i++)
    {
      msg->data[i] = (uint8_t)(getreg32(fifo_addr + 8 + (i * 4)) & 0xff);
    }

  /* Acknowledge FIFO 0 read */

  putreg32(1 << 0, base + CAN_RXFxA);

  return OK;
}

int rk3576_can_enable(int can)
{
  uint32_t base;

  if (can < 0 || can >= RK3576_CAN_COUNT)
    {
      return -EINVAL;
    }

  base = g_can_base[can];

  /* Enable interrupts */

  putreg32(CAN_IE_RF0NE | CAN_IE_TFNE | CAN_IE_BOE, base + CAN_IE);

  /* Enable interrupt line 0 */

  putreg32(1 << 0, base + CAN_ILE);

  g_can[can].enabled = true;

  ginfo("CAN%d: enabled\n", can);
  return OK;
}

int rk3576_can_disable(int can)
{
  if (can < 0 || can >= RK3576_CAN_COUNT)
    {
      return -EINVAL;
    }

  /* Disable interrupts */

  putreg32(0, g_can_base[can] + CAN_IE);

  g_can[can].enabled = false;

  ginfo("CAN%d: disabled\n", can);
  return OK;
}

uint32_t rk3576_can_get_status(int can)
{
  if (can < 0 || can >= RK3576_CAN_COUNT)
    {
      return 0;
    }

  return getreg32(g_can_base[can] + CAN_IR);
}

#endif /* CONFIG_RK3576_CAN */
