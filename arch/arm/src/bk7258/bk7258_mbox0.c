/****************************************************************************
 * arch/arm/src/bk7258/bk7258_mbox0.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 MBOX0 v2 physical transport.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/bk7258_mbox.h"
#include "hardware/bk7258_sysctrl.h"
#include "irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MBOX_RX_DESC_COUNT      (2u * 3u + 1u)
#define MBOX_IRQ_DRAIN_BUDGET   8u

struct mbox_rx_descriptor
{
  struct bk7258_mb_wire_message message;
  uint32_t order;
  bool used;
};

static bk7258_mbox_callback_t g_callback;
static struct mbox_rx_descriptor g_rx_desc[MBOX_RX_DESC_COUNT];
static uint32_t g_rx_order;
static uint32_t g_processing_order;
static bool g_mbox_ready;
static struct bk7258_mbox_stats g_stats;

static_assert(MBOX_RX_DESC_COUNT > 2u * 3u,
               "RX descriptors must hold a deferred command and two FIFOs");

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t bk7258_mbox_rx_status(void)
{
  return getreg32(BK7258_MBOX_CH1_STATUS);
}

static void mbox_clear_errors(void)
{
  uint32_t config = getreg32(BK7258_MBOX_CH1_CFG);
  uint32_t errors = config & BK7258_MBOX_CFG_ERROR_STATUS;

  if ((errors & BK7258_MBOX_CFG_WRERR_STATUS) != 0)
    {
      g_stats.write_error++;
    }

  if ((errors & BK7258_MBOX_CFG_RDERR_STATUS) != 0)
    {
      g_stats.read_error++;
    }

  if ((errors & BK7258_MBOX_CFG_WRFULL_STATUS) != 0)
    {
      g_stats.write_full++;
    }

  if (errors != 0)
    {
      putreg32((config & BK7258_MBOX_CFG_RW_MASK) | errors,
               BK7258_MBOX_CH1_CFG);
    }
}

static bool mbox_valid_envelope(uint8_t source, uint32_t address,
                                uint32_t length)
{
  if (source != 0)
    {
      g_stats.bad_source++;
      return false;
    }

  if (length != BK7258_MB_MESSAGE_SIZE)
    {
      g_stats.bad_length++;
      return false;
    }

  if ((address & 3u) != 0 || address < BK7258_CP_RAM_START ||
      address > BK7258_CP_RAM_END - BK7258_MB_MESSAGE_SIZE)
    {
      g_stats.bad_address++;
      return false;
    }

  return true;
}

int bk7258_mbox_send(uint8_t destination, const uint32_t data[2])
{
  irqstate_t flags;
  uintptr_t status;

  if (!g_mbox_ready || destination > 1 || data == NULL)
    {
      return -EINVAL;
    }

  /* CPU1 transmits through channel 1.  Full is reported by the target's
   * receive FIFO status register.
   */

  status = destination == 0 ? BK7258_MBOX_CH0_STATUS :
                              BK7258_MBOX_CH1_STATUS;
  flags = up_irq_save();
  if ((getreg32(status) & 1u) != 0)
    {
      up_irq_restore(flags);
      return -EAGAIN;
    }

  /* TDATA0/TDATA1 are staging registers and TID commits the descriptor.
   * Keep concurrent transport sends from interleaving these writes.
   */

  putreg32(data[0], BK7258_MBOX_CH1_TDATA0);
  putreg32(data[1], BK7258_MBOX_CH1_TDATA1);
  putreg32(destination, BK7258_MBOX_CH1_TID);
  up_irq_restore(flags);
  return OK;
}

static struct mbox_rx_descriptor *mbox_alloc_desc(void)
{
  unsigned int i;

  for (i = 0; i < MBOX_RX_DESC_COUNT; i++)
    {
      if (!g_rx_desc[i].used)
        {
          return &g_rx_desc[i];
        }
    }

  return NULL;
}

static bool mbox_priority_message(
  const struct bk7258_mb_wire_message *message)
{
  uint8_t control = bk7258_mb_header_ctrl(message);

  return (control & (BK7258_MB_CTRL_ACK_BOX | BK7258_MB_CTRL_RESET)) != 0;
}

static struct mbox_rx_descriptor *mbox_oldest_desc(bool priority)
{
  struct mbox_rx_descriptor *oldest = NULL;
  unsigned int i;

  for (i = 0; i < MBOX_RX_DESC_COUNT; i++)
    {
      struct mbox_rx_descriptor *desc = &g_rx_desc[i];

      if (!desc->used || mbox_priority_message(&desc->message) != priority)
        {
          continue;
        }

      if (oldest == NULL || (int32_t)(desc->order - oldest->order) < 0)
        {
          oldest = desc;
        }
    }

  return oldest;
}

static void mbox_process_desc(void)
{
  struct mbox_rx_descriptor *desc;
  int ret;

  /* ACK and RESET never need ACK slots.  Process them ahead of a deferred
   * ordinary command, then retry ordinary commands in original FIFO order.
   */

  while ((desc = mbox_oldest_desc(true)) != NULL)
    {
      g_processing_order = desc->order;
      ret = g_callback == NULL ? -ENOSYS : g_callback(&desc->message);
      g_processing_order = 0;
      if (ret == -EAGAIN)
        {
          g_stats.descriptor_deferred++;
          break;
        }

      desc->used = false;
    }

  while ((desc = mbox_oldest_desc(false)) != NULL)
    {
      g_processing_order = desc->order;
      ret = g_callback == NULL ? -ENOSYS : g_callback(&desc->message);
      g_processing_order = 0;
      if (ret == -EAGAIN)
        {
          break;
        }

      desc->used = false;
    }
}

static int bk7258_mbox_irq(int irq, void *context, void *arg)
{
  uintptr_t rdata0;
  uintptr_t rdata1;
  uintptr_t sidreg;
  unsigned int budget = MBOX_IRQ_DRAIN_BUDGET;

  (void)irq;
  (void)context;
  (void)arg;

  mbox_clear_errors();
  sidreg = BK7258_MBOX_CH1_SID;
  rdata0 = BK7258_MBOX_CH1_RDATA0;
  rdata1 = BK7258_MBOX_CH1_RDATA1;

  while ((bk7258_mbox_rx_status() & 2u) == 0 && budget-- != 0)
    {
      struct mbox_rx_descriptor *desc;
      uint32_t address;
      uint32_t length;
      uint8_t source;

      mbox_process_desc();

      desc = mbox_alloc_desc();
      if (desc == NULL)
        {
          g_stats.descriptor_full++;
          break;
        }

      source = getreg32(sidreg) & 0x0fu;
      address = getreg32(rdata0);
      length = getreg32(rdata1);
      if (!mbox_valid_envelope(source, address, length))
        {
          continue;
        }

      __asm__ volatile("dmb sy" ::: "memory");
      memcpy(&desc->message, (const void *)(uintptr_t)address,
             sizeof(desc->message));
      desc->order = ++g_rx_order;
      desc->used = true;
      g_stats.rx_messages++;
      mbox_process_desc();
    }

  mbox_process_desc();

  return OK;
}

void bk7258_mbox_set_callback(bk7258_mbox_callback_t callback)
{
  g_callback = callback;
}

void bk7258_mbox_kick_rx(void)
{
  irqstate_t flags = up_irq_save();

  (void)bk7258_mbox_irq(0, NULL, NULL);
  up_irq_restore(flags);
}

void bk7258_mbox_discard_deferred(void)
{
  unsigned int i;

  for (i = 0; i < MBOX_RX_DESC_COUNT; i++)
    {
      if (g_rx_desc[i].used && g_processing_order != 0 &&
          (int32_t)(g_rx_desc[i].order - g_processing_order) < 0 &&
          !mbox_priority_message(&g_rx_desc[i].message))
        {
          g_rx_desc[i].used = false;
        }
    }
}

void bk7258_mbox_get_stats(struct bk7258_mbox_stats *stats)
{
  irqstate_t flags;

  if (stats == NULL)
    {
      return;
    }

  flags = up_irq_save();
  *stats = g_stats;
  up_irq_restore(flags);
}

int bk7258_mbox_init(void)
{
  int ret;

  if (g_mbox_ready)
    {
      return OK;
    }

  memset(g_rx_desc, 0, sizeof(g_rx_desc));
  g_rx_order = 0;
  g_processing_order = 0;
  memset(&g_stats, 0, sizeof(g_stats));

  /* CPU1 owns channel 1 and FIFO entries 2..4.  Install the callback and
   * descriptors before enabling the route; pre-existing FIFO entries are
   * then processed normally instead of being blindly discarded.
   */

  putreg32(2u | BK7258_MBOX_CFG_INT_EN | BK7258_MBOX_CFG_WRERR_EN |
            BK7258_MBOX_CFG_RDERR_EN | BK7258_MBOX_CFG_WRFULL_EN,
            BK7258_MBOX_CH1_CFG);
  putreg32(1u | (3u << 1), BK7258_MBOX_CH1_FIFO_CFG);
  modifyreg32(BK7258_MBOX_CTRL, 0, 1u << 2);
  mbox_clear_errors();

  ret = irq_attach(BK7258_IRQ_MAILBOX, bk7258_mbox_irq, NULL);
  if (ret < 0)
    {
      return ret;
    }

  g_mbox_ready = true;
  modifyreg32(BK7258_CPU1_IRQ_EN1, 0, 1u << 31);
  up_enable_irq(BK7258_IRQ_MAILBOX);
  return OK;
}
