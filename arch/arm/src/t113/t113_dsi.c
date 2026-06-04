/****************************************************************************
 * arch/arm/src/t113/t113_dsi.c
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

/* T113-S3 MIPI DSI host driver.
 *
 * Implements clock + DPHY bring-up and an LP-mode command path: DCS short
 * write, DCS long write, DCS short read.  LP transfer completion is
 * signalled by the DSI host's INSTR_END interrupt via a per-host
 * semaphore.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/mipi_dsi.h>
#include <nuttx/video/mipi_display.h>

#include "arm_internal.h"
#include "hardware/t113_dsi.h"
#include "t113_clk.h"
#include "t113_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Phase 2 hard-codes 2 lanes to match the X4B panel.  See note in
 * t113_dsi_phy_init().
 */

#define T113_DSI_LANES           2
#define T113_DSI_LANE_MASK       ((1u << T113_DSI_LANES) - 1u)

/* CMD_TX FIFO is 64 32-bit entries = 256 bytes total. */

#define T113_DSI_TX_FIFO_BYTES   256

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The embedded mipi_dsi_host MUST be the first member so that a pointer
 * returned to the framework can be cast back to t113_dsi_dev_s with a
 * simple container_of() in later tasks.
 */

struct t113_dsi_dev_s
{
  struct mipi_dsi_host host;        /* Framework handle (must stay first) */
  bool                 initialized; /* One-time guard for init path */
  sem_t                done_sem;    /* Posted by ISR on INSTR_END */

  /* CMD_CTL captured pre-W1C in lp_run; lets the caller see RX_FLAG /
   * TX_FLAG as they were when the sequencer ended, not after lp_run
   * cleared them.
   */

  uint32_t             last_cmd_ctl;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_dsi_attach(FAR struct mipi_dsi_host *host,
                           FAR struct mipi_dsi_device *device);
static int t113_dsi_detach(FAR struct mipi_dsi_host *host,
                           FAR struct mipi_dsi_device *device);
static ssize_t t113_dsi_transfer(FAR struct mipi_dsi_host *host,
                                 FAR const struct mipi_dsi_msg *msg);

static int      t113_dsi_isr(int irq, FAR void *context, FAR void *arg);
static uint8_t  t113_dsi_ecc(uint32_t header24);
static uint16_t t113_dsi_crc(FAR const uint8_t *p, size_t n);
static uint16_t t113_dsi_crc_zeros(uint32_t n);
static int      t113_dsi_inst_init(uint32_t lane_mask);
static int      t113_dsi_lp_run(bool with_rx);
static void     t113_dsi_wait_inst_idle(void);
static ssize_t  t113_dsi_short_write(FAR const struct mipi_dsi_msg *msg);
static ssize_t  t113_dsi_long_write(FAR const struct mipi_dsi_msg *msg);
static ssize_t  t113_dsi_read(FAR const struct mipi_dsi_msg *msg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mipi_dsi_host_ops g_t113_dsi_host_ops =
{
  .attach   = t113_dsi_attach,
  .detach   = t113_dsi_detach,
  .transfer = t113_dsi_transfer,
};

static struct t113_dsi_dev_s g_t113_dsi;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_dsi_attach
 *
 * Description:
 *   Attach a peripheral to this host.  Real bring-up sequencing lives in
 *   the panel driver and the board glue; the host has nothing to do here.
 *
 ****************************************************************************/

static int t113_dsi_attach(FAR struct mipi_dsi_host *host,
                           FAR struct mipi_dsi_device *device)
{
  UNUSED(host);
  UNUSED(device);
  lcdinfo("attach\n");
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_detach
 ****************************************************************************/

static int t113_dsi_detach(FAR struct mipi_dsi_host *host,
                           FAR struct mipi_dsi_device *device)
{
  UNUSED(host);
  UNUSED(device);
  lcdinfo("detach\n");
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_isr
 *
 * Description:
 *   DSI host top-half.  Reads GINT0, write-1-clears every flag bit that was
 *   set, and posts the LP done semaphore on INSTR_END.  The other source
 *   bits are ack'd but otherwise ignored at this stage of bring-up.
 *
 ****************************************************************************/

static int t113_dsi_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct t113_dsi_dev_s *priv = (FAR struct t113_dsi_dev_s *)arg;
  uint32_t gint0;
  uint32_t flag_bits;

  UNUSED(irq);
  UNUSED(context);

  gint0     = t113_dsi_getreg(T113_DSI_GINT0_OFFSET);
  flag_bits = (gint0 & T113_DSI_GINT0_IRQ_FLAG_MASK) >>
              (T113_DSI_GINT0_IRQ_FLAG_SHIFT - T113_DSI_GINT0_IRQ_EN_SHIFT);

  if (gint0 & T113_DSI_GINT0_IRQ_FLAG_MASK)
    {
      /* W1C: write the flag bits back at their flag-field position. */

      t113_dsi_putreg(T113_DSI_GINT0_OFFSET,
                      gint0 & T113_DSI_GINT0_IRQ_FLAG_MASK);
    }

  if ((flag_bits & T113_DSI_GINT0_SRC_INSTR_END) && priv != NULL)
    {
      nxsem_post(&priv->done_sem);
    }

  return OK;
}

/****************************************************************************
 * Name: t113_dsi_ecc
 *
 * Description:
 *   Hamming(26,24) parity over the 24-bit packet header, as defined in the
 *   MIPI Alliance DSI specification.  Same algorithm in every DSI host.
 *
 ****************************************************************************/

static uint8_t t113_dsi_ecc(uint32_t header24)
{
  uint8_t  d[24];
  uint8_t  ecc = 0;
  unsigned i;

  for (i = 0; i < 24; i++)
    {
      d[i] = (header24 >> i) & 1u;
    }

  ecc |= (d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[10] ^ d[11] ^ d[13] ^
          d[16] ^ d[20] ^ d[21] ^ d[22] ^ d[23]) << 0;
  ecc |= (d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10] ^ d[12] ^ d[14] ^
          d[17] ^ d[20] ^ d[21] ^ d[22] ^ d[23]) << 1;
  ecc |= (d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[11] ^ d[12] ^ d[15] ^
          d[18] ^ d[20] ^ d[21] ^ d[22]) << 2;
  ecc |= (d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^
          d[19] ^ d[20] ^ d[21] ^ d[23]) << 3;
  ecc |= (d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[16] ^ d[17] ^ d[18] ^
          d[19] ^ d[20] ^ d[22] ^ d[23]) << 4;
  ecc |= (d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^
          d[18] ^ d[19] ^ d[21] ^ d[22] ^ d[23]) << 5;
  return ecc;
}

/****************************************************************************
 * Name: t113_dsi_crc
 *
 * Description:
 *   CRC-16/MCRF4XX over the long-packet payload: poly 0x1021, init 0xffff,
 *   reflected input/output, no final XOR.  Bit-serial implementation -
 *   long packets are tens of bytes, table-lookup wins nothing here.
 *
 ****************************************************************************/

static uint16_t t113_dsi_crc(FAR const uint8_t *p, size_t n)
{
  uint16_t crc = 0xffff;
  size_t   i;
  unsigned b;

  for (i = 0; i < n; i++)
    {
      crc ^= (uint16_t)p[i];
      for (b = 0; b < 8; b++)
        {
          if (crc & 1u)
            {
              crc = (crc >> 1) ^ 0x8408;  /* reflected 0x1021 */
            }
          else
            {
              crc >>= 1;
            }
        }
    }

  return crc;
}

/****************************************************************************
 * Name: t113_dsi_crc_zeros
 *
 * Description:
 *   CRC-16/MCRF4XX over n zero bytes.  Equivalent to t113_dsi_crc() with a
 *   buffer of zeros, but spares the few-KB scratch buffer that a blanking
 *   long packet would otherwise need on the stack.  Each XOR with 0x00 is
 *   a no-op so only the polynomial shifts run.
 *
 ****************************************************************************/

static uint16_t t113_dsi_crc_zeros(uint32_t n)
{
  uint16_t crc = 0xffff;
  unsigned b;

  while (n--)
    {
      for (b = 0; b < 8; b++)
        {
          if (crc & 1u)
            {
              crc = (crc >> 1) ^ 0x8408;
            }
          else
            {
              crc >>= 1;
            }
        }
    }

  return crc;
}

/****************************************************************************
 * Name: t113_dsi_inst_init
 *
 * Description:
 *   Program the instruction-sequencer slots needed for LP-mode command
 *   transfers (LP11, TBA, LPDT, DLY) and enable the DSI host.  HS pixel-
 *   mode slots are configured later by the TCON/DE bring-up.
 *
 ****************************************************************************/

static int t113_dsi_inst_init(uint32_t lane_mask)
{
  uint32_t v;

  /* LP11: drive data lanes to LP-11 stop, leave CK lane untouched so HSC
   * continuous (set up by t113_dsi_start_clk) survives LP DCS bursts.
   * Vendor BL dump confirms INST_FUNC[0] = 0x00000003 (LANE_CEN cleared).
   */

  v  = (T113_DSI_MODE_STOP << T113_DSI_INST_MODE_SHIFT) &
       T113_DSI_INST_MODE_MASK;
  v |= (lane_mask << T113_DSI_INST_LANE_DEN_SHIFT) &
       T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_LP11), v);

  /* TBA: bus turnaround on lane 0 only (read response). */

  v  = (T113_DSI_MODE_TBA << T113_DSI_INST_MODE_SHIFT) &
       T113_DSI_INST_MODE_MASK;
  v |= (1u << T113_DSI_INST_LANE_DEN_SHIFT) & T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_TBA), v);

  /* LPDT: low-power data transmission via escape mode on lane 0. */

  v  = (T113_DSI_MODE_ESCAPE << T113_DSI_INST_MODE_SHIFT) &
       T113_DSI_INST_MODE_MASK;
  v |= (T113_DSI_ESCA_LPDT << T113_DSI_INST_ESCAPE_ENTRY_SHIFT) &
       T113_DSI_INST_ESCAPE_ENTRY_MASK;
  v |= (T113_DSI_PACK_COMMAND << T113_DSI_INST_TRANS_PACK_SHIFT) &
       T113_DSI_INST_TRANS_PACK_MASK;
  v |= (1u << T113_DSI_INST_LANE_DEN_SHIFT) & T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_LPDT), v);

  /* DLY: NOP without driving CK lane.  Same rationale as LP11 - BTA needs
   * the CK lane to stay in HS continuous so the byte clock keeps running.
   */

  v  = (T113_DSI_MODE_NOP << T113_DSI_INST_MODE_SHIFT) &
       T113_DSI_INST_MODE_MASK;
  v |= (lane_mask << T113_DSI_INST_LANE_DEN_SHIFT) &
       T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_DLY), v);

  /* Loop counters: 50 cycles on each of the two counters (N0 for the
   * primary LP11/DLY chain, N1 for the with_rx variant), so the DPHY has
   * enough settle time on either side of the escape burst.
   */

  v  = ((50u - 1) << T113_DSI_LOOP_N0_SHIFT) & T113_DSI_LOOP_N0_MASK;
  v |= ((50u - 1) << T113_DSI_LOOP_N1_SHIFT) & T113_DSI_LOOP_N1_MASK;
  t113_dsi_putreg(T113_DSI_INST_LOOP_NUM_OFFSET, v);
  t113_dsi_putreg(T113_DSI_INST_LOOP_NUM2_OFFSET, v);
  t113_dsi_putreg(T113_DSI_INST_LOOP_SEL_OFFSET,
                  ((uint32_t)T113_DSI_ID_HSC << (4 * T113_DSI_ID_LP11)) |
                  ((uint32_t)T113_DSI_ID_HSD << (4 * T113_DSI_ID_DLY)));

  /* DEBUG_DATA holds the byte the host pads onto an idle/aborted LP line.
   * Reset value is 0x00 which can be misread by the panel as an LPDT
   * SoT continuation; vendor uses 0xff so the lane returns to recognisably
   * idle bytes between escape bursts.
   */

  t113_dsi_putreg(T113_DSI_DBG_DATA_OFFSET, 0xffu);

  /* Enable ECC + CRC BEFORE the controller is brought out of reset so the
   * very first DCS byte the panel sees in LP escape mode is properly
   * framed.  EoTp is intentionally OFF: GC9503CV does not gate on the
   * end-of-transmission packet, and the vendor reference disables it.
   * (start_video re-asserts the same bits when it configures the HS
   * pixel-mode plumbing - a redundant write but kept for symmetry with
   * that path.)
   */

  t113_dsi_putreg(T113_DSI_BASIC_CTL0_OFFSET,
                  T113_DSI_BCTL0_ECC_EN | T113_DSI_BCTL0_CRC_EN);

  /* Bring the host out of reset. */

  modifyreg32(T113_DSI_BASE + T113_DSI_GCTL_OFFSET, 0, T113_DSI_GCTL_EN);
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_lp_run
 *
 * Description:
 *   Kick the instruction sequencer through one LP transaction and wait for
 *   the INSTR_END interrupt.  Caller must have staged the header/payload
 *   into CMD_TX and set CMD_CTL.TX_SIZE.  with_rx selects the slot chain
 *   that includes a bus-turnaround for read responses; INSTR_END fires for
 *   both chains so it also gates the W1C set applied to CMD_CTL on the way
 *   out (TX_FLAG only vs TX_FLAG | RX_FLAG).
 *
 ****************************************************************************/

static int t113_dsi_lp_run(bool with_rx)
{
  FAR struct t113_dsi_dev_s *priv = &g_t113_dsi;
  uint32_t jump;

  /* Build the slot chain.
   *
   * TX-only: LP11 -> LPDT -> END
   * TX+RX:   LP11 -> LPDT -> DLY -> TBA -> END
   */

  jump = ((uint32_t)T113_DSI_ID_LPDT << (4 * T113_DSI_ID_LP11)) |
         ((uint32_t)T113_DSI_ID_END  << (4 * T113_DSI_ID_LPDT));

  if (with_rx)
    {
      jump = ((uint32_t)T113_DSI_ID_LPDT << (4 * T113_DSI_ID_LP11)) |
             ((uint32_t)T113_DSI_ID_DLY  << (4 * T113_DSI_ID_LPDT)) |
             ((uint32_t)T113_DSI_ID_TBA  << (4 * T113_DSI_ID_DLY))  |
             ((uint32_t)T113_DSI_ID_END  << (4 * T113_DSI_ID_TBA));
    }

  /* DO NOT re-pulse HSC here.  This was an experiment that caused a
   * regression: every LP transfer would force INST_ST 0->1 with the HSC
   * jump chain, which interrupted the in-flight HS continuous on the CK
   * lane.  Vendor lcd_panel_init pulses HSC exactly once via
   * sunxi_lcd_dsi_clk_enable() and never touches it again.  Trust the
   * upfront pulse; LP transfers go directly to the LP11->LPDT path.
   */

  /* Vendor-style polling: wait inst_st==0 (sequencer idle) up to 5ms,
   * then set JUMP_SEL and pulse INST_ST.  Fire-and-forget - vendor does
   * NOT wait for completion, the next call's pre-poll catches the
   * previous transfer.  This skips IRQ semantics entirely.
   */

  for (int n = 0; n < 50; n++)
    {
      if ((t113_dsi_getreg(T113_DSI_BASIC_CTL0_OFFSET) &
           T113_DSI_BCTL0_INST_ST) == 0)
        {
          break;
        }

      up_udelay(100);
    }

  t113_dsi_putreg(T113_DSI_INST_JUMP_SEL_OFFSET, jump);
  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              T113_DSI_BCTL0_INST_ST, 0);
  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              0, T113_DSI_BCTL0_INST_ST);

  /* For with_rx (DCS read), poll inst_st back to 0 then snapshot CMD_CTL
   * for RX_FLAG / RX FIFO.  TX-only is fire-and-forget like vendor.
   */

  if (with_rx)
    {
      int n;

      for (n = 0; n < 100; n++)
        {
          if ((t113_dsi_getreg(T113_DSI_BASIC_CTL0_OFFSET) &
               T113_DSI_BCTL0_INST_ST) == 0)
            {
              break;
            }

          up_udelay(100);
        }

      /* Snapshot CMD_CTL while RX_FLAG is still latched - once the
       * sequencer self-clears INST_ST, subsequent CMD_CTL reads can drop
       * RX_FLAG via the W1C path.  Caller must consume priv->last_cmd_ctl
       * to see the original BTA outcome.
       */

      priv->last_cmd_ctl = t113_dsi_getreg(T113_DSI_CMD_CTL_OFFSET);

      if (n >= 100)
        {
          lcderr("DSI lp_run: BTA timeout (10ms), CMD_CTL=0x%08lx\n",
                 (unsigned long)priv->last_cmd_ctl);
          return -ETIMEDOUT;
        }
    }
  else
    {
      priv->last_cmd_ctl = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_dsi_wait_inst_idle
 *
 * Description:
 *   Poll for the instruction sequencer to self-clear INST_ST, up to ~5 ms.
 *   Force INST_ST=0 only on timeout.  Mirrors the wait at the head of the
 *   vendor dsi_dcs_wr() - if the previous chain has not fully drained out
 *   of the END slot, filling CMD_TX/CMD_CTL for a new transfer races the
 *   in-flight sequencer and the DDIC sees garbled bytes.
 *
 ****************************************************************************/

static void t113_dsi_wait_inst_idle(void)
{
  unsigned int count;

  for (count = 0; count < 50; count++)
    {
      if ((t113_dsi_getreg(T113_DSI_BASIC_CTL0_OFFSET) &
           T113_DSI_BCTL0_INST_ST) == 0)
        {
          return;
        }

      up_udelay(100);
    }

  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              T113_DSI_BCTL0_INST_ST, 0);
}

/****************************************************************************
 * Name: t113_dsi_short_write
 ****************************************************************************/

static ssize_t t113_dsi_short_write(FAR const struct mipi_dsi_msg *msg)
{
  struct mipi_dsi_packet pkt;
  uint32_t header24;
  uint32_t word;
  int      ret;

  ret = mipi_dsi_create_packet(&pkt, msg);
  if (ret < 0)
    {
      return ret;
    }

  /* Block until the previous LP chain has fully drained.  The IRQ posts on
   * INSTR_END, which can fire a hair before the sequencer self-clears
   * INST_ST; a poll here keeps the FIFO write below from racing the tail
   * of an in-flight transfer.
   */

  t113_dsi_wait_inst_idle();

  header24 = (uint32_t)pkt.header[0]       |
             (uint32_t)pkt.header[1] << 8  |
             (uint32_t)pkt.header[2] << 16;
  pkt.header[3] = t113_dsi_ecc(header24);

  word = (uint32_t)pkt.header[0]        |
         (uint32_t)pkt.header[1] << 8   |
         (uint32_t)pkt.header[2] << 16  |
         (uint32_t)pkt.header[3] << 24;
  t113_dsi_putreg(T113_DSI_CMD_TX_OFFSET(0), word);

  modifyreg32(T113_DSI_BASE + T113_DSI_CMD_CTL_OFFSET,
              T113_DSI_CMDCTL_TX_SIZE_MASK,
              ((4u - 1) << T113_DSI_CMDCTL_TX_SIZE_SHIFT) &
              T113_DSI_CMDCTL_TX_SIZE_MASK);

  ret = t113_dsi_lp_run(false);
  if (ret < 0)
    {
      return ret;
    }

  return (ssize_t)msg->tx_len;
}

/****************************************************************************
 * Name: t113_dsi_long_write
 ****************************************************************************/

static ssize_t t113_dsi_long_write(FAR const struct mipi_dsi_msg *msg)
{
  struct mipi_dsi_packet pkt;
  uint32_t header24;
  uint16_t crc;
  size_t   total;
  size_t   nwords;
  size_t   i;
  uint32_t word;
  uint8_t  bytebuf[T113_DSI_TX_FIFO_BYTES];
  int      ret;

  if (msg->tx_buf == NULL && msg->tx_len > 0)
    {
      return -EINVAL;
    }

  total = 4 + msg->tx_len + 2;        /* header + payload + CRC */
  if (total > T113_DSI_TX_FIFO_BYTES)
    {
      return -EMSGSIZE;
    }

  ret = mipi_dsi_create_packet(&pkt, msg);
  if (ret < 0)
    {
      return ret;
    }

  /* See note in t113_dsi_short_write - wait out the previous chain before
   * touching CMD_TX / CMD_CTL for this one.
   */

  t113_dsi_wait_inst_idle();

  header24 = (uint32_t)pkt.header[0]       |
             (uint32_t)pkt.header[1] << 8  |
             (uint32_t)pkt.header[2] << 16;
  pkt.header[3] = t113_dsi_ecc(header24);

  crc = t113_dsi_crc((FAR const uint8_t *)msg->tx_buf, msg->tx_len);

  /* Stage as a contiguous byte stream, then drain word-by-word into the
   * FIFO.  Tail bytes left zero - the hardware uses CMD_CTL.TX_SIZE to
   * decide how many bytes to actually shift out.
   */

  memset(bytebuf, 0, sizeof(bytebuf));
  bytebuf[0] = pkt.header[0];
  bytebuf[1] = pkt.header[1];
  bytebuf[2] = pkt.header[2];
  bytebuf[3] = pkt.header[3];
  if (msg->tx_len > 0)
    {
      memcpy(&bytebuf[4], msg->tx_buf, msg->tx_len);
    }

  bytebuf[4 + msg->tx_len]     = (uint8_t)(crc & 0xff);
  bytebuf[4 + msg->tx_len + 1] = (uint8_t)((crc >> 8) & 0xff);

  nwords = (total + 3) / 4;
  for (i = 0; i < nwords; i++)
    {
      word = (uint32_t)bytebuf[i * 4]            |
             (uint32_t)bytebuf[i * 4 + 1] << 8   |
             (uint32_t)bytebuf[i * 4 + 2] << 16  |
             (uint32_t)bytebuf[i * 4 + 3] << 24;
      t113_dsi_putreg(T113_DSI_CMD_TX_OFFSET(i), word);
    }

  modifyreg32(T113_DSI_BASE + T113_DSI_CMD_CTL_OFFSET,
              T113_DSI_CMDCTL_TX_SIZE_MASK,
              ((uint32_t)(total - 1) << T113_DSI_CMDCTL_TX_SIZE_SHIFT) &
              T113_DSI_CMDCTL_TX_SIZE_MASK);

  ret = t113_dsi_lp_run(false);
  if (ret < 0)
    {
      return ret;
    }

  return (ssize_t)msg->tx_len;
}

/****************************************************************************
 * Name: t113_dsi_read
 *
 * Description:
 *   DCS short read.  Issues a DCS_READ_0_PARAM short packet and copies the
 *   1- or 2-byte response back into msg->rx_buf.  Long-read decode is
 *   deferred - Phase 2 panel use only needs DCS 0x09 status (1 byte).
 *
 ****************************************************************************/

static ssize_t t113_dsi_read(FAR const struct mipi_dsi_msg *msg)
{
  struct mipi_dsi_packet pkt;
  uint32_t header24;
  uint32_t word;
  uint32_t cs;
  uint32_t rx;
  uint8_t  rx_dt;
  size_t   ncopy;
  int      ret;

  if (msg->rx_buf == NULL || msg->rx_len == 0)
    {
      return -EINVAL;
    }

  ret = mipi_dsi_create_packet(&pkt, msg);
  if (ret < 0)
    {
      return ret;
    }

  /* D4: drain any in-flight LP chain before staging this read.  Without
   * this we race the tail of the prior write's sequencer the same way the
   * short_write / long_write paths would.
   */

  t113_dsi_wait_inst_idle();

  header24 = (uint32_t)pkt.header[0]       |
             (uint32_t)pkt.header[1] << 8  |
             (uint32_t)pkt.header[2] << 16;
  pkt.header[3] = t113_dsi_ecc(header24);

  word = (uint32_t)pkt.header[0]        |
         (uint32_t)pkt.header[1] << 8   |
         (uint32_t)pkt.header[2] << 16  |
         (uint32_t)pkt.header[3] << 24;
  t113_dsi_putreg(T113_DSI_CMD_TX_OFFSET(0), word);

  modifyreg32(T113_DSI_BASE + T113_DSI_CMD_CTL_OFFSET,
              T113_DSI_CMDCTL_TX_SIZE_MASK,
              ((4u - 1) << T113_DSI_CMDCTL_TX_SIZE_SHIFT) &
              T113_DSI_CMDCTL_TX_SIZE_MASK);

  ret = t113_dsi_lp_run(true);
  if (ret < 0)
    {
      return ret;
    }

  /* D3: use the snapshot lp_run captured BEFORE the sequencer's W1C path
   * could clear RX_FLAG.  A fresh getreg(CMD_CTL) here would race the HW
   * self-clear and routinely show RX_FLAG=0 even on a successful BTA.
   */

  cs = g_t113_dsi.last_cmd_ctl;

  if (cs & T113_DSI_CMDCTL_RX_OVERFLOW)
    {
      lcderr("DSI read: RX overflow, CMD_CTL=0x%08lx\n", (unsigned long)cs);
      return -EIO;
    }

  rx    = t113_dsi_getreg(T113_DSI_CMD_RX_OFFSET(0));
  rx_dt = (uint8_t)(rx & 0xff);

  switch (rx_dt)
    {
      case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
      case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
        ncopy = 1;
        break;

      case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
      case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
        ncopy = 2;
        break;

      default:
        lcderr("DSI read: unexpected DT=0x%02x rx0=0x%08lx "
               "CMD_CTL=0x%08lx\n",
               rx_dt, (unsigned long)rx, (unsigned long)cs);
        return -EIO;
    }

  if (ncopy > msg->rx_len)
    {
      lcderr("DSI read: caller buffer too small: have=%zu need=%zu\n",
             msg->rx_len, ncopy);
      return -EOVERFLOW;
    }

  ((FAR uint8_t *)msg->rx_buf)[0] = (uint8_t)((rx >> 8) & 0xff);
  if (ncopy == 2)
    {
      ((FAR uint8_t *)msg->rx_buf)[1] = (uint8_t)((rx >> 16) & 0xff);
    }

  return (ssize_t)ncopy;
}

/****************************************************************************
 * Name: t113_dsi_transfer
 *
 * Description:
 *   Dispatch a DSI message onto the LP command path.
 *
 ****************************************************************************/

static ssize_t t113_dsi_transfer(FAR struct mipi_dsi_host *host,
                                 FAR const struct mipi_dsi_msg *msg)
{
  UNUSED(host);

  if (msg == NULL)
    {
      return -EINVAL;
    }

  switch (msg->type)
    {
      case MIPI_DSI_DCS_SHORT_WRITE_0_PARAM:
      case MIPI_DSI_DCS_SHORT_WRITE_1_PARAM:
      case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
      case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
      case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
      case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
        return t113_dsi_short_write(msg);

      case MIPI_DSI_DCS_LONG_WRITE:
      case MIPI_DSI_LONG_GENERIC_WRITE:
        return t113_dsi_long_write(msg);

      case MIPI_DSI_DCS_READ_0_PARAM:
      case MIPI_DSI_GENERIC_READ_0_PARAM:
      case MIPI_DSI_GENERIC_READ_1_PARAM:
      case MIPI_DSI_GENERIC_READ_2_PARAM:
        return t113_dsi_read(msg);

      default:
        lcderr("Unsupported DSI msg type 0x%02x\n", msg->type);
        return -EOPNOTSUPP;
    }
}

/****************************************************************************
 * Name: t113_dsi_clock_init
 *
 * Description:
 *   Strict minimum gates required for the DSI host + DPHY register windows
 *   to be reachable from the CPU.  Per facts.md section 2 the DPHY PLL is
 *   local to the DPHY block and does not consume a CCU clock - there is no
 *   T113_CLK_DSI_PHY.  DPSS_TOP / TCON / DE belong to Tasks 3 and 4 and are
 *   left untouched here.
 *
 ****************************************************************************/

static void t113_dsi_clock_init(void)
{
  t113_clk_enable(T113_CLK_DSI_BUS);
  t113_clk_enable(T113_CLK_DSI);
  t113_clk_reset_deassert(T113_RST_DSI);
}

/****************************************************************************
 * Name: t113_dsi_phy_pll_set
 *
 * Description:
 *   Program the DPHY PLL for the requested per-lane HS bit-rate.  The VCO
 *   target is 1.3 GHz.  The PLL is fed by the 24 MHz HOSC and produces:
 *
 *     hs_clk = 24 MHz * N / ((P + 1) * (M0 + 1))
 *
 *   We pick the smallest M (final divider 1..4) that lifts the loop above
 *   the VCO floor, then back-solve N from 24 MHz.  P is held at 0 (final
 *   divider 1) and M1 at 2 to match silicon validation values.  Spread
 *   spectrum and SDM are disabled.
 *
 *   The T113 DPHY PLL has no CPU-readable lock-detect bit, so we simply
 *   write the dividers and return.  Callers wait through the analog
 *   bring-up sequence in t113_dsi_phy_init() instead.
 *
 ****************************************************************************/

static int t113_dsi_phy_pll_set(uint32_t bitrate_per_lane_hz)
{
  const uint64_t vco_floor = 1300000000ull;
  const uint32_t hosc_hz   = 24000000;
  uint64_t target;
  uint32_t m;
  uint32_t n;
  uint32_t m0;
  uint32_t p;
  uint32_t m1;
  uint32_t reg0;

  /* Smallest m in [1..4] such that bitrate * m > 1.3 GHz. */

  for (m = 1; m <= 4; m++)
    {
      if ((uint64_t)bitrate_per_lane_hz * m > vco_floor)
        {
          break;
        }
    }

  if (m > 4)
    {
      lcdwarn("DSI PHY: pclk too low, capping m=4\n");
      m = 4;
    }

  target = (uint64_t)bitrate_per_lane_hz * m;
  n      = (uint32_t)(target / hosc_hz);
  m0     = m - 1;
  p      = 0;
  m1     = 2;

  /* Sanity check the X4B 324 Mbps/lane case: m=4, m0=3, p=0, n=54.  This
   * catches a math typo at runtime but is harmless for other bitrates.
   */

  if (bitrate_per_lane_hz == 324000000 && (n != 54 || m0 != 3))
    {
      lcderr("DSI PHY: PLL math wrong for 324 Mbps/lane: n=%u m0=%u\n",
             n, m0);
      return -EINVAL;
    }

  /* Read-modify-write: keep TDIV / NDET / EN_LVS / CP36_EN at their
   * hardware reset values (vendor de_dsi_type.h marks NDET / EN_LVS /
   * CP36_EN as default 1).  We only stamp our integer dividers and ensure
   * PLL_EN + LDO_EN are on.
   */

  reg0  = t113_dphy_getreg(T113_DPHY_PLL_REG0_OFFSET);
  reg0 &= ~(T113_DPHY_PLL_M1_MASK | T113_DPHY_PLL_M0_MASK |
            T113_DPHY_PLL_N_MASK  | T113_DPHY_PLL_P_MASK);
  reg0 |= (m1 << T113_DPHY_PLL_M1_SHIFT) & T113_DPHY_PLL_M1_MASK;
  reg0 |= (m0 << T113_DPHY_PLL_M0_SHIFT) & T113_DPHY_PLL_M0_MASK;
  reg0 |= (n  << T113_DPHY_PLL_N_SHIFT)  & T113_DPHY_PLL_N_MASK;
  reg0 |= (p  << T113_DPHY_PLL_P_SHIFT)  & T113_DPHY_PLL_P_MASK;
  reg0 |= T113_DPHY_PLL_EN | T113_DPHY_PLL_LDO_EN;
  t113_dphy_putreg(T113_DPHY_PLL_REG0_OFFSET, reg0);

  /* Disable SDM / spread-spectrum / fractional. */

  t113_dphy_putreg(T113_DPHY_PLL_REG2_OFFSET, 0);

  lcdinfo("DSI PHY PLL: bitrate=%u Hz n=%u m=%u (m0=%u) p=%u m1=%u\n",
          bitrate_per_lane_hz, n, m, m0, p, m1);
  return 0;
}

/****************************************************************************
 * Name: t113_dsi_phy_init
 *
 * Description:
 *   Bring the combo DPHY out of reset and configure it for MIPI mode at
 *   the requested per-lane bit-rate.  The sequence below mirrors the
 *   vendor power-up order: analog drive trim, CPU-control enables, LDOs,
 *   PHY core bias, PLL, MIPI mode select, and finally the global module
 *   enable.  A 1 us soak between LDO-up and termination-up gives the LDOs
 *   time to settle before they're loaded.
 *
 *   Lane count is hard-coded at 2 to match the X4B panel; the host bus is
 *   currently single-instance.  TODO: make this dynamic when panel attach
 *   feeds lane count back in.
 *
 ****************************************************************************/

static int t113_dsi_phy_init(uint32_t bitrate_per_lane_hz)
{
  const uint32_t lanes    = 2;
  const uint32_t lane_mask = (1u << lanes) - 1;  /* one bit per active lane */
  uint32_t v;
  int ret;

  /* (1) Latch lane_num while MODULE_EN=0.  The DPHY samples lane_num into
   * its internal state machine on the rising edge of MODULE_EN, so the
   * value must be in place before the enable bit is set.  Programming both
   * fields in a single write leaves lane_num at its reset value (0 -> one
   * lane), which would gate D1 LP-TX permanently low.
   */

  t113_dphy_putreg(T113_DPHY_GCTL_OFFSET,
                   ((lanes - 1u) << T113_DPHY_GCTL_LANE_NUM_SHIFT) &
                   T113_DPHY_GCTL_LANE_NUM_MASK);

  /* (2) DPHY TX timing.  Programmed BEFORE the analog enable block so the
   * silicon-validated R528 timing constants are present in the digital
   * registers when MODULE_EN's rising edge samples both digital and analog
   * config in one coherent state.  Programming TX_TIME after analog stand-up
   * leaves DBG0.lptx_sta_clk stuck at 0 (CK lane never reaches HS).  These
   * counts are the vendor's defaults across every panel rate (24..1.5 Gbps/
   * lane); they are NOT recomputed per bitrate.  D-PHY minimums met:
   * T_LPX>=50ns, T_HS-PREPARE>=40ns+4UI, T_HS-TRAIL>=max(8UI,60ns+4UI),
   * T_CLK-ZERO>=262ns, T_CLK-POST>=60ns+52UI, T_CLK-TRAIL>=60ns.  DTERM=0
   * (vendor leaves the differential termination-setup count idle; a non-zero
   * value here delays D-term assertion past SoT and the panel cannot lock
   * onto the LP->HS preamble).
   */

  v  = (14u << T113_DPHY_TX_TIME0_LPX_SHIFT)      &
       T113_DPHY_TX_TIME0_LPX_MASK;
  v |= (0u << T113_DPHY_TX_TIME0_DTERM_SHIFT)    &
       T113_DPHY_TX_TIME0_DTERM_MASK;
  v |= (6u << T113_DPHY_TX_TIME0_HS_PRE_SHIFT)   &
       T113_DPHY_TX_TIME0_HS_PRE_MASK;
  v |= (10u << T113_DPHY_TX_TIME0_HS_TRAIL_SHIFT) &
       T113_DPHY_TX_TIME0_HS_TRAIL_MASK;
  t113_dphy_putreg(T113_DPHY_TX_TIME0_OFFSET, v);

  v  = (7u << T113_DPHY_TX_TIME1_CK_PREP_SHIFT) &
       T113_DPHY_TX_TIME1_CK_PREP_MASK;
  v |= (50u << T113_DPHY_TX_TIME1_CK_ZERO_SHIFT) &
       T113_DPHY_TX_TIME1_CK_ZERO_MASK;
  v |= (3u << T113_DPHY_TX_TIME1_CK_PRE_SHIFT)  &
       T113_DPHY_TX_TIME1_CK_PRE_MASK;
  v |= (10u << T113_DPHY_TX_TIME1_CK_POST_SHIFT) &
       T113_DPHY_TX_TIME1_CK_POST_MASK;
  t113_dphy_putreg(T113_DPHY_TX_TIME1_OFFSET, v);

  v = (30u << T113_DPHY_TX_TIME2_CK_TRAIL_SHIFT) &
      T113_DPHY_TX_TIME2_CK_TRAIL_MASK;
  t113_dphy_putreg(T113_DPHY_TX_TIME2_OFFSET, v);

  t113_dphy_putreg(T113_DPHY_TX_TIME3_OFFSET, 0);

  v  = (3u << T113_DPHY_TX_TIME4_HSTX_ANA0_SHIFT) &
       T113_DPHY_TX_TIME4_HSTX_ANA0_MASK;
  v |= (3u << T113_DPHY_TX_TIME4_HSTX_ANA1_SHIFT) &
       T113_DPHY_TX_TIME4_HSTX_ANA1_MASK;
  t113_dphy_putreg(T113_DPHY_TX_TIME4_OFFSET, v);

  /* (3) HS clock continuous between transfers - required by the panel's
   * PLL recovery for stable LP/HS handoff during DCS init.  Latched while
   * MODULE_EN=0 alongside the TX timing above; vendor's dsi_dphy_cfg writes
   * this in the same digital-config block before any analog stand-up.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_TX_CTL_OFFSET,
              0, T113_DPHY_TX_CLK_CONT);

  /* (4) ANA4 analog drive / impedance trim. */

  v = t113_dphy_getreg(T113_DPHY_ANA4_OFFSET);
  v &= ~(T113_DPHY_ANA4_IB_MASK   | T113_DPHY_ANA4_DMPLVC      |
         T113_DPHY_ANA4_DMPLVD_MASK | T113_DPHY_ANA4_VTT_SET_MASK |
         T113_DPHY_ANA4_CKDV_MASK | T113_DPHY_ANA4_TMSD_MASK    |
         T113_DPHY_ANA4_TMSC_MASK | T113_DPHY_ANA4_TXPUSD_MASK  |
         T113_DPHY_ANA4_TXPUSC_MASK | T113_DPHY_ANA4_TXDNSD_MASK);
  v |= (2u << T113_DPHY_ANA4_IB_SHIFT)      & T113_DPHY_ANA4_IB_MASK;
  v |= (4u << T113_DPHY_ANA4_DMPLVD_SHIFT)  & T113_DPHY_ANA4_DMPLVD_MASK;
  v |= (3u << T113_DPHY_ANA4_VTT_SET_SHIFT) & T113_DPHY_ANA4_VTT_SET_MASK;
  v |= (3u << T113_DPHY_ANA4_CKDV_SHIFT)    & T113_DPHY_ANA4_CKDV_MASK;
  v |= (1u << T113_DPHY_ANA4_TMSD_SHIFT)    & T113_DPHY_ANA4_TMSD_MASK;
  v |= (1u << T113_DPHY_ANA4_TMSC_SHIFT)    & T113_DPHY_ANA4_TMSC_MASK;
  v |= (2u << T113_DPHY_ANA4_TXPUSD_SHIFT)  & T113_DPHY_ANA4_TXPUSD_MASK;
  v |= (3u << T113_DPHY_ANA4_TXPUSC_SHIFT)  & T113_DPHY_ANA4_TXPUSC_MASK;
  v |= (2u << T113_DPHY_ANA4_TXDNSD_SHIFT)  & T113_DPHY_ANA4_TXDNSD_MASK;
  t113_dphy_putreg(T113_DPHY_ANA4_OFFSET, v);

  /* (3) ANA2: clock-lane CPU enable, then bias-current enable.  Order
   * matters: clock-lane control must be live before the bias is loaded.
   *
   * Also enable the per-lane HS RX CPU enables and the HS RX clock-lane
   * enable here - vendor de_dsi_type.h enrx_cpu (bit[23:20]) and the
   * clock-lane HS RX enable (bit[14]) must be live before BTA so that
   * the receiver path can latch return data when the panel turns the
   * bus around.  Without these the LP RX status (DBG0[7:0]) never
   * leaves 0x07 and BTA times out.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_ANA2_OFFSET,
              0,
              T113_DPHY_ANA2_ENCK_CPU |
              T113_DPHY_ANA2_ENCKRX_CPU |
              (((uint32_t)lane_mask << T113_DPHY_ANA2_ENRX_CPU_SHIFT) &
               T113_DPHY_ANA2_ENRX_CPU_MASK));
  modifyreg32(T113_DPHY_BASE + T113_DPHY_ANA2_OFFSET,
              0, T113_DPHY_ANA2_ENIB);

  /* (4) ANA3: power up reference / clock / data LDOs, and enable the
   * per-lane LP RX + clock-lane LP RX path (vendor enlprx_cpu /
   * enlprxc_cpu).  These bits gate the LP receiver comparators; without
   * them BTA never sees the panel turn LP-00 -> LP-11 and the controller
   * sits in waitack forever.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_ANA3_OFFSET,
              0,
              T113_DPHY_ANA3_ENLDOR |
              T113_DPHY_ANA3_ENLDOC |
              T113_DPHY_ANA3_ENLDOD |
              T113_DPHY_ANA3_ENLPRXC_CPU |
              (((uint32_t)lane_mask << T113_DPHY_ANA3_ENLPRX_CPU_SHIFT) &
               T113_DPHY_ANA3_ENLPRX_CPU_MASK));

  /* (5) ANA0: PHY-core bias settings.  PLR=4, SFB=1, SELSCK=0, RSD=0. */

  v = t113_dphy_getreg(T113_DPHY_ANA0_OFFSET);
  v &= ~(T113_DPHY_ANA0_PLR_MASK | T113_DPHY_ANA0_SFB_MASK |
         T113_DPHY_ANA0_SELSCK   | T113_DPHY_ANA0_RSD);
  v |= (4u << T113_DPHY_ANA0_PLR_SHIFT) & T113_DPHY_ANA0_PLR_MASK;
  v |= (1u << T113_DPHY_ANA0_SFB_SHIFT) & T113_DPHY_ANA0_SFB_MASK;
  t113_dphy_putreg(T113_DPHY_ANA0_OFFSET, v);

  /* (6) Combo PHY: charge-pump on. */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_COMBO_PHY0_OFFSET,
              0, T113_DPHY_COMBO_EN_CP);

  /* (7) Program the PLL. */

  ret = t113_dsi_phy_pll_set(bitrate_per_lane_hz);
  if (ret < 0)
    {
      return ret;
    }

  /* (8) ANA4: enter MIPI mode and finalise pull-down clock drive. */

  v = t113_dphy_getreg(T113_DPHY_ANA4_OFFSET);
  v &= ~T113_DPHY_ANA4_TXDNSC_MASK;
  v |= (3u << T113_DPHY_ANA4_TXDNSC_SHIFT) & T113_DPHY_ANA4_TXDNSC_MASK;
  v |= T113_DPHY_ANA4_EN_MIPI;
  t113_dphy_putreg(T113_DPHY_ANA4_OFFSET, v);

  /* (9) Combo PHY: switch to MIPI and enable the combo LDO. */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_COMBO_PHY0_OFFSET,
              0,
              T113_DPHY_COMBO_EN_MIPI | T113_DPHY_COMBO_EN_LDO);

  /* (10) Combo PHY: HS->LP stop delay = 20 cycles. */

  v = t113_dphy_getreg(T113_DPHY_COMBO_PHY2_OFFSET);
  v &= ~T113_DPHY_COMBO_HS_STOP_DLY_MASK;
  v |= (20u << T113_DPHY_COMBO_HS_STOP_DLY_SHIFT) &
       T113_DPHY_COMBO_HS_STOP_DLY_MASK;
  t113_dphy_putreg(T113_DPHY_COMBO_PHY2_OFFSET, v);

  /* (11) 1 us LDO-settle soak. */

  up_udelay(1);

  /* (12) ANA3: clock termination + per-lane data termination + divider. */

  v = t113_dphy_getreg(T113_DPHY_ANA3_OFFSET);
  v &= ~T113_DPHY_ANA3_ENVTTD_MASK;
  v |= T113_DPHY_ANA3_ENVTTC | T113_DPHY_ANA3_ENDIV;
  v |= ((uint32_t)lane_mask << T113_DPHY_ANA3_ENVTTD_SHIFT) &
       T113_DPHY_ANA3_ENVTTD_MASK;
  t113_dphy_putreg(T113_DPHY_ANA3_OFFSET, v);

  /* (13) ANA2: re-affirm clock-lane CPU enable.  Done as a standalone write
   * BEFORE vttmode flips, matching the vendor combo-DPHY power-up ordering
   * (R528 dsi_dphy_open).  Combining enck_cpu and enp2s_cpu into one write
   * caused the data-lane parallel-to-serial drivers to come live before the
   * VTT regulator was switched into regulated mode - symptom: DBG0
   * lptx_sta_clk stuck at 0, CK lane never acquires HS lock after the HSC
   * pulse, every LP DCS read returns DT=0x00 with the panel mute on the
   * wire.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_ANA2_OFFSET,
              0, T113_DPHY_ANA2_ENCK_CPU);

  /* (14) ANA1: VTT mode = 1 (regulated-VTT path).  Must precede the
   * enp2s_cpu enable - the regulated termination path needs to be live
   * before the per-lane drivers latch into their HS bias.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_ANA1_OFFSET,
              0, T113_DPHY_ANA1_VTTMODE);

  /* (14b) ANA2: install per-lane parallel-to-serial CPU enables for our
   * active data lanes.  Last analog enable before module_en flips.
   */

  v = t113_dphy_getreg(T113_DPHY_ANA2_OFFSET);
  v &= ~T113_DPHY_ANA2_ENP2S_CPU_MASK;
  v |= ((uint32_t)lane_mask << T113_DPHY_ANA2_ENP2S_CPU_SHIFT) &
       T113_DPHY_ANA2_ENP2S_CPU_MASK;
  t113_dphy_putreg(T113_DPHY_ANA2_OFFSET, v);

  /* (17) Release DPHY from disabled state.  lane_num, TX timing and
   * hstx_clk_cont were already latched in steps (1)-(3); flipping
   * MODULE_EN now samples the entire digital + analog config into the
   * state machine and the LP-TX drivers go live on all configured
   * lanes.
   */

  modifyreg32(T113_DPHY_BASE + T113_DPHY_GCTL_OFFSET,
              0, T113_DPHY_GCTL_MODULE_EN);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_dsi_initialize
 *
 * Description:
 *   Bring up the DSI module clocks + DPHY, install the IRQ handler, and
 *   register the host with the NuttX MIPI DSI framework.
 *
 * Returned Value:
 *   OK on success.  -EALREADY if called twice.  Negative errno otherwise.
 *
 ****************************************************************************/

int t113_dsi_initialize(void)
{
  int ret;

  if (g_t113_dsi.initialized)
    {
      return -EALREADY;
    }

  nxsem_init(&g_t113_dsi.done_sem, 0, 0);

  t113_dsi_clock_init();

  /* EVB4 2-lane.  TCON module clock is hardcoded HOSC/1 = 24 MHz, so
   * actual panel pclk is 24 MHz, not the 31 MHz the EVB4 config wants.
   * Bitrate = 24 MHz * 24 bpp / 2 lanes = 288 Mbps/lane, matching what
   * TCON actually delivers downstream.
   */

  ret = t113_dsi_phy_init(288000000);
  if (ret < 0)
    {
      lcderr("t113_dsi_phy_init failed: %d\n", ret);
      return ret;
    }

  ret = t113_dsi_inst_init(T113_DSI_LANE_MASK);
  if (ret < 0)
    {
      lcderr("t113_dsi_inst_init failed: %d\n", ret);
      return ret;
    }

  g_t113_dsi.host.bus = 0;
  g_t113_dsi.host.ops = &g_t113_dsi_host_ops;

  ret = mipi_dsi_host_register(&g_t113_dsi.host);
  if (ret < 0)
    {
      lcderr("mipi_dsi_host_register failed: %d\n", ret);
      return ret;
    }

  irq_attach(T113_IRQ_DSI, t113_dsi_isr, &g_t113_dsi);
  up_enable_irq(T113_IRQ_DSI);

  g_t113_dsi.initialized = true;
  lcdinfo("T113 DSI host registered on bus %d\n", g_t113_dsi.host.bus);
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_blk_long
 *
 * Description:
 *   Stage one of the five blanking long packets (HSA, HBP, HFP, HBLK, VBLK)
 *   into its BLK_x0/BLK_x1 register pair.  Header carries DT_BLK + word
 *   count + ECC; payload is wc zero bytes so the CRC can be computed
 *   without materialising any buffer.
 *
 ****************************************************************************/

static void t113_dsi_blk_long(uint32_t ofs0, uint32_t ofs1, uint32_t wc)
{
  uint32_t header;
  uint32_t crc;

  header = (uint32_t)T113_DSI_DT_BLK |
           ((wc & 0xffffu) << T113_DSI_PPH_WC_SHIFT);
  header |= (uint32_t)t113_dsi_ecc(header) << T113_DSI_BLK_ECC_SHIFT;
  t113_dsi_putreg(ofs0, header);

  crc = (uint32_t)t113_dsi_crc_zeros(wc);
  t113_dsi_putreg(ofs1,
                  (crc << T113_DSI_BLK_PF_SHIFT) & T113_DSI_BLK_PF_MASK);
}

/****************************************************************************
 * Name: t113_dsi_short_packet
 *
 * Description:
 *   Build a 32-bit DSI short-packet word for SYNC_HSS/HSE/VSS/VSE: data
 *   bytes are zero, only DT and ECC matter.
 *
 ****************************************************************************/

static uint32_t t113_dsi_short_packet(uint8_t dt)
{
  uint32_t hdr = dt;

  hdr |= (uint32_t)t113_dsi_ecc(hdr) << T113_DSI_SHORT_ECC_SHIFT;
  return hdr;
}

/****************************************************************************
 * Name: t113_dsi_start_clk
 *
 * Description:
 *   Pulse the HSC chain (LP11 -> HSC -> END) so the DPHY clock lane goes
 *   into HS continuous mode.  Mirrors the vendor sequence
 *   dsi_clk_enable() -> dsi_start(DSI_START_HSC), which the GC9503CV
 *   bring-up calls as the first step of lcd_panel_init() - before the DCS
 *   init stream is sent.  Without it the LP DCS bytes are clocked into a
 *   clock lane that is still in LP-11 idle and the DDIC sees nothing.
 *
 *   Idempotent: t113_dsi_start_video() re-runs the same HSC pulse as its
 *   stage 1, so calling start_clk early does not break the later video
 *   start path.
 *
 ****************************************************************************/

int t113_dsi_start_clk(void)
{
  uint32_t reg;

  if (!g_t113_dsi.initialized)
    {
      return -ENODEV;
    }

  /* HSC slot: drive CK lane through HS-zero/HS-sync prelude into HS
   * continuous.  PIXEL pack with no data lanes - only the clock lane is
   * brought up here.
   */

  reg  = (T113_DSI_MODE_HS << T113_DSI_INST_MODE_SHIFT) &
         T113_DSI_INST_MODE_MASK;
  reg |= (T113_DSI_PACK_PIXEL << T113_DSI_INST_TRANS_PACK_SHIFT) &
         T113_DSI_INST_TRANS_PACK_MASK;
  reg |= T113_DSI_INST_LANE_CEN;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_HSC), reg);

  /* JUMP chain: LP11 -> HSC -> END.  One-shot pulse: the sequencer walks
   * through HSC once and stops, leaving the clock lane in HS continuous.
   */

  t113_dsi_putreg(T113_DSI_INST_JUMP_SEL_OFFSET,
                  ((uint32_t)T113_DSI_ID_HSC << (4 * T113_DSI_ID_LP11)) |
                  ((uint32_t)T113_DSI_ID_END << (4 * T113_DSI_ID_HSC)));

  /* Force INST_ST=0 baseline, then rising edge to 1.  The hardware
   * self-clears INST_ST when the chain finishes; an active 1->0 write
   * here would interrupt the in-flight chain before it completes.
   */

  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              T113_DSI_BCTL0_INST_ST, 0);
  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET, 0,
              T113_DSI_BCTL0_INST_ST);

  /* Vendor lcd_panel_init waits 20 ms after sunxi_lcd_dsi_clk_enable
   * before the first DCS write - gives the DPHY HS continuous loop
   * enough cycles to lock and the panel byte clock to come up clean.
   */

  up_mdelay(20);

  /* Leave INST_FUNC[LP11].LANE_CEN at its init value (1).  Vendor de_dsi.c
   * (v40) sets lane_cen = (pd_plug_dis ? 0 : 1) after the HSC pulse - for
   * the X4B default pd_plug_dis=0 this stays 1, which keeps the CK lane in
   * HS continuous while the sequencer dwells in LP11 between LP DCS bursts.
   * Clearing it (earlier code) caused DBG0.lptx_sta_clk to stay at 0 across
   * post-init / post-first-tx / post-failed-read dumps, i.e. the lock never
   * built - the previous E1 reading of "CEN=1 collapses the lock" looks to
   * have been the same symptom misattributed.
   */

  /* Tiny settle so the byte clock has a couple of cycles before the next
   * LP escape entry.  Vendor doesn't poll either; an explicit microsecond
   * is well above the byte-clock period at 324 Mbps/lane.
   */

  up_udelay(10);

  lcdinfo("DSI HS clock continuous\n");
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_start_video
 *
 * Description:
 *   Switch the DSI host from LP-only mode to HS sync-pulse video.  Programs
 *   the HS pixel-mode instruction slots, the BASIC_CTL0/1 video timing, the
 *   blanking long packets that carry HSA/HBP/HFP/HBLK/VBLK and the pixel
 *   packet header, then pulses INST_ST twice: first through the HSC chain
 *   so the clock lane locks in HS continuous mode, then through the HSD
 *   chain to begin streaming pixels.
 *
 *   Must be called after the panel-side DCS init has completed (the host
 *   needs to remain in LP for those commands) and before the TCON pushes
 *   pixels into the DSI host.
 *
 ****************************************************************************/

int t113_dsi_start_video(const struct t113_dsi_video_s *v)
{
  /* Wire bits per pixel and pixel-format DT for each PIXEL_CTL0.fmt code
   * (0..3): {RGB888, RGB666 loose, RGB666 packed, RGB565}.
   */

  static const uint8_t bits_video[4] =
  {
    24u, 24u, 18u, 16u
  };

  static const uint8_t pixel_dt[4] =
  {
    0x3eu, 0x2eu, 0x1eu, 0x0eu
  };

  uint32_t lane_mask;
  uint32_t bpp;
  uint32_t hsa;
  uint32_t hbp;
  uint32_t hact;
  uint32_t hblk;
  uint32_t hfp;
  uint32_t vblk;
  uint32_t vsd;
  uint32_t reg;
  uint32_t ph;

  if (v == NULL || v->lanes < 1 || v->lanes > 4 || v->format > 3 ||
      v->hsync == 0 || v->vsync == 0 || v->hbp < v->hsync ||
      v->vbp < v->vsync || v->htotal <= v->hactive + v->hbp ||
      v->vtotal <= v->vactive + v->vbp)
    {
      return -EINVAL;
    }

  if (!g_t113_dsi.initialized)
    {
      return -EPERM;
    }

  lane_mask = (1u << v->lanes) - 1u;
  bpp       = bits_video[v->format];

  /* Long-packet byte counts.  Each blanking transaction on the DSI line
   * costs 4 header bytes + WC payload + 2 CRC bytes; the HBP/HBLK group
   * additionally absorbs the 4-byte HSE short packet inserted between
   * them.  These formulas come from the T113-S3 DSI host pipeline timing
   * model and reproduce the values the hardware expects to see on the
   * wire for a sync-pulse video frame.
   */

  hsa  = (uint32_t)v->hsync * bpp / 8u - 10u;
  hbp  = ((uint32_t)v->hbp - v->hsync) * bpp / 8u - 6u;
  hact = (uint32_t)v->hactive * bpp / 8u;
  hblk = ((uint32_t)v->htotal - v->hsync) * bpp / 8u - 10u;
  hfp  = hblk - (4u + hact + 2u) - (4u + hbp + 2u);

  /* Vertical-blank padding so the frame ends on a 4-lane boundary.  Only
   * the 4-lane case can ever see a non-zero remainder.
   */

  if (v->lanes == 4)
    {
      uint32_t tmp;

      tmp = (uint32_t)v->htotal * bpp / 8u * v->vtotal - (4u + hblk + 2u);
      vblk = (4u - tmp % 4u) & 3u;
    }
  else
    {
      vblk = 0;
    }

  /* Frame-start delay: stream the new frame starting at line
   * (vactive + vbp + 1), wrapping into [1, vtotal].
   */

  vsd = (uint32_t)v->vactive + v->vbp + 1u;
  if (vsd > v->vtotal)
    {
      vsd -= v->vtotal;
    }

  if (vsd == 0)
    {
      vsd = 1;
    }

  /* HS clock-prepare slot: drive clock + data lanes through the HS-zero /
   * HS-sync prelude that brings the clock lane up to HS continuous.
   */

  reg  = (T113_DSI_MODE_HS << T113_DSI_INST_MODE_SHIFT) &
         T113_DSI_INST_MODE_MASK;
  reg |= (T113_DSI_PACK_PIXEL << T113_DSI_INST_TRANS_PACK_SHIFT) &
         T113_DSI_INST_TRANS_PACK_MASK;
  reg |= T113_DSI_INST_LANE_CEN;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_HSC), reg);

  /* HS data slot: only data lanes drive HS, the clock lane has already
   * latched HS continuous in the HSC pulse above.
   */

  reg  = (T113_DSI_MODE_HS << T113_DSI_INST_MODE_SHIFT) &
         T113_DSI_INST_MODE_MASK;
  reg |= (T113_DSI_PACK_PIXEL << T113_DSI_INST_TRANS_PACK_SHIFT) &
         T113_DSI_INST_TRANS_PACK_MASK;
  reg |= (lane_mask << T113_DSI_INST_LANE_DEN_SHIFT) &
         T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_HSD), reg);

  /* HSCEXIT slot: tear the clock lane back down to LP between bursts. */

  reg  = (T113_DSI_MODE_HSCEXIT << T113_DSI_INST_MODE_SHIFT) &
         T113_DSI_INST_MODE_MASK;
  reg |= T113_DSI_INST_LANE_CEN;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_HSCEXIT), reg);

  /* NOP slot: holds data lanes idle inside the HSD chain.  Reuses the
   * NOP slot index in the inst_func array (DLY remains the LP-mode NOP).
   */

  reg  = (T113_DSI_MODE_NOP << T113_DSI_INST_MODE_SHIFT) &
         T113_DSI_INST_MODE_MASK;
  reg |= (lane_mask << T113_DSI_INST_LANE_DEN_SHIFT) &
         T113_DSI_INST_LANE_DEN_MASK;
  t113_dsi_putreg(T113_DSI_INST_FUNC_OFFSET(T113_DSI_ID_NOP), reg);

  /* Loop counters: 50 cycles each on N0 and N1 (both LOOP_NUM and
   * LOOP_NUM2 - the secondary copy gates the per-line LP11 -> NOP -> HSD
   * loop, the primary copy gates the inter-frame DLY -> LP11 loop).
   */

  reg  = ((50u - 1u) << T113_DSI_LOOP_N0_SHIFT) & T113_DSI_LOOP_N0_MASK;
  reg |= ((50u - 1u) << T113_DSI_LOOP_N1_SHIFT) & T113_DSI_LOOP_N1_MASK;
  t113_dsi_putreg(T113_DSI_INST_LOOP_NUM_OFFSET, reg);
  t113_dsi_putreg(T113_DSI_INST_LOOP_NUM2_OFFSET, reg);

  /* When LP11's loop expires, fall through to HSCEXIT (HSC chain done);
   * when DLY's loop expires, fall back to LP11 for the next line.
   */

  t113_dsi_putreg(T113_DSI_INST_LOOP_SEL_OFFSET,
                  ((uint32_t)T113_DSI_ID_HSCEXIT << (4 * T113_DSI_ID_LP11)) |
                  ((uint32_t)T113_DSI_ID_LP11    << (4 * T113_DSI_ID_DLY)));

  /* Vertical sync + back-porch sizes (in lines).  The VBP register field
   * excludes the sync pulse - caller passes back porch INCLUDING sync per
   * panel-vendor convention so subtract here.
   */

  reg  = ((uint32_t)v->vsync << T113_DSI_BASIC_SIZE0_VSA_SHIFT) &
         T113_DSI_BASIC_SIZE0_VSA_MASK;
  reg |= ((uint32_t)(v->vbp - v->vsync) << T113_DSI_BASIC_SIZE0_VBP_SHIFT) &
         T113_DSI_BASIC_SIZE0_VBP_MASK;
  t113_dsi_putreg(T113_DSI_BASIC_SIZE0_OFFSET, reg);

  reg  = ((uint32_t)v->vactive << T113_DSI_BASIC_SIZE1_VACT_SHIFT) &
         T113_DSI_BASIC_SIZE1_VACT_MASK;
  reg |= ((uint32_t)v->vtotal  << T113_DSI_BASIC_SIZE1_VT_SHIFT) &
         T113_DSI_BASIC_SIZE1_VT_MASK;
  t113_dsi_putreg(T113_DSI_BASIC_SIZE1_OFFSET, reg);

  /* Enable ECC + CRC on outgoing packets; leave source-select at the
   * reset default (TCON-driven).  EoTp matches the LP-init path (off).
   */

  t113_dsi_putreg(T113_DSI_BASIC_CTL0_OFFSET,
                  T113_DSI_BCTL0_ECC_EN | T113_DSI_BCTL0_CRC_EN);

  /* Enter video sync-pulse mode and program the start delay. */

  reg  = T113_DSI_BCTL1_DSI_MODE | T113_DSI_BCTL1_VFR_START |
         T113_DSI_BCTL1_VPMA;
  reg |= (vsd << T113_DSI_BCTL1_VSD_SHIFT) & T113_DSI_BCTL1_VSD_MASK;
  t113_dsi_putreg(T113_DSI_BASIC_CTL1_OFFSET, reg);

  /* Pixel transmit timing: 10-cycle TRANS_START gap, no HS-zero shrink.
   * TCON_DRQ uses fixed-set mode only when the back-porch budget is
   * generous (>=21 cycles); X4B's 20-cycle gap falls under that so
   * DRQ_SET stays at 0 and the auto DRQ mode is left enabled.
   */

  t113_dsi_putreg(T113_DSI_TRANS_START_OFFSET,
                  (10u << T113_DSI_TRANS_START_SET_SHIFT) &
                  T113_DSI_TRANS_START_SET_MASK);
  t113_dsi_putreg(T113_DSI_TRANS_ZERO_OFFSET, 0);

  if ((uint32_t)v->htotal - v->hactive - v->hbp < 21u)
    {
      t113_dsi_putreg(T113_DSI_TCON_DRQ_OFFSET, 0);
    }
  else
    {
      uint32_t drq;

      drq  = ((uint32_t)v->htotal - v->hactive - v->hbp - 20u) * bpp / 32u;
      reg  = T113_DSI_TCON_DRQ_MODE;
      reg |= (drq << T113_DSI_TCON_DRQ_SET_SHIFT) &
             T113_DSI_TCON_DRQ_SET_MASK;
      t113_dsi_putreg(T113_DSI_TCON_DRQ_OFFSET, reg);
    }

  /* Pixel data path: format code, plug-disconnect-detect off (the field
   * actually means "leave LP tail off the trailing HSE so the HS clock
   * keeps running into the next line").
   */

  reg  = ((uint32_t)(8u + v->format) << T113_DSI_PIXEL_CTL0_FMT_SHIFT) &
         T113_DSI_PIXEL_CTL0_FMT_MASK;
  reg |= T113_DSI_PIXEL_CTL0_PD_PLUG_DIS;
  t113_dsi_putreg(T113_DSI_PIXEL_CTL0_OFFSET, reg);

  /* Pixel-packet header used on every active line.  Note: WC field
   * unit is hardware-specific; using `hact` directly produced visible
   * (mis-aligned) output, while hact*bpp/8 produced no output at all.
   */

  ph  = (uint32_t)pixel_dt[v->format] |
        ((hact & 0xffffu) << T113_DSI_PPH_WC_SHIFT);
  ph |= (uint32_t)t113_dsi_ecc(ph) << T113_DSI_BLK_ECC_SHIFT;
  t113_dsi_putreg(T113_DSI_PIXEL_PH_OFFSET, ph);
  t113_dsi_putreg(T113_DSI_PIXEL_PD_OFFSET, 0);
  t113_dsi_putreg(T113_DSI_PIXEL_PF0_OFFSET, 0xffffu);
  t113_dsi_putreg(T113_DSI_PIXEL_PF1_OFFSET, 0xffffu | (0xffffu << 16));

  /* Sync short packets - DT carries the event, payload bytes are zero. */

  t113_dsi_putreg(T113_DSI_SYNC_HSS_OFFSET,
                  t113_dsi_short_packet(T113_DSI_DT_HSS));
  t113_dsi_putreg(T113_DSI_SYNC_HSE_OFFSET,
                  t113_dsi_short_packet(T113_DSI_DT_HSE));
  t113_dsi_putreg(T113_DSI_SYNC_VSS_OFFSET,
                  t113_dsi_short_packet(T113_DSI_DT_VSS));
  t113_dsi_putreg(T113_DSI_SYNC_VSE_OFFSET,
                  t113_dsi_short_packet(T113_DSI_DT_VSE));

  /* Blanking long packets - header + zero-payload CRC. */

  t113_dsi_blk_long(T113_DSI_BLK_HSA0_OFFSET,
                    T113_DSI_BLK_HSA1_OFFSET,  hsa);
  t113_dsi_blk_long(T113_DSI_BLK_HBP0_OFFSET,
                    T113_DSI_BLK_HBP1_OFFSET,  hbp);
  t113_dsi_blk_long(T113_DSI_BLK_HFP0_OFFSET,
                    T113_DSI_BLK_HFP1_OFFSET,  hfp);
  t113_dsi_blk_long(T113_DSI_BLK_HBLK0_OFFSET,
                    T113_DSI_BLK_HBLK1_OFFSET, hblk);
  t113_dsi_blk_long(T113_DSI_BLK_VBLK0_OFFSET,
                    T113_DSI_BLK_VBLK1_OFFSET, vblk);

  /* JUMP_CFG0: the HSC chain runs once and then the NOP -> HSCEXIT jump
   * settles the clock lane.  EN=0 -> fire only when LOOP_NUM expires.
   */

  reg  = (1u << T113_DSI_JUMP_CFG_NUM_SHIFT) & T113_DSI_JUMP_CFG_NUM_MASK;
  reg |= ((uint32_t)T113_DSI_ID_NOP << T113_DSI_JUMP_CFG_POINT_SHIFT) &
         T113_DSI_JUMP_CFG_POINT_MASK;
  reg |= ((uint32_t)T113_DSI_ID_HSCEXIT << T113_DSI_JUMP_CFG_TO_SHIFT) &
         T113_DSI_JUMP_CFG_TO_MASK;
  t113_dsi_putreg(T113_DSI_INST_JUMP_CFG0_OFFSET, reg);

  /* Stage 1 - pulse the HSC chain (LP11 -> HSC -> END) so the clock lane
   * locks in HS continuous mode before the first pixel goes out.
   *
   * INST_ST: clear baseline first, then rising edge to 1.  The hardware
   * self-clears INST_ST when the chain finishes; an active 1->0 write
   * here would interrupt the in-flight chain before it completes.
   */

  t113_dsi_putreg(T113_DSI_INST_JUMP_SEL_OFFSET,
                  ((uint32_t)T113_DSI_ID_HSC << (4 * T113_DSI_ID_LP11)) |
                  ((uint32_t)T113_DSI_ID_END << (4 * T113_DSI_ID_HSC)));

  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              T113_DSI_BCTL0_INST_ST, 0);
  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET, 0,
              T113_DSI_BCTL0_INST_ST);

  /* DO NOT clear LP11 LANE_CEN here.  Vendor de_dsi.c:165-166 sets
   * lane_cen=1 in dsi_start(HSC) and never clears it again - keeping
   * CK driven through LP11 slots is exactly what holds the HS continuous
   * clock.  Clearing it here causes DBG0 high byte to drop 0x11 -> 0x01
   * (CK falls out of HS continuous), and D0/D1 never reach HS active.
   */

  /* Stage 2 - pulse the HSD chain (LP11 -> NOP -> HSD -> DLY -> NOP and
   * out via HSCEXIT -> END) to begin streaming pixels.  Same INST_ST
   * polarity contract as stage 1: clear baseline, then rising edge.
   */

  t113_dsi_putreg(T113_DSI_INST_JUMP_SEL_OFFSET,
                  ((uint32_t)T113_DSI_ID_NOP << (4 * T113_DSI_ID_LP11)) |
                  ((uint32_t)T113_DSI_ID_HSD << (4 * T113_DSI_ID_NOP)) |
                  ((uint32_t)T113_DSI_ID_DLY << (4 * T113_DSI_ID_HSD)) |
                  ((uint32_t)T113_DSI_ID_NOP << (4 * T113_DSI_ID_DLY)) |
                  ((uint32_t)T113_DSI_ID_END << (4 * T113_DSI_ID_HSCEXIT)));

  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET,
              T113_DSI_BCTL0_INST_ST, 0);
  modifyreg32(T113_DSI_BASE + T113_DSI_BASIC_CTL0_OFFSET, 0,
              T113_DSI_BCTL0_INST_ST);

  lcdinfo("DSI HS video started: %ux%u, %u lane(s), %u bpp\n",
          (unsigned)v->hactive, (unsigned)v->vactive,
          (unsigned)v->lanes, (unsigned)bpp);
  return OK;
}

/****************************************************************************
 * Name: t113_dsi_get_host
 *
 * Description:
 *   Return the host handle to the panel driver / board glue.  NULL until
 *   t113_dsi_initialize() has succeeded.
 *
 ****************************************************************************/

struct mipi_dsi_host *t113_dsi_get_host(void)
{
  return g_t113_dsi.initialized ? &g_t113_dsi.host : NULL;
}
