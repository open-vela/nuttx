/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_mbox.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 Mailbox V2 and mailbox-UART wire ABI.
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_MBOX_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_MBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_MBOX_FIFO_SIZE       8u
#define BK7258_MBOX_IRQ             79
#define BK7258_MBOX_CMD_FIFO        1u
#define BK7258_MBOX_ACK_FIFO        0u

#define BK7258_MBOX_REG(n)          (BK7258_MBOX0_BASE + ((n) * 4u))
#define BK7258_MBOX_CTRL            BK7258_MBOX_REG(0x02)
#define BK7258_MBOX_CH0_STATUS      BK7258_MBOX_REG(0x18)
#define BK7258_MBOX_CH1_CFG         BK7258_MBOX_REG(0x20)
#define BK7258_MBOX_CH1_FIFO_CFG    BK7258_MBOX_REG(0x21)
#define BK7258_MBOX_CH1_TDATA0      BK7258_MBOX_REG(0x22)
#define BK7258_MBOX_CH1_TDATA1      BK7258_MBOX_REG(0x23)
#define BK7258_MBOX_CH1_TID         BK7258_MBOX_REG(0x24)
#define BK7258_MBOX_CH1_SID         BK7258_MBOX_REG(0x25)
#define BK7258_MBOX_CH1_RDATA0      BK7258_MBOX_REG(0x26)
#define BK7258_MBOX_CH1_RDATA1      BK7258_MBOX_REG(0x27)
#define BK7258_MBOX_CH1_STATUS      BK7258_MBOX_REG(0x28)
#define BK7258_MBOX_CFG_INT_EN         (1u << 8)
#define BK7258_MBOX_CFG_WRERR_EN       (1u << 9)
#define BK7258_MBOX_CFG_RDERR_EN       (1u << 10)
#define BK7258_MBOX_CFG_WRFULL_EN      (1u << 11)
#define BK7258_MBOX_CFG_WRERR_STATUS   (1u << 16)
#define BK7258_MBOX_CFG_RDERR_STATUS   (1u << 17)
#define BK7258_MBOX_CFG_WRFULL_STATUS  (1u << 18)
#define BK7258_MBOX_CFG_ERROR_STATUS   \
  (BK7258_MBOX_CFG_WRERR_STATUS | BK7258_MBOX_CFG_RDERR_STATUS | \
   BK7258_MBOX_CFG_WRFULL_STATUS)
#define BK7258_MBOX_CFG_RW_MASK        0x00000fffu

#define BK7258_MB_MESSAGE_SIZE      16u
#define BK7258_MB_HEADER_CMD_SHIFT  0u
#define BK7258_MB_HEADER_CMD_MASK   (0xffu << BK7258_MB_HEADER_CMD_SHIFT)
#define BK7258_MB_HEADER_STATE_SHIFT 8u
#define BK7258_MB_HEADER_STATE_MASK (0x0fu << BK7258_MB_HEADER_STATE_SHIFT)
#define BK7258_MB_HEADER_CTRL_SHIFT 12u
#define BK7258_MB_HEADER_CTRL_MASK  (0x0fu << BK7258_MB_HEADER_CTRL_SHIFT)
#define BK7258_MB_HEADER_SEQ_SHIFT  16u
#define BK7258_MB_HEADER_SEQ_MASK   (0xffu << BK7258_MB_HEADER_SEQ_SHIFT)
#define BK7258_MB_HEADER_CHAN_SHIFT 24u
#define BK7258_MB_HEADER_CHAN_MASK  (0xffu << BK7258_MB_HEADER_CHAN_SHIFT)

static_assert((BK7258_MB_HEADER_CMD_MASK |
                BK7258_MB_HEADER_STATE_MASK |
                BK7258_MB_HEADER_CTRL_MASK |
                BK7258_MB_HEADER_SEQ_MASK |
                BK7258_MB_HEADER_CHAN_MASK) == UINT32_MAX,
               "mailbox header masks must cover exactly 32 bits");
static_assert((BK7258_MB_HEADER_STATE_MASK &
                BK7258_MB_HEADER_CTRL_MASK) == 0 &&
               (BK7258_MB_HEADER_SEQ_MASK &
                BK7258_MB_HEADER_CHAN_MASK) == 0,
               "mailbox header masks overlap");

#define BK7258_MB_CTRL_ACK_BOX      0x01u
#define BK7258_MB_CTRL_SYNC_TX      0x02u
#define BK7258_MB_CTRL_RESET        0x04u
#define BK7258_MB_STATE_COM_FAIL    0x01u

/* Armino's ACK_STATE_COMPLETE.  This is mb_chnl_ack_t.ack_state in mailbox
 * word 3.  It is deliberately not a BK7258_MB_STATE_* value: hdr.state only
 * accepts CHNL_STATE_COM_FAIL, and the CP rejects any other bit before the
 * mb_ipc completion callback can clear STATE_RX_IN_PROCESS.
 */

#define BK7258_MB_ACK_STATE_COMPLETE 0x02u

#define BK7258_MB_CHAN_HW_CTRL_TX   0x10u

/* mb_ipc's socket router.  Index 1 in the vendor's channel enum
 * (include/driver/mailbox_channel.h: MB_CHNL_HW_CTRL, CP0_MB_CHNL_IPC,
 * MB_CHNL_PWC, ...), which is what the CP's flash server listens on.
 */

#define BK7258_MB_CHAN_IPC_TX       0x11u
#define BK7258_MB_CHAN_PWC_TX       0x12u
#define BK7258_MB_CHAN_BT_TX        0x13u
#define BK7258_MB_CHAN_WIFI_CMD_TX  0x14u
#define BK7258_MB_CHAN_WIFI_DATA_TX 0x15u
#define BK7258_MB_CHAN_UART0_TX     0x19u

/* The flash operation notification, index 11 in the same enum
 * (MB_CHNL_FLASH).  The CP raises it before and after every flash access it
 * performs -- cp/middleware/driver/flash/flash_notify.c
 * send_flash_op_state()
 * -- and then spins for up to 5ms waiting for an acknowledgement whose
 * ack_data1 reads IPC_FLASH_OP_ACK.  Only the RX direction is used here: the
 * CP is always the one that announces, this core only answers.
 */

#define BK7258_MB_CHAN_FLASH_TX     0x1bu
#define BK7258_MB_CHAN_IPC_RX       0x41u
#define BK7258_MB_CHAN_PWC_RX       0x42u
#define BK7258_MB_CHAN_BT_RX        0x43u
#define BK7258_MB_CHAN_WIFI_CMD_RX  0x44u
#define BK7258_MB_CHAN_WIFI_DATA_RX 0x45u
#define BK7258_MB_CHAN_UART0_RX     0x49u
#define BK7258_MB_CHAN_FLASH_RX     0x4bu
#define BK7258_MB_CHAN_SARADC_RX    0x4cu

#define BK7258_MB_UART_DATA         0u
#define BK7258_MB_UART_STATE        1u
#define BK7258_MB_UART_CHUNK_SIZE   128u

/* These values are generated by the app_ab RAM layout.  The linker script
 * independently asserts the complete SWAP reservation.
 */

#define BK7258_CP_RAM_START         0x28064000u
#define BK7258_CP_RAM_END           0x2809f700u
#define BK7258_SWAP_BASE            0x2809f800u
#define BK7258_SWAP_SIZE            0x00000800u
#define BK7258_IPC_TX_ADDRESS       0x2809f900u
#define BK7258_IPC_TX_SIZE          0x00000080u
/* The flash service's 16-byte descriptor.  It has to live in SWAP, not in
 * the
 * AP's own RAM: the frame only carries a pointer, and the CP has to be able
 * to
 * read what it points at.  A descriptor in AP RAM produced no answer at all
 * from the server -- the request was accepted by the router and then
 * silently
 * dropped.
 */

#define BK7258_FLASH_IPC_ADDRESS    0x2809f980u
#define BK7258_FLASH_IPC_SIZE       0x00000020u

/* Payload staging for the flash service, one protocol chunk (512 bytes, the
 * FLASH_IPC_READ_SIZE/FLASH_IPC_WRITE_SIZE the vendor's flash_ipc.h fixes).
 *
 * A write frame carries only a pointer, and the CP dereferences it directly
 * (flash_server.c flash_write_handler() memcpy's from cmd_buff->buff and
 * then
 * re-computes the CRC over it).  A pointer into the AP's own heap is no
 * good:
 * the descriptor itself had to be moved here for exactly that reason -- see
 * BK7258_FLASH_IPC_ADDRESS -- so the bytes it points at have to be here too.
 */

#define BK7258_FLASH_DATA_ADDRESS   0x2809fa00u
#define BK7258_FLASH_DATA_SIZE      0x00000200u

#define BK7258_MB_UART_RX_ADDRESS   0x2809fc00u
#define BK7258_MB_UART_TX_ADDRESS   0x2809fd00u
#define BK7258_MB_SHARED_TX_START   BK7258_IPC_TX_ADDRESS
#define BK7258_MB_SHARED_TX_SIZE    \
  (BK7258_MB_UART_TX_ADDRESS + BK7258_MB_UART_CHUNK_SIZE - \
   BK7258_MB_SHARED_TX_START)

/* The contiguous run of SWAP this core writes and the CP reads: the IPC TX
 * frame, then the flash descriptor, then the flash payload staging buffer,
 * ending where the read-only UART RX window begins.  bk7258_start.c maps
 * this
 * as one MPU region, which is what keeps it non-cacheable -- without a
 * region
 * the addresses only work by falling back to the default memory map, and
 * that
 * map calls this range cacheable.
 */

#define BK7258_MB_SHARED_RW_START   BK7258_IPC_TX_ADDRESS
#define BK7258_MB_SHARED_RW_SIZE    \
  (BK7258_MB_UART_RX_ADDRESS - BK7258_MB_SHARED_RW_START)

static_assert(BK7258_IPC_TX_ADDRESS >= BK7258_SWAP_BASE &&
               BK7258_IPC_TX_ADDRESS + BK7258_IPC_TX_SIZE <=
               BK7258_SWAP_BASE + BK7258_SWAP_SIZE,
               "IPC TX buffer lies outside SWAP");
static_assert((BK7258_IPC_TX_ADDRESS & 31u) == 0,
               "IPC TX buffer must be cache-line aligned");
static_assert(BK7258_MB_SHARED_TX_START + BK7258_MB_SHARED_TX_SIZE <=
               BK7258_SWAP_BASE + BK7258_SWAP_SIZE,
               "mailbox shared TX window lies outside SWAP");
static_assert(BK7258_MB_UART_RX_ADDRESS >= BK7258_SWAP_BASE &&
               BK7258_MB_UART_RX_ADDRESS + BK7258_MB_UART_CHUNK_SIZE <=
               BK7258_SWAP_BASE + BK7258_SWAP_SIZE,
               "mailbox UART RX lies outside SWAP");
static_assert(BK7258_MB_UART_TX_ADDRESS >= BK7258_SWAP_BASE &&
               BK7258_MB_UART_TX_ADDRESS + BK7258_MB_UART_CHUNK_SIZE <=
               BK7258_SWAP_BASE + BK7258_SWAP_SIZE,
               "mailbox UART TX lies outside SWAP");
static_assert((BK7258_MB_UART_RX_ADDRESS & 31u) == 0 &&
               (BK7258_MB_UART_TX_ADDRESS & 31u) == 0,
               "mailbox UART buffers must be cache-line aligned");
static_assert(BK7258_FLASH_IPC_ADDRESS >= BK7258_MB_SHARED_RW_START &&
               BK7258_FLASH_IPC_ADDRESS + BK7258_FLASH_IPC_SIZE <=
               BK7258_MB_SHARED_RW_START + BK7258_MB_SHARED_RW_SIZE,
               "flash descriptor lies outside the shared RW window");
static_assert(BK7258_FLASH_DATA_ADDRESS >= BK7258_MB_SHARED_RW_START &&
               BK7258_FLASH_DATA_ADDRESS + BK7258_FLASH_DATA_SIZE <=
               BK7258_MB_SHARED_RW_START + BK7258_MB_SHARED_RW_SIZE,
               "flash payload staging lies outside the shared RW window");
static_assert(BK7258_FLASH_IPC_ADDRESS >=
               BK7258_IPC_TX_ADDRESS + BK7258_IPC_TX_SIZE &&
               BK7258_FLASH_DATA_ADDRESS >=
               BK7258_FLASH_IPC_ADDRESS + BK7258_FLASH_IPC_SIZE,
               "flash windows overlap the IPC TX frame or each other");
static_assert((BK7258_FLASH_IPC_ADDRESS & 31u) == 0 &&
               (BK7258_FLASH_DATA_ADDRESS & 31u) == 0,
               "flash windows must be cache-line aligned");
static_assert(BK7258_MB_SHARED_RW_START >= BK7258_SWAP_BASE &&
               BK7258_MB_SHARED_RW_START + BK7258_MB_SHARED_RW_SIZE <=
               BK7258_SWAP_BASE + BK7258_SWAP_SIZE,
               "shared RW window lies outside SWAP");

struct bk7258_mb_wire_message
{
  uint32_t header;
  uint32_t payload_address;
  uint16_t payload_length;
  uint8_t flags;
  uint8_t crc8;
  uint32_t reserved;
};

static_assert(sizeof(struct bk7258_mb_wire_message) ==
               BK7258_MB_MESSAGE_SIZE, "mailbox message ABI size");
static_assert(offsetof(struct bk7258_mb_wire_message, header) == 0,
               "mailbox header ABI offset");
static_assert(offsetof(struct bk7258_mb_wire_message, payload_address) == 4,
               "mailbox payload ABI offset");
static_assert(offsetof(struct bk7258_mb_wire_message, payload_length) == 8,
               "mailbox length ABI offset");
static_assert(offsetof(struct bk7258_mb_wire_message, flags) == 10,
               "mailbox flags ABI offset");
static_assert(offsetof(struct bk7258_mb_wire_message, crc8) == 11,
               "mailbox CRC ABI offset");
static_assert(offsetof(struct bk7258_mb_wire_message, reserved) == 12,
               "mailbox reserved ABI offset");

static inline uint8_t
bk7258_mb_header_cmd(const struct bk7258_mb_wire_message *message)
{
  return (message->header & BK7258_MB_HEADER_CMD_MASK) >>
         BK7258_MB_HEADER_CMD_SHIFT;
}

static inline uint8_t
bk7258_mb_header_state(const struct bk7258_mb_wire_message *message)
{
  return (message->header & BK7258_MB_HEADER_STATE_MASK) >>
         BK7258_MB_HEADER_STATE_SHIFT;
}

static inline uint8_t
bk7258_mb_header_ctrl(const struct bk7258_mb_wire_message *message)
{
  return (message->header & BK7258_MB_HEADER_CTRL_MASK) >>
         BK7258_MB_HEADER_CTRL_SHIFT;
}

static inline uint8_t
bk7258_mb_header_seq(const struct bk7258_mb_wire_message *message)
{
  return (message->header & BK7258_MB_HEADER_SEQ_MASK) >>
         BK7258_MB_HEADER_SEQ_SHIFT;
}

static inline uint8_t
bk7258_mb_header_channel(const struct bk7258_mb_wire_message *message)
{
  return (uint8_t)((message->header & BK7258_MB_HEADER_CHAN_MASK) >>
                   BK7258_MB_HEADER_CHAN_SHIFT);
}

static inline uint32_t bk7258_mb_make_header(uint8_t command, uint8_t state,
                                             uint8_t control,
                                             uint8_t sequence,
                                             uint8_t channel)
{
  return ((uint32_t)command << BK7258_MB_HEADER_CMD_SHIFT) |
         ((uint32_t)(state & 0x0fu) << BK7258_MB_HEADER_STATE_SHIFT) |
         ((uint32_t)(control & 0x0fu) << BK7258_MB_HEADER_CTRL_SHIFT) |
         ((uint32_t)sequence << BK7258_MB_HEADER_SEQ_SHIFT) |
         ((uint32_t)channel << BK7258_MB_HEADER_CHAN_SHIFT);
}

static inline uint32_t bk7258_mb_make_ack_header(
    const struct bk7258_mb_wire_message *request, bool failed)
{
  return bk7258_mb_make_header(
    bk7258_mb_header_cmd(request),
    failed ? BK7258_MB_STATE_COM_FAIL : 0u,
    BK7258_MB_CTRL_ACK_BOX,
    bk7258_mb_header_seq(request),
    bk7258_mb_header_channel(request));
}

#define BK7258_MB_IPC_RSP_FLAG 0x80u

static inline uint8_t
bk7258_mb_ipc_tag(const struct bk7258_mb_wire_message *message)
{
  return (uint8_t)((message->payload_address >> 16) & 0xffu);
}

static inline bool bk7258_mb_ipc_is_response(
    const struct bk7258_mb_wire_message *message, uint8_t command,
    uint8_t tag)
{
  return bk7258_mb_header_cmd(message) ==
           (uint8_t)(command | BK7258_MB_IPC_RSP_FLAG) &&
         bk7258_mb_ipc_tag(message) == tag;
}

static inline void bk7258_mb_ipc_make_response(
    const struct bk7258_mb_wire_message *request,
    struct bk7258_mb_wire_message *response)
{
  uint32_t param1 = request->payload_address;

  response->header = bk7258_mb_make_header(
    (uint8_t)(bk7258_mb_header_cmd(request) | BK7258_MB_IPC_RSP_FLAG),
    0u, 0u, 0u, 0u);
  response->payload_address = ((param1 & 0x000000ffu) << 8) |
                              ((param1 & 0x0000ff00u) >> 8) |
                              (param1 & 0x00ff0000u);
  response->payload_length = request->payload_length;
  response->flags = request->flags;
  response->crc8 = request->crc8;
  response->reserved = request->reserved;
}

typedef struct
{
  uint8_t src_cpu;
  uint8_t dest_cpu;
  uint32_t data[2];
} bk7258_mbox_message_t;

struct bk7258_mbox_stats
{
  uint32_t rx_messages;
  uint32_t bad_source;
  uint32_t bad_length;
  uint32_t bad_address;
  uint32_t write_error;
  uint32_t read_error;
  uint32_t write_full;
  uint32_t descriptor_full;
  uint32_t descriptor_deferred;
};

enum bk7258_mb_link_state
{
  BK7258_MB_LINK_DOWN = 0,
  BK7258_MB_LINK_ABORTING,
  BK7258_MB_LINK_PROBING,
  BK7258_MB_LINK_READY,
  BK7258_MB_LINK_QUIESCING
};

typedef int (*bk7258_mbox_callback_t)
  (const struct bk7258_mb_wire_message *message);
typedef int (*bk7258_mb_channel_rx_t)
  (const struct bk7258_mb_wire_message *message, uint8_t *ack_flags,
    void *arg);
typedef void (*bk7258_mb_tx_complete_t)
  (const struct bk7258_mb_wire_message *ack, int result, void *arg);
typedef void (*bk7258_mb_link_callback_t)
  (enum bk7258_mb_link_state state, void *arg);
typedef void (*bk7258_mb_uart_callback_t)(void *arg);

int bk7258_mbox_init(void);
int bk7258_mbox_send(uint8_t destination, const uint32_t data[2]);
uint32_t bk7258_mbox_rx_status(void);
void bk7258_mbox_set_callback(bk7258_mbox_callback_t callback);
void bk7258_mbox_get_stats(struct bk7258_mbox_stats *stats);
void bk7258_mbox_kick_rx(void);
void bk7258_mbox_discard_deferred(void);

int bk7258_mailbox_init(void);
int bk7258_mailbox_start(void);
int bk7258_mailbox_send_wire(uint8_t logical_channel,
                             const struct bk7258_mb_wire_message *message,
                             bk7258_mb_tx_complete_t callback, void *arg);
int bk7258_mailbox_send_raw(uint8_t logical_channel,
                            const uint8_t frame[BK7258_MB_MESSAGE_SIZE],
                            bk7258_mb_tx_complete_t callback, void *arg);
int bk7258_mailbox_register_rx(uint8_t logical_channel,
                               bk7258_mb_channel_rx_t callback, void *arg);
int bk7258_mailbox_start_probe(void);
void bk7258_mailbox_probe_complete(bool ready);
void bk7258_mailbox_force_reset(void);
enum bk7258_mb_link_state bk7258_mailbox_link_state(void);
bool bk7258_mailbox_link_ready(void);
int bk7258_mailbox_wait_link_ready(unsigned int timeout_ms);
void bk7258_mailbox_set_link_callback(bk7258_mb_link_callback_t callback,
                                       void *arg);
uint32_t bk7258_mailbox_peer_reset_generation(void);

/* Compatibility APIs used by the existing HW_CTRL and PWC services. */

int bk7258_mbox_send_message(uint8_t command, uint8_t logical_channel,
                             uint32_t param1, uint32_t param2,
                             uint32_t param3);
int bk7258_mailbox_send_pwc(uint8_t command, uint32_t p1, uint32_t p2,
                            uint32_t p3);
void bk7258_mailbox_set_pwc_rx(int (*callback)(const void *message));
int bk7258_mailbox_wait_hw_control(unsigned int timeout_ms);
int bk7258_mailbox_wait_pwc(unsigned int timeout_ms);
void bk7258_mailbox_dump_stats(void);
int bk7258_ipc_heartbeat_start(void);
void bk7258_ipc_heartbeat_poll(void);
int bk7258_pwc_start(void);
int bk7258_pwc_psram_start(void);

int bk7258_mb_uart_init(void);
void bk7258_mb_uart_start(void);
void bk7258_mb_uart_request_state(void);
ssize_t bk7258_mbox_uart_write(const uint8_t *data, size_t length);

/* Latch "this system is crashing", which is what arms
 * bk7258_mbox_uart_drain_polled().  One way: nothing clears it, because
 * nothing recovers from what sets it.  Called from board_autoled_on() for
 * LED_ASSERTION and LED_PANIC, which _assert() raises before it produces any
 * output.
 */

void bk7258_mbox_uart_crash_mode(void);

/* Drain the console TX ring to the hardware with register writes and bounded
 * spins only.  For crash paths: safe from interrupt context and with
 * interrupts disabled, where the worker that normally does this cannot run.
 *
 * Does nothing until bk7258_mbox_uart_crash_mode() has been called.  That
 * gate
 * is not advisory -- this function bypasses the transport's sequencing and
 * reuses its staging buffer, so running it beside a live transport corrupts
 * the link.  Best-effort by construction; see the implementation.
 */

void bk7258_mbox_uart_drain_polled(void);
ssize_t bk7258_mbox_uart_read(uint8_t *data, size_t length,
                              unsigned int *status);
void bk7258_mbox_uart_early_init(void);
bool bk7258_mbox_uart_rxavailable(void);
bool bk7258_mbox_uart_txready(void);
bool bk7258_mbox_uart_txempty(void);
void bk7258_mbox_uart_rxflowcontrol(bool upper);
void bk7258_mbox_uart_set_callback(bk7258_mb_uart_callback_t callback,
                                   void *arg);
int bk7258_mbox_uart_flush(unsigned int timeout_ms);
void bk7258_mbox_uart_dump_stats(void);

#endif
