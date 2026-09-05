#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_SDMMC_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_SDMMC_H

#include <nuttx/config.h>

#ifdef CONFIG_ESP32P4_SDMMC

#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>

#define ESP32P4_SDMMC_SLOT0     0
#define ESP32P4_SDMMC_SLOT1     1

/* Slot0 默认 IOMUX 引脚（ESP32-P4 Function EV Board） */
#define ESP32P4_SDMMC0_CLK      43
#define ESP32P4_SDMMC0_CMD      44
#define ESP32P4_SDMMC0_D0       39
#define ESP32P4_SDMMC0_D1       40
#define ESP32P4_SDMMC0_D2       41
#define ESP32P4_SDMMC0_D3       42
#define ESP32P4_SDMMC0_D4       45
#define ESP32P4_SDMMC0_D5       46
#define ESP32P4_SDMMC0_D6       47
#define ESP32P4_SDMMC0_D7       48

/* 时钟源 160MHz PLL，目标频率 */
#define SDMMC_CLK_SRC_KHZ       160000
#define SDMMC_INIT_KHZ          400
#define SDMMC_DEFAULT_KHZ       20000
#define SDMMC_HIGHSPEED_KHZ     40000

/* DMA 描述符数量（单块传输 1 个即可，多块预留） */
#define SDMMC_DMA_DESC_NUM      8

/* 描述符需要对齐到 64 字节边界（IDMAC 要求，与 HAL sdmmc_desc_t 一致） */
struct esp32p4_sdmmc_dmadesc_s
{
  uint32_t ctrl;           /* owned_by_idmac, first/last descriptor 等 */
  uint32_t size;           /* buffer1_size, buffer2_size */
  void *buf_addr;          /* buffer1_ptr */
  void *next_desc;         /* next_desc_ptr (second_address_chained 模式) */
  uint32_t reserved[12];   /* cache 对齐填充，使结构体为 64 字节 */
} __attribute__((aligned(64)));

struct esp32p4_sdmmc_state_s
{
  struct sdio_dev_s dev;
  int slot;
  bool initialized;
  bool widebus;
  uint32_t freq_khz;
  void *ldo_config;
  int cpuint;
  sem_t sem;
  volatile uint32_t pending;
  sdio_eventset_t waitevents;
  sdio_eventset_t wkupevent;
  worker_t callback;
  void *callbackarg;
  struct esp32p4_sdmmc_dmadesc_s dma_desc[SDMMC_DMA_DESC_NUM]
    __attribute__((aligned(64)));

  /* PIO 传输状态 */
    uint8_t *dma_buffer;        /* 当前搬运指针（会推进） */
    uint8_t *dma_orig_buffer;   /* 原始 buffer 基址（用于 cache invalidate） */
    size_t   dma_nbytes;        /* 剩余字节数 */
    size_t   dma_orig_nbytes;   /* 原始总字节数 */
    bool     dma_is_tx;         /* true=写, false=读 */

  /* 64 字节对齐的 bounce buffer，用于非对齐地址的 DMA 中转 */
  uint8_t bounce_buf[4096] __attribute__((aligned(64)));
  bool dma_use_bounce;
};

int esp32p4_sdmmc_initialize(int slot, int minor);

#endif /* CONFIG_ESP32P4_SDMMC */
#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_SDMMC_H */
