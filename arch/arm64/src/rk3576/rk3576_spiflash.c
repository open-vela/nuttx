/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_spiflash.c - SPI NOR Flash driver
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_spiflash.h"
#include "rk3576_spiflash.h"

#define SF_TIMEOUT 10000

static int rk3576_sf_wait_ready(void)
{
  int timeout = SF_TIMEOUT;
  while (timeout--)
    {
      if (!(getreg32(RK3576_SPIFLASH_ADDR + SF_STATE) & SF_SR_IS_BUSY))
        return OK;
      up_udelay(1);
    }
  return -ETIMEDOUT;
}

static void rk3576_sf_write_enable(void)
{
  putreg32(0x06, RK3576_SPIFLASH_ADDR + SF_DATA);
  putreg32(1, RK3576_SPIFLASH_ADDR + SF_LEN);
  putreg32(SF_CTRL_EN | SF_CTRL_DIR_WRITE, RK3576_SPIFLASH_ADDR + SF_CTRL);
  rk3576_sf_wait_ready();
}

void rk3576_spiflash_init(void)
{
  putreg32(0, RK3576_SPIFLASH_ADDR + SF_CTRL);
  uinfo("SPI Flash: initialized\n");
}

void rk3576_spiflash_read_id(uint8_t *mfg, uint8_t *type)
{
  putreg32(0x9f, RK3576_SPIFLASH_ADDR + SF_DATA);
  putreg32(3, RK3576_SPIFLASH_ADDR + SF_LEN);
  putreg32(SF_CTRL_EN | SF_CTRL_DIR_READ, RK3576_SPIFLASH_ADDR + SF_CTRL);
  rk3576_sf_wait_ready();
  uint32_t data = getreg32(RK3576_SPIFLASH_ADDR + SF_DATA);
  if (mfg) *mfg = (uint8_t)(data & 0xff);
  if (type) *type = (uint8_t)((data >> 8) & 0xff);
  uinfo("SPI Flash: ID %02x %02x\n", mfg ? *mfg : 0, type ? *type : 0);
}

int rk3576_spiflash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  putreg32(0x03, RK3576_SPIFLASH_ADDR + SF_DATA);
  putreg32(addr, RK3576_SPIFLASH_ADDR + SF_ADDR);
  putreg32(len, RK3576_SPIFLASH_ADDR + SF_LEN);
  putreg32(SF_CTRL_EN | SF_CTRL_DIR_READ, RK3576_SPIFLASH_ADDR + SF_CTRL);
  rk3576_sf_wait_ready();
  for (uint32_t i = 0; i < len; i += 4)
    {
      uint32_t word = getreg32(RK3576_SPIFLASH_ADDR + SF_DATA);
      buf[i] = word & 0xff;
      if (i + 1 < len) buf[i + 1] = (word >> 8) & 0xff;
      if (i + 2 < len) buf[i + 2] = (word >> 16) & 0xff;
      if (i + 3 < len) buf[i + 3] = (word >> 24) & 0xff;
    }
  return OK;
}

int rk3576_spiflash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  rk3576_sf_write_enable();
  for (uint32_t i = 0; i < len; i += 4)
    {
      uint32_t word = buf[i] | (buf[i + 1] << 8) |
                     (buf[i + 2] << 16) | (buf[i + 3] << 24);
      putreg32(word, RK3576_SPIFLASH_ADDR + SF_DATA);
    }
  putreg32(0x02, RK3576_SPIFLASH_ADDR + SF_DATA);
  putreg32(addr, RK3576_SPIFLASH_ADDR + SF_ADDR);
  putreg32(len, RK3576_SPIFLASH_ADDR + SF_LEN);
  putreg32(SF_CTRL_EN | SF_CTRL_DIR_WRITE, RK3576_SPIFLASH_ADDR + SF_CTRL);
  rk3576_sf_wait_ready();
  return OK;
}

int rk3576_spiflash_erase(uint32_t addr, uint32_t len)
{
  rk3576_sf_write_enable();
  putreg32(0xd8, RK3576_SPIFLASH_ADDR + SF_DATA);
  putreg32(addr, RK3576_SPIFLASH_ADDR + SF_ADDR);
  putreg32(len, RK3576_SPIFLASH_ADDR + SF_LEN);
  putreg32(SF_CTRL_EN | SF_CTRL_DIR_WRITE, RK3576_SPIFLASH_ADDR + SF_CTRL);
  rk3576_sf_wait_ready();
  return OK;
}
