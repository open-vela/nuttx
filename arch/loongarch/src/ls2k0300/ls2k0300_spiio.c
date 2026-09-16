#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_spiio.h"

#if defined(CONFIG_LS2K0300_SPIIO)

/* SR1 W1C flags from test code: EOT | bit11 | bit10 | bit9 | bit8 */

#define SR1_W1C_FLAGS  (SR1_EOT | (1U << 11) | (1U << 10) | (1U << 9) | (1U << 8))

#define SPIIO_CLOCK_FREQ 200000000UL
#define SPIIO_TIMEOUT     10000    /* matched to test code SPI_WAIT_TIMEOUT_US */

struct ls2k0300_spiio_priv_s
{
  struct spi_dev_s dev;
  uintptr_t base;
  uint32_t clock_freq;
  uint32_t frequency;
  enum spi_mode_e mode;
  int nbits;
  bool initialized;
};

static uint32_t spiio_read_reg(FAR struct ls2k0300_spiio_priv_s *priv,
                               uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void spiio_write_reg(FAR struct ls2k0300_spiio_priv_s *priv,
                            uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint8_t spiio_read_reg_byte(FAR struct ls2k0300_spiio_priv_s *priv,
                                   uint32_t offset)
{
  return getreg8(priv->base + offset);
}

static void spiio_write_reg_byte(FAR struct ls2k0300_spiio_priv_s *priv,
                                 uint32_t offset, uint8_t value)
{
  putreg8(value, priv->base + offset);
}

/* ===== SPI bus abort (de-init) — matches spi2_abort() ===== */

static void spiio_abort(FAR struct ls2k0300_spiio_priv_s *priv)
{
  uint32_t val;
  int i;

  val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
  val &= ~(CR1_CSTART | CR1_AUTOSUS);
  spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

  spiio_write_reg(priv, LS2K0300_SPIIO_CR4, 0);
  spiio_write_reg(priv, LS2K0300_SPIIO_CR3, 0);
  up_udelay(10);

  spiio_write_reg(priv, LS2K0300_SPIIO_SR1, SR1_W1C_FLAGS);

  for (i = 0; i < 4; i++)
    {
      if (spiio_read_reg(priv, LS2K0300_SPIIO_SR1) & 0x10) /* RXE */
        break;

      (void)spiio_read_reg_byte(priv, LS2K0300_SPIIO_DR);
    }
}

/* ===== Lock / Unlock ===== */

static int ls2k0300_spiio_lock(FAR struct spi_dev_s *dev, bool lock)
{
  return OK;
}

/* ===== CS =====
 * GPIO85, active-low.  Pinmux set in hardware_init by blind-write.
 */

static void ls2k0300_spiio_select(FAR struct spi_dev_s *dev,
                                 uint32_t devid, bool selected)
{
  uintptr_t gpio_out = PHYS_TO_UNCACHED(LS2K0300_GPIO_BASE) + 0x900;

  if (selected)
    {
      putreg8(0, gpio_out + 85);
    }
  else
    {
      putreg8(1, gpio_out + 85);
    }
}

/* ===== setfrequency ===== */

static uint32_t ls2k0300_spiio_setfrequency(FAR struct spi_dev_s *dev,
                                           uint32_t frequency)
{
  FAR struct ls2k0300_spiio_priv_s *priv =
      (FAR struct ls2k0300_spiio_priv_s *)dev;
  uint32_t brint;

  if (frequency == 0)
    frequency = priv->clock_freq / 2;

  brint = priv->clock_freq / frequency;
  if (brint < 2)
    brint = 2;
  if (brint > 255)
    brint = 255;

  spiio_write_reg(priv, LS2K0300_SPIIO_CFG2,
                  brint << CFG2_BRINT_SHIFT);

  priv->frequency = frequency;
  return priv->frequency;
}

/* ===== setmode ===== */

static void ls2k0300_spiio_setmode(FAR struct spi_dev_s *dev,
                                  enum spi_mode_e mode)
{
  FAR struct ls2k0300_spiio_priv_s *priv =
      (FAR struct ls2k0300_spiio_priv_s *)dev;
  uint32_t cfg1;

  cfg1 = spiio_read_reg(priv, LS2K0300_SPIIO_CFG1);
  cfg1 &= ~(CFG1_CPOL | CFG1_CPHA);

  switch (mode)
    {
      case SPIDEV_MODE0: break;
      case SPIDEV_MODE1: cfg1 |= CFG1_CPHA; break;
      case SPIDEV_MODE2: cfg1 |= CFG1_CPOL; break;
      case SPIDEV_MODE3: cfg1 |= CFG1_CPOL | CFG1_CPHA; break;
      default: return;
    }

  spiio_write_reg(priv, LS2K0300_SPIIO_CFG1, cfg1);
  priv->mode = mode;
}

/* ===== setbits ===== */

static void ls2k0300_spiio_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct ls2k0300_spiio_priv_s *priv =
      (FAR struct ls2k0300_spiio_priv_s *)dev;
  uint32_t cfg1;

  if (nbits < 4 || nbits > 32)
    return;

  cfg1 = spiio_read_reg(priv, LS2K0300_SPIIO_CFG1);
  cfg1 &= ~CFG1_DSIZE_MASK;
  cfg1 |= ((nbits - 1) << CFG1_DSIZE_SHIFT);
  spiio_write_reg(priv, LS2K0300_SPIIO_CFG1, cfg1);

  priv->nbits = nbits;
}

/* ===== send (single byte) — EXACT MATCH of spi2_xfer_byte() =====
 *
 * CS asserted BEFORE this function by select(true).
 * CS deasserted AFTER by select(false).
 */

static uint32_t ls2k0300_spiio_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct ls2k0300_spiio_priv_s *priv =
      (FAR struct ls2k0300_spiio_priv_s *)dev;
  uint32_t val;
  uint32_t sr1;
  uint32_t i;

  /* 1. Enable SPE */

  val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
  val |= CR1_SPE;
  spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

  /* 2. Clear CSTART/AUTOSUS for clean rising edge */

  val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
  val &= ~(CR1_CSTART | CR1_AUTOSUS);
  spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

  /* 3. Clear W1C status flags */

  spiio_write_reg(priv, LS2K0300_SPIIO_SR1, SR1_W1C_FLAGS);

  /* 4. TSIZE = 0 (single frame) */

  spiio_write_reg(priv, LS2K0300_SPIIO_CR3, 0);

  /* 5. Wait for TX FIFO available */

  for (i = 0; i < SPIIO_TIMEOUT; i++)
    {
      sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
      if (sr1 & SR1_TXA)
        break;

      up_udelay(1);
    }

  if (i >= SPIIO_TIMEOUT)
    {
      spiio_abort(priv);
      return 0;
    }

  /* 6. Write TX byte (8-bit access!) */

  spiio_write_reg_byte(priv, LS2K0300_SPIIO_DR, (uint8_t)(wd & 0xff));

  /* 7. CSTART only, no AUTOSUS */

  val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
  val |= CR1_CSTART;
  spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

  /* 8. Wait EOT */

  for (i = 0; i < SPIIO_TIMEOUT; i++)
    {
      sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
      if (sr1 & SR1_EOT)
        break;

      up_udelay(1);
    }

  if (i >= SPIIO_TIMEOUT)
    {
      spiio_abort(priv);
      return 0;
    }

  /* 9. Clear EOT */

  spiio_write_reg(priv, LS2K0300_SPIIO_SR1, SR1_EOT);

  /* 10. Wait RX FIFO */

  for (i = 0; i < SPIIO_TIMEOUT; i++)
    {
      sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
      if (sr1 & SR1_RXA)
        break;

      up_udelay(1);
    }

  if (i >= SPIIO_TIMEOUT)
    {
      spiio_abort(priv);
      return 0;
    }

  /* 11. Read RX byte (8-bit access!) */

  return (uint32_t)spiio_read_reg_byte(priv, LS2K0300_SPIIO_DR);
}

/* ===== exchange (multi-byte) ===== */

#ifdef CONFIG_SPI_EXCHANGE
static void ls2k0300_spiio_exchange(FAR struct spi_dev_s *dev,
                                   FAR const void *txbuffer,
                                   FAR void *rxbuffer,
                                   size_t nwords)
{
  FAR struct ls2k0300_spiio_priv_s *priv =
      (FAR struct ls2k0300_spiio_priv_s *)dev;
  FAR const uint8_t *txptr = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *rxptr = (FAR uint8_t *)rxbuffer;
  uint32_t val;
  uint32_t sr1;
  uint32_t j;
  size_t i;

  for (i = 0; i < nwords; i++)
    {
      /* 1. Enable SPE */

      val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
      val |= CR1_SPE;
      spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

      /* 2. Clear CSTART/AUTOSUS */

      val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
      val &= ~(CR1_CSTART | CR1_AUTOSUS);
      spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

      /* 3. Clear W1C flags */

      spiio_write_reg(priv, LS2K0300_SPIIO_SR1, SR1_W1C_FLAGS);

      /* 4. TSIZE=0 */

      spiio_write_reg(priv, LS2K0300_SPIIO_CR3, 0);

      /* 5. Wait TXA */

      for (j = 0; j < SPIIO_TIMEOUT; j++)
        {
          sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
          if (sr1 & SR1_TXA)
            break;

          up_udelay(1);
        }

      if (j >= SPIIO_TIMEOUT) break;

      /* 6. Write byte */

      spiio_write_reg_byte(priv, LS2K0300_SPIIO_DR,
                           txptr ? *txptr++ : 0x00);

      /* 7. CSTART */

      val = spiio_read_reg(priv, LS2K0300_SPIIO_CR1);
      val |= CR1_CSTART;
      spiio_write_reg(priv, LS2K0300_SPIIO_CR1, val);

      /* 8. Wait EOT */

      for (j = 0; j < SPIIO_TIMEOUT; j++)
        {
          sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
          if (sr1 & SR1_EOT)
            break;

          up_udelay(1);
        }

      if (j >= SPIIO_TIMEOUT) break;

      /* 9. Clear EOT */

      spiio_write_reg(priv, LS2K0300_SPIIO_SR1, SR1_EOT);

      /* 10. Wait RXA */

      for (j = 0; j < SPIIO_TIMEOUT; j++)
        {
          sr1 = spiio_read_reg(priv, LS2K0300_SPIIO_SR1);
          if (sr1 & SR1_RXA)
            break;

          up_udelay(1);
        }

      if (j >= SPIIO_TIMEOUT) break;

      /* 11. Read */

      if (rxptr)
        *rxptr++ = spiio_read_reg_byte(priv, LS2K0300_SPIIO_DR);
      else
        (void)spiio_read_reg_byte(priv, LS2K0300_SPIIO_DR);
    }
}
#endif

/* ===== status ===== */

static uint8_t ls2k0300_spiio_status(FAR struct spi_dev_s *dev,
                                    uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

/* ===== ops table ===== */

static const struct spi_ops_s g_spiio_ops =
{
  ls2k0300_spiio_lock,
  ls2k0300_spiio_select,
  ls2k0300_spiio_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  NULL,
#endif
  ls2k0300_spiio_setmode,
  ls2k0300_spiio_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  NULL,
#endif
  ls2k0300_spiio_status,
#ifdef CONFIG_SPI_CMDDATA
  NULL,
#endif
  ls2k0300_spiio_send,
#ifdef CONFIG_SPI_EXCHANGE
  ls2k0300_spiio_exchange,
#else
  NULL,
  NULL,
#endif
#ifdef CONFIG_SPI_TRIGGER
  NULL,
#endif
  NULL
};

/* ===== pre-allocated instances ===== */

static struct ls2k0300_spiio_priv_s g_spiio_priv[2] =
{
  {
    .dev = { .ops = &g_spiio_ops },
    .base = PHYS_TO_UNCACHED(LS2K0300_SPI2_BASE),
    .clock_freq = SPIIO_CLOCK_FREQ,
    .nbits = 8,
  },
  {
    .dev = { .ops = &g_spiio_ops },
    .base = PHYS_TO_UNCACHED(LS2K0300_SPI3_BASE),
    .clock_freq = SPIIO_CLOCK_FREQ,
    .nbits = 8,
  },
};

/* ===== initialize — matches spi2_init() ===== */

FAR struct spi_dev_s *ls2k0300_spiio_initialize(int port)
{
  FAR struct ls2k0300_spiio_priv_s *priv;
  uint32_t brint;

  if (port < 0 || port > 1)
    return NULL;

  priv = &g_spiio_priv[port];

  if (!priv->initialized)
    {
      /* 1. Enable clock gate (CFG5 bit 17/18, handled by bringup) */

      /* 2. SSMODE_SW, DIE, DOE, MSTR */

      spiio_write_reg(priv, LS2K0300_SPIIO_CFG3,
                      CFG3_MSTR | CFG3_DIE | CFG3_DOE | (1U << 8));

      /* 3. 8-bit frame */

      spiio_write_reg(priv, LS2K0300_SPIIO_CFG1,
                      (7U << CFG1_DSIZE_SHIFT));

      /* 4. Baud rate */

      brint = priv->clock_freq / (CONFIG_GD25_SPIFREQUENCY ?
                  CONFIG_GD25_SPIFREQUENCY : 10000000);
      if (brint < 2)
        brint = 2;
      if (brint > 255)
        brint = 255;

      spiio_write_reg(priv, LS2K0300_SPIIO_CFG2,
                      brint << CFG2_BRINT_SHIFT);

      /* 5. Enable SPI */

      spiio_write_reg(priv, LS2K0300_SPIIO_CR1, CR1_SPE);

      up_udelay(10);

      priv->initialized = true;
    }

  return &priv->dev;
}

#endif