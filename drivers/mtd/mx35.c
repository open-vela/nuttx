/****************************************************************************
 * drivers/mtd/mx35.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_MX35_QSPI
#  include <nuttx/spi/qspi.h>
#else
#  include <nuttx/spi/spi.h>
#endif
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Per the data sheet, MX35 parts can be driven with either SPI mode 0
 * (CPOL=0 and CPHA=0) or mode 3 (CPOL=1 and CPHA=1). If CONFIG_MX35_SPIMODE
 * is not defined, mode 0 will be used.
 */

#ifndef CONFIG_MX35_SPIMODE
#  define CONFIG_MX35_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_MX35_SPIFREQUENCY
#  define CONFIG_MX35_SPIFREQUENCY 104000000
#endif

#ifndef CONFIG_MX35_MANUFACTURER
#  define CONFIG_MX35_MANUFACTURER 0xC2
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_MX35_DEBUG
#  define mx35err(format, ...)    _err(format, ##__VA_ARGS__)
#  define mx35info(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define mx35err(x...)
#  define mx35info(x...)
#endif

/* Identification register values *******************************************/

#define MX35_MANUFACTURER              CONFIG_MX35_MANUFACTURER
#define MX35_MX35LF1GE4AB_CAPACITY     0x12  /* 1 Gb */
#define MX35_MX35LF2GE4AB_CAPACITY     0x22  /* 2 Gb */

/* Chip Geometries **********************************************************/

/* MX35LF1GE4AB capacity is 1 G-bit */

#define MX35_MX35LF1GE4AB_SECTOR_SHIFT  17   /* Sector size 1 << 17 = 128 Kb */
#define MX35_MX35LF1GE4AB_NSECTORS      1024
#define MX35_MX35LF1GE4AB_PAGE_SHIFT    11   /* Page size 1 << 11 = 2 Kb */

/* MX35LF2GE4AB capacity is 2 G-bit */

#define MX35_MX35LF2GE4AB_SECTOR_SHIFT  17   /* Sector size 1 << 17 = 128 Kb */
#define MX35_MX35LF2GE4AB_NSECTORS      2048
#define MX35_MX35LF2GE4AB_PAGE_SHIFT    11   /* Page size 1 << 11 = 2 Kb */

/* MX35 Instructions ********************************************************/

/* Command                    Value     Description             Addr   Data */

/*                                                                 Dummy    */
#define MX35_GET_FEATURE            0x0F   /* Get features         1  0  1      */
#define MX35_SET_FEATURE            0x1F   /* Set features         1  0  1      */
#define MX35_PAGE_READ              0x13   /* Array read           3  0  0      */
#define MX35_READ_FROM_CACHE        0x03   /* Output cache data
                                            * on SO                2  1  1-2112 */
#define MX35_READ_FROM_CACHE_X1     0x0B   /* Output cache data
                                            * on SO                2  1  1-2112 */
#define MX35_READ_FROM_CACHE_X2     0x3B   /* Output cache data
                                            * on SI and SO         2  1  1-2112 */
#define MX35_READ_FROM_CACHE_X4     0x6B   /* Output cache data
                                            * on SI, SO, WP, HOLD  2  1  1-2112 */
#define MX35_READ_ID                0x9F   /* Read device ID       0  1  2      */
#define MX35_ECC_STATUS_READ        0x7C   /* Internal ECC status
                                            * output               0  1  1      */
#define MX35_BLOCK_ERASE            0xD8   /* Block erase          3  0  0      */
#define MX35_PROGRAM_EXECUTE        0x10   /* Enter block/page
                                            * address, execute     3  0  0      */
#define MX35_PROGRAM_LOAD           0x02   /* Load program data with
                                            * cache reset first    2  0  1-2112 */
#define MX35_PROGRAM_LOAD_RANDOM    0x84   /* Load program data
                                            * without cache reset  2  0  1-2112 */
#define MX35_PROGRAM_LOAD_X4        0x32   /* Program load operation
                                            * with x4 data input   2  0  1-2112 */
#define MX35_PROGRAM_LOAD_RANDOM_X4 0x34   /* Load random operation
                                            * with x4 data input   2  0  1-2112 */
#define MX35_WRITE_ENABLE           0x06   /*                      0  0  0      */
#define MX35_WRITE_DISABLE          0x04   /*                      0  0  0      */
#define MX35_RESET                  0xFF   /* Reset the device     0  0  0      */
#define MX35_DUMMY                  0x00   /* No Operation         0  0  0      */

/* Bus abstraction flags -- values match QSPICMD_* so QSPI path
 * can forward directly; SPI path interprets them independently.
 */

#define MX35_CMD_ADDRESS    (1 << 0)
#define MX35_CMD_READDATA   (1 << 1)
#define MX35_CMD_WRITEDATA  (1 << 2)

/* Feature register *********************************************************/

/* Register address */

#define MX35_SECURE_OTP            0xB0
#define MX35_STATUS                0xC0
#define MX35_BLOCK_PROTECTION      0xA0

/* Bit definitions */

/* Secure OTP (On-Time-Programmable) register */

#define MX35_SOTP_QE               (1 << 0)  /* Bit 0: Quad Enable */
#define MX35_SOTP_ECC              (1 << 4)  /* Bit 4: ECC enabled */
#define MX35_SOTP_SOTP_EN          (1 << 6)  /* Bit 6: Secure OTP Enable */
#define MX35_SOTP_SOTP_PROT        (1 << 7)  /* Bit 7: Secure OTP Protect */

/* Status register */

#define MX35_SR_OIP                (1 << 0)  /* Bit 0: Operation in progress */
#define MX35_SR_WEL                (1 << 1)  /* Bit 1: Write enable latch */
#define MX35_SR_E_FAIL             (1 << 2)  /* Bit 2: Erase fail */
#define MX35_SR_P_FAIL             (1 << 3)  /* Bit 3: Program Fail */
#define MX35_SR_ECC_S0             (1 << 4)  /* Bit 4-5: ECC Status  */
#define MX35_SR_ECC_S1             (1 << 5)

/* Block Protection register */

#define MX35_BP_SP                 (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define MX35_BP_COMPL              (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define MX35_BP_INV                (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define MX35_BP_BP0                (1 << 3)  /* Bit 3: Block Protection 0 */
#define MX35_BP_BP1                (1 << 4)  /* Bit 4: Block Protection 1 */
#define MX35_BP_BP2                (1 << 5)  /* Bit 5: Block Protection 2 */
#define MX35_BP_BPRWD              (1 << 7)  /* Bit 7: Block Protection Register
                                              *        Write Disable */

/* ECC Status register */

#define MX35_FEATURE_ECC_MASK          (0x03 << 4)
#define MX35_FEATURE_ECC_INCORRECTABLE (0x02 << 4)
#define MX35_FEATURE_ECC_OFFSET        4
#define MX35_ECC_STATUS_MASK           0x0F
#define MX35_ECC_INCORRECTABLE         0x0F

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct m25p_dev_s.
 */

struct mx35_dev_s
{
  struct mtd_dev_s mtd;
#ifdef CONFIG_MX35_QSPI
  FAR struct qspi_dev_s *dev;
#else
  FAR struct spi_dev_s  *dev;
  uint32_t              spi_devid;
#endif
  uint8_t highcapacity;
  uint8_t  sectorshift;
  uint16_t nsectors;
  uint8_t  pageshift;
  uint8_t eccstatus;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_MX35_QSPI
static inline void mx35_lock(FAR struct qspi_dev_s *dev);
static inline void mx35_unlock(FAR struct qspi_dev_s *dev);
#else
static inline void mx35_lock(FAR struct spi_dev_s *dev);
static inline void mx35_unlock(FAR struct spi_dev_s *dev);
#endif

static int mx35_readid(FAR struct mx35_dev_s *priv);
static bool mx35_waitstatus(FAR struct mx35_dev_s *priv, uint8_t mask,
                            bool successif);
static inline void mx35_writeenable(struct mx35_dev_s *priv);
static inline void mx35_writedisable(struct mx35_dev_s *priv);
static inline uint32_t mx35_addresstorow(FAR struct mx35_dev_s *priv,
                                         uint32_t address);
static inline uint32_t mx35_addresstocolumn(FAR struct mx35_dev_s *priv,
                                            uint32_t address);

static bool mx35_sectorerase(FAR struct mx35_dev_s *priv,
                             off_t startsector);
static int mx35_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks);

static void mx35_readbuffer(FAR struct mx35_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer, size_t length);
static void mx35_issue_page_read(FAR struct mx35_dev_s *priv,
                                 uint32_t pageaddress);
static bool mx35_wait_page_ready(FAR struct mx35_dev_s *priv);
static ssize_t mx35_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer);
static ssize_t mx35_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer);
static ssize_t mx35_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buffer);

static void mx35_write_to_cache(FAR struct mx35_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length);
static void mx35_issue_execute_write(FAR struct mx35_dev_s *priv,
                                     uint32_t pageaddress);
static bool mx35_wait_write_complete(FAR struct mx35_dev_s *priv);
static ssize_t mx35_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer);

static int mx35_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd,
                      unsigned long arg);
static int mx35_isbad(FAR struct mtd_dev_s *dev, off_t block);
static int mx35_markbad(FAR struct mtd_dev_s *dev, off_t block);
static inline void mx35_eccstatusread(struct mx35_dev_s *priv);
static inline void mx35_enableecc(struct mx35_dev_s *priv);
static inline void mx35_unlockblocks(struct mx35_dev_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Bus Abstraction Layer
 ****************************************************************************/

#ifdef CONFIG_MX35_QSPI

static inline void mx35_lock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, true);
  QSPI_SETFREQUENCY(dev, CONFIG_MX35_SPIFREQUENCY);
  QSPI_SETMODE(dev, CONFIG_MX35_SPIMODE);
  QSPI_SETBITS(dev, 8);
}

static inline void mx35_unlock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, false);
}

static inline void mx35_cmd(FAR struct mx35_dev_s *priv,
                            uint8_t cmd, uint32_t addr,
                            uint8_t addrlen, FAR void *buf,
                            size_t buflen, uint32_t flags)
{
  struct qspi_cmdinfo_s cmdinfo;

  cmdinfo.flags   = flags;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.addr    = addr;
  cmdinfo.buflen  = buflen;
  cmdinfo.buffer  = buf;

  QSPI_COMMAND(priv->dev, &cmdinfo);
}

static inline void mx35_memread(FAR struct mx35_dev_s *priv,
                                uint8_t cmd, uint16_t addr,
                                uint8_t dummies, bool quadio,
                                FAR void *buf, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  meminfo.flags   = QSPIMEM_READ |
                    (quadio ? QSPIMEM_QUADIO : 0);
  meminfo.addrlen = 2;
  meminfo.dummies = dummies;
  meminfo.cmd     = cmd;
  meminfo.addr    = addr;
  meminfo.buflen  = buflen;
  meminfo.buffer  = buf;
  meminfo.key     = 0;

  QSPI_MEMORY(priv->dev, &meminfo);
}

static inline void mx35_memwrite(FAR struct mx35_dev_s *priv,
                                 uint8_t cmd, uint16_t addr,
                                 bool quadio,
                                 FAR const void *buf,
                                 size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  meminfo.flags   = QSPIMEM_WRITE |
                    (quadio ? QSPIMEM_QUADIO : 0);
  meminfo.addrlen = 2;
  meminfo.dummies = 0;
  meminfo.cmd     = cmd;
  meminfo.addr    = addr;
  meminfo.buflen  = buflen;
  meminfo.buffer  = (FAR void *)buf;
  meminfo.key     = 0;

  QSPI_MEMORY(priv->dev, &meminfo);
}

#else /* SPI mode */

static inline void mx35_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);
  SPI_SETMODE(dev, CONFIG_MX35_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_MX35_SPIFREQUENCY);
}

static inline void mx35_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

static inline void mx35_cmd(FAR struct mx35_dev_s *priv,
                            uint8_t cmd, uint32_t addr,
                            uint8_t addrlen, FAR void *buf,
                            size_t buflen, uint32_t flags)
{
  int i;

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), true);

  SPI_SEND(priv->dev, cmd);

  if ((flags & MX35_CMD_ADDRESS) && addrlen > 0)
    {
      for (i = addrlen - 1; i >= 0; i--)
        {
          SPI_SEND(priv->dev, (addr >> (i * 8)) & 0xff);
        }
    }

  if ((flags & MX35_CMD_READDATA) && buflen > 0)
    {
      SPI_RECVBLOCK(priv->dev, buf, buflen);
    }
  else if ((flags & MX35_CMD_WRITEDATA) && buflen > 0)
    {
      SPI_SNDBLOCK(priv->dev, buf, buflen);
    }

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), false);
}

static inline void mx35_memread(FAR struct mx35_dev_s *priv,
                                uint8_t cmd, uint16_t addr,
                                uint8_t dummies, bool quadio,
                                FAR void *buf, size_t buflen)
{
  int i;

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, cmd);
  SPI_SEND(priv->dev, (addr >> 8) & 0xff);
  SPI_SEND(priv->dev, addr & 0xff);

  for (i = 0; i < dummies; i++)
    {
      SPI_SEND(priv->dev, 0x00);
    }

  SPI_RECVBLOCK(priv->dev, buf, buflen);
  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), false);
}

static inline void mx35_memwrite(FAR struct mx35_dev_s *priv,
                                 uint8_t cmd, uint16_t addr,
                                 bool quadio,
                                 FAR const void *buf,
                                 size_t buflen)
{
  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, cmd);
  SPI_SEND(priv->dev, (addr >> 8) & 0xff);
  SPI_SEND(priv->dev, addr & 0xff);
  SPI_SNDBLOCK(priv->dev, buf, buflen);
  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), false);
}

#endif /* CONFIG_MX35_QSPI */

/****************************************************************************
 * Name: m25p_readid
 ****************************************************************************/

static int mx35_readid(struct mx35_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t capacity;
  uint8_t idbuf[2];

  mx35info("priv: %p\n", priv);

  mx35_lock(priv->dev);

  mx35_cmd(priv, MX35_READ_ID, 0x00, 1,
           idbuf, 2, MX35_CMD_ADDRESS | MX35_CMD_READDATA);

  manufacturer = idbuf[0];
  capacity     = idbuf[1];

  mx35_unlock(priv->dev);

  mx35info("manufacturer: %02x capacity: %02x\n",
           manufacturer, capacity);

  /* Check for a valid manufacturer */

  if (manufacturer == MX35_MANUFACTURER)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      if (capacity == MX35_MX35LF1GE4AB_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->highcapacity = 0;
          priv->sectorshift = MX35_MX35LF1GE4AB_SECTOR_SHIFT;
          priv->nsectors    = MX35_MX35LF1GE4AB_NSECTORS;
          priv->pageshift   = MX35_MX35LF1GE4AB_PAGE_SHIFT;
          return OK;
        }
      else if (capacity == MX35_MX35LF2GE4AB_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->highcapacity = 1;
          priv->sectorshift = MX35_MX35LF2GE4AB_SECTOR_SHIFT;
          priv->nsectors    = MX35_MX35LF2GE4AB_NSECTORS;
          priv->pageshift   = MX35_MX35LF2GE4AB_PAGE_SHIFT;
          return OK;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: mx35_waitstatus
 ****************************************************************************/

static bool mx35_waitstatus(FAR struct mx35_dev_s *priv,
                            uint8_t mask,
                            bool successif)
{
  uint8_t status;
  int polls = 0;

  do
    {
      mx35_cmd(priv, MX35_GET_FEATURE, MX35_STATUS, 1,
               &status, 1, MX35_CMD_ADDRESS | MX35_CMD_READDATA);

      if ((status & MX35_SR_OIP) == 0)
        {
          break;
        }

      if (++polls > 500)
        {
          nxsig_usleep(1000);
        }
    }
  while (polls < 10000);

  if (polls >= 10000)
    {
      mx35err("waitstatus timeout mask=%02x\n", mask);
      return false;
    }

  mx35info("Complete\n");
  return successif ? ((status & mask) != 0) : ((status & mask) == 0);
}

/****************************************************************************
 * Name:  mx35_writeenable
 ****************************************************************************/

static inline void mx35_writeenable(struct mx35_dev_s *priv)
{
  mx35_cmd(priv, MX35_WRITE_ENABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  mx35_writedisable
 ****************************************************************************/

static inline void mx35_writedisable(struct mx35_dev_s *priv)
{
  mx35_cmd(priv, MX35_WRITE_DISABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  mx35_addresstorow
 ****************************************************************************/

static inline uint32_t mx35_addresstorow(FAR struct mx35_dev_s *priv,
                        uint32_t address)
{
  /* Convert to page */

  uint32_t row = address >> priv->pageshift;

  if (priv->highcapacity)
    {
      const uint32_t plane = (row >> (16 - 6)) & 0x40;

      /* Shift block address */

      row = ((row & ~0x3f) << 1) | (row & 0x3f);

      /* Insert plane select bit */

      row = row | plane;
    }

  return row;
}

/****************************************************************************
 * Name:  mx35_addresstocolumn
 ****************************************************************************/

static inline uint32_t mx35_addresstocolumn(FAR struct mx35_dev_s *priv,
                                            uint32_t address)
{
  uint32_t column = address % (1 << priv->pageshift);

  if (priv->highcapacity)
    {
      /* Convert to page */

      const uint32_t row = address >> priv->pageshift;
      const uint32_t plane = (row >> (16 - 12)) & 0x1000;

      /* Insert plane select bit */

      column = column | plane;
    }
  else
    {
      uint16_t wraplength = 0x00;
      column |= (wraplength & 0xc000);
    }

  return column;
}

/****************************************************************************
 * Name:  mx35_sectorerase (128K)
 ****************************************************************************/

static bool mx35_sectorerase(FAR struct mx35_dev_s *priv, off_t startsector)
{
  off_t address = (off_t)startsector << priv->sectorshift;
  const uint32_t block = mx35_addresstorow(priv, address);

  mx35info("sector: %08lx\n", (long)startsector);

  mx35_writeenable(priv);

  mx35_cmd(priv, MX35_BLOCK_ERASE, block, 3,
           NULL, 0, MX35_CMD_ADDRESS);

  mx35info("Erased\n");
  return mx35_waitstatus(priv, MX35_SR_E_FAIL, false);
}

/****************************************************************************
 * Name: mx35_erase
 ****************************************************************************/

static int mx35_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t blocksleft = nblocks;

  mx35info("startblock: %08lx nblocks: %d\n",
           (long)startblock,
           (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while (blocksleft-- > 0)
    {
      mx35_sectorerase(priv, startblock);
      startblock++;
    }

  mx35_unlock(priv->dev);
  return (int)nblocks;
}

/****************************************************************************
 * Name: mx35_readbuffer
 ****************************************************************************/

static void mx35_readbuffer(FAR struct mx35_dev_s *priv, uint32_t address,
                            uint8_t *buffer, size_t length)
{
  const uint16_t offset = mx35_addresstocolumn(priv, address);

#ifdef CONFIG_MX35_QSPI
  mx35_memread(priv, MX35_READ_FROM_CACHE_X4, offset, 1, true,
               buffer, length);
#else
  mx35_memread(priv, MX35_READ_FROM_CACHE, offset, 1, false,
               buffer, length);
#endif
}

/****************************************************************************
 * Name: mx35_read_page
 ****************************************************************************/

static void mx35_issue_page_read(FAR struct mx35_dev_s *priv,
                                 uint32_t pageaddress)
{
  const uint32_t row = mx35_addresstorow(priv, pageaddress);

  mx35_cmd(priv, MX35_PAGE_READ, row, 3,
           NULL, 0, MX35_CMD_ADDRESS);
}

static bool mx35_wait_page_ready(FAR struct mx35_dev_s *priv)
{
  uint8_t status;
  int polls = 0;

  /* Poll OIP bit and capture status in one pass — avoids the extra
   * SPI transaction that mx35_eccstatusread() would add.
   */

  do
    {
      mx35_cmd(priv, MX35_GET_FEATURE, MX35_STATUS, 1,
               &status, 1, MX35_CMD_ADDRESS | MX35_CMD_READDATA);

      if ((status & MX35_SR_OIP) == 0)
        {
          break;
        }

      if (++polls > 500)
        {
          nxsig_usleep(1000);
        }
    }
  while (polls < 10000);

  if (polls >= 10000)
    {
      mx35err("wait_page_ready timeout\n");
      return false;
    }

  /* Reuse the final status for ECC check */

  priv->eccstatus = status;

  if ((status & MX35_FEATURE_ECC_MASK) == MX35_FEATURE_ECC_INCORRECTABLE)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: mx35_read
 ****************************************************************************/

static ssize_t mx35_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;
  bool page_issued = false;

  mx35info("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) <<
                                    priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) -
                                 position;
      const size_t chunklength = bytesleft < spaceleft ?
                                 bytesleft : spaceleft;

      if (!page_issued)
        {
          mx35_issue_page_read(priv, pageaddress);
          page_issued = true;
        }

      if (!mx35_wait_page_ready(priv))
        {
          break;
        }

      mx35_readbuffer(priv, position, buffer, chunklength);

      /* Pipeline: issue next PAGE_READ after reading current cache */

      if (bytesleft > chunklength)
        {
          uint32_t nextpos = position + chunklength;
          uint32_t nextpage = (nextpos >> priv->pageshift) <<
                               priv->pageshift;
          mx35_issue_page_read(priv, nextpage);
          page_issued = true;
        }
      else
        {
          page_issued = false;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  mx35_unlock(priv->dev);

  mx35info("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: mx35_write_to_cache
 ****************************************************************************/

static void mx35_write_to_cache(FAR struct mx35_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length)
{
  const uint16_t offset = mx35_addresstocolumn(priv, address);

#ifdef CONFIG_MX35_QSPI
  mx35_memwrite(priv, MX35_PROGRAM_LOAD_X4, offset, true,
                buffer, length);
#else
  mx35_memwrite(priv, MX35_PROGRAM_LOAD, offset, false,
                buffer, length);
#endif
}

/****************************************************************************
 * Name: mx35_issue_execute_write
 ****************************************************************************/

static void mx35_issue_execute_write(FAR struct mx35_dev_s *priv,
                                     uint32_t pageaddress)
{
  const uint32_t row = mx35_addresstorow(priv, pageaddress);

  mx35_cmd(priv, MX35_PROGRAM_EXECUTE, row, 3,
           NULL, 0, MX35_CMD_ADDRESS);
}

static bool mx35_wait_write_complete(FAR struct mx35_dev_s *priv)
{
  return mx35_waitstatus(priv, MX35_SR_P_FAIL, false);
}

/****************************************************************************
 * Name: mx35_write
 ****************************************************************************/

static ssize_t mx35_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) <<
                                    priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) -
                                 position;
      const size_t chunklength = bytesleft < spaceleft ?
                                 bytesleft : spaceleft;

      mx35_writeenable(priv);
      mx35_write_to_cache(priv, position, buffer, chunklength);
      mx35_issue_execute_write(priv, pageaddress);

      if (!mx35_wait_write_complete(priv))
        {
          ferr("P_FAIL at pos=%" PRIu32 " page=%" PRIu32 "\n",
               position, pageaddress);
          break;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  mx35_unlock(priv->dev);

  return nbytes - bytesleft;
}

static ssize_t mx35_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  ssize_t nbytes;

  nbytes = mx35_read(dev,
                     startblock << priv->pageshift,
                     nblocks << priv->pageshift,
                     buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }

  return nbytes;
}

static ssize_t mx35_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  ssize_t nbytes;

  nbytes = mx35_write(dev,
                      startblock << priv->pageshift,
                      nblocks << priv->pageshift,
                      buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: mx35_ioctl
 ****************************************************************************/

static int mx35_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  mx35info("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
                  (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks. That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              mx35info("blocksize: %d erasesize: %d neraseblocks: %d\n",
                       geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->nsectors <<
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = mx35_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result =
              (priv->eccstatus & MX35_FEATURE_ECC_MASK) >>
               MX35_FEATURE_ECC_OFFSET;

          ret = OK;
        }
      break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  mx35info("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: mx35_isbad
 *
 * Description:
 *   Check if a block is bad by reading the bad block marker from the
 *   spare area (column 2048) of the first page in the block.
 *   Returns 0 if good, 1 if bad, or negative errno on error.
 *
 ****************************************************************************/

static int mx35_isbad(FAR struct mtd_dev_s *dev, off_t block)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  uint8_t marker;
  uint32_t pageaddr;

  mx35_lock(priv->dev);

  /* First page of the block */

  pageaddr = block << priv->sectorshift;

  /* Issue PAGE READ to load page into cache */

  mx35_issue_page_read(priv, pageaddr);
  mx35_waitstatus(priv, MX35_SR_OIP, false);

  /* Read spare area byte 0 (column 2048) -- bad block marker */

  mx35_memread(priv, MX35_READ_FROM_CACHE_X1,
               (1 << priv->pageshift), 1, false, &marker, 1);

  mx35_unlock(priv->dev);

  /* 0xFF = good block, anything else = bad */

  return marker != 0xff ? 1 : 0;
}

/****************************************************************************
 * Name: mx35_markbad
 *
 * Description:
 *   Mark a block as bad by writing 0x00 to the spare area (column 2048)
 *   of the first page in the block.
 *
 ****************************************************************************/

static int mx35_markbad(FAR struct mtd_dev_s *dev, off_t block)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  uint8_t marker = 0x00;
  uint32_t pageaddr;
  uint32_t row;

  mx35_lock(priv->dev);

  pageaddr = block << priv->sectorshift;
  row = mx35_addresstorow(priv, pageaddr);

  mx35_writeenable(priv);

  /* Write 0x00 to spare area byte 0 (column 2048) */

  mx35_memwrite(priv, MX35_PROGRAM_LOAD,
                (1 << priv->pageshift), false, &marker, 1);

  /* Execute program */

  mx35_cmd(priv, MX35_PROGRAM_EXECUTE, row, 3,
           NULL, 0, MX35_CMD_ADDRESS);

  if (!mx35_waitstatus(priv, MX35_SR_P_FAIL, false))
    {
      mx35err("markbad program failed block=%ld\n", (long)block);
      mx35_unlock(priv->dev);
      return -EIO;
    }

  mx35_unlock(priv->dev);

  return OK;
}

/****************************************************************************
 * Name:  mx35_eccstatusread
 ****************************************************************************/

static inline void mx35_eccstatusread(struct mx35_dev_s *priv)
{
  mx35_cmd(priv, MX35_GET_FEATURE, MX35_STATUS, 1,
           &priv->eccstatus, 1,
           MX35_CMD_ADDRESS | MX35_CMD_READDATA);
}

/****************************************************************************
 * Name:  mx35_enableecc
 ****************************************************************************/

static inline void mx35_enableecc(struct mx35_dev_s *priv)
{
#ifdef CONFIG_MX35_QSPI
  uint8_t secureotp = MX35_SOTP_ECC | MX35_SOTP_QE;
#else
  uint8_t secureotp = MX35_SOTP_ECC;
#endif

  mx35_lock(priv->dev);
  mx35_writeenable(priv);

  mx35_cmd(priv, MX35_SET_FEATURE, MX35_SECURE_OTP, 1,
           &secureotp, 1,
           MX35_CMD_ADDRESS | MX35_CMD_WRITEDATA);

  mx35_writedisable(priv);
  mx35_unlock(priv->dev);
}

/****************************************************************************
 * Name:  mx35_unlockblocks
 ****************************************************************************/

static inline void mx35_unlockblocks(struct mx35_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  mx35_lock(priv->dev);
  mx35_writeenable(priv);

  mx35_cmd(priv, MX35_SET_FEATURE, MX35_BLOCK_PROTECTION, 1,
           &blockprotection, 1,
           MX35_CMD_ADDRESS | MX35_CMD_WRITEDATA);

  mx35_writedisable(priv);
  mx35_unlock(priv->dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx35_initialize / mx35_qspi_initialize
 *
 * Description:
 *   Create an initialize MTD device instance. MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

#ifndef CONFIG_MX35_QSPI
FAR struct mtd_dev_s *mx35_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
  FAR struct mx35_dev_s *priv;
  int ret;

  mx35info("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct mx35_dev_s));
  if (priv)
    {
      priv->mtd.erase  = mx35_erase;
      priv->mtd.bread  = mx35_bread;
      priv->mtd.bwrite = mx35_bwrite;
      priv->mtd.read   = mx35_read;
      priv->mtd.write  = mx35_write;
      priv->mtd.ioctl  = mx35_ioctl;
      priv->mtd.isbad  = mx35_isbad;
      priv->mtd.markbad = mx35_markbad;
      priv->mtd.name   = "mx35";
      priv->dev        = dev;
      priv->spi_devid  = spi_devid;

      /* De-select the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(spi_devid), false);

      /* Reset the flash */

      mx35_cmd(priv, MX35_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      mx35_waitstatus(priv, MX35_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = mx35_readid(priv);
      if (ret != OK)
        {
          mx35err("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      mx35_enableecc(priv);
      mx35_unlockblocks(priv);
    }

  mx35info("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}

#else /* CONFIG_MX35_QSPI */

FAR struct mtd_dev_s *mx35_initialize(FAR struct qspi_dev_s *dev)
{
  FAR struct mx35_dev_s *priv;
  int ret;

  mx35info("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct mx35_dev_s));
  if (priv)
    {
      priv->mtd.erase  = mx35_erase;
      priv->mtd.bread  = mx35_bread;
      priv->mtd.bwrite = mx35_bwrite;
      priv->mtd.read   = mx35_read;
      priv->mtd.write  = mx35_write;
      priv->mtd.ioctl  = mx35_ioctl;
      priv->mtd.isbad  = mx35_isbad;
      priv->mtd.markbad = mx35_markbad;
      priv->mtd.name   = "mx35";
      priv->dev        = dev;

      /* Reset the flash */

      mx35_cmd(priv, MX35_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      mx35_waitstatus(priv, MX35_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = mx35_readid(priv);
      if (ret != OK)
        {
          mx35err("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      mx35_enableecc(priv);
      mx35_unlockblocks(priv);
    }

  mx35info("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
#endif /* CONFIG_MX35_QSPI */
