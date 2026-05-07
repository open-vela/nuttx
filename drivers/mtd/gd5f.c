/****************************************************************************
 * drivers/mtd/gd5f.c
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
#include <inttypes.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_MTD_GD5F_QSPI
#  include <nuttx/spi/qspi.h>
#else
#  include <nuttx/spi/spi.h>
#endif
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_GD5F_SPIMODE
#  define CONFIG_GD5F_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_GD5F_SPIFREQUENCY
#  define CONFIG_GD5F_SPIFREQUENCY  20000000
#endif

/* GD5F Instructions ********************************************************/

/*      Command                  Value     Description       Addr   Data    */

/*                                                                    Dummy */

#define GD5F_GET_FEATURE          0x0f /* Get features        1   0   1     */
#define GD5F_SET_FEATURE          0x1f /* Set features        1   0   1     */
#define GD5F_PAGE_READ            0x13 /* Array read          3   0   0     */
#define GD5F_READ_FROM_CACHE      0x03 /* Output cache data
                                        *  on SO              2   1   1-2112 */
#define GD5F_READ_ID              0x9f /* Read device ID      0   1   2     */
#define GD5F_ECC_STATUS_READ      0x7c /* Internal ECC status
                                        *  output             0   1   1     */
#define GD5F_BLOCK_ERASE          0xd8 /* Block erase         3   0   0     */
#define GD5F_PROGRAM_EXECUTE      0x10 /* Enter block/page
                                        * address, execute    3   0   0     */
#define GD5F_PROGRAM_LOAD         0x02 /* Load program data with
                                        * cache reset first   2   0   1-2112 */
#define GD5F_PROGRAM_LOAD_RANDOM  0x84 /* Load program data
                                        * without cache reset 2   0   1-2112 */
#define GD5F_WRITE_ENABLE         0x06 /*                     0   0   0     */
#define GD5F_WRITE_DISABLE        0x04 /*                     0   0   0     */
#define GD5F_RESET                0xff /* Reset the device    0   0   0     */
#define GD5F_DUMMY                0x00 /* No Operation        0   0   0     */

#define GD5F_READ_FROM_CACHE_X4   0x6b /* Read cache x4       2   1   1-2112 */
#define GD5F_PROGRAM_LOAD_X4      0x32 /* Program load x4 (reserved for
                                        * future quad write)  2   0   1-2112 */

/* Bus abstraction flags -- values match QSPICMD_* so QSPI path
 * can forward directly; SPI path interprets them independently.
 */

#define GD5F_CMD_ADDRESS    (1 << 0)
#define GD5F_CMD_READDATA   (1 << 1)
#define GD5F_CMD_WRITEDATA  (1 << 2)

/* Feature register *********************************************************/

/* JEDEC Read ID register values */

#define GD5F_MANUFACTURER           0xc8
#define GD5F_GD5F_CAPACITY_MASK     0x0f
#define GD5F_CAPACITY_1GBIT         0x01  /* 1 Gb */
#define GD5F_CAPACITY_2GBIT         0x02  /* 2 Gb */
#define GD5F_CAPACITY_4GBIT         0x04  /* 4 Gb */

#define GD5F_NSECTORS_1GBIT         1024  /* 1024x131072 = 1Gbit memory capacity */
#define GD5F_NSECTORS_2GBIT         2048  /* 2048x131072 = 2Gbit memory capacity */
#define GD5F_NSECTORS_4GBIT         4096  /* 4096x131072 = 4Gbit memory capacity */

#define GD5F_SECTOR_SHIFT           17    /* 131072 byte */
#define GD5F_PAGE_SHIFT             11    /* 2048 */

/* Register address */

#define GD5F_SECURE_OTP             0xb0
#define GD5F_STATUS                 0xc0
#define GD5F_BLOCK_PROTECTION       0xa0

/* Bit definitions */

/* Secure OTP (On-Time-Programmable) register */

#define GD5F_SOTP_QE                (1 << 0)  /* Bit 0: Quad Enable */
#define GD5F_SOTP_ECC               (1 << 4)  /* Bit 4: ECC enabled */
#define GD5F_SOTP_SOTP_EN           (1 << 6)  /* Bit 6: Secure OTP Enable */
#define GD5F_SOTP_SOTP_PROT         (1 << 7)  /* Bit 7: Secure OTP Protect */

/* Status register */

#define GD5F_SR_OIP                 (1 << 0)  /* Bit 0: Operation in progress */
#define GD5F_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch */
#define GD5F_SR_E_FAIL              (1 << 2)  /* Bit 2: Erase fail */
#define GD5F_SR_P_FAIL              (1 << 3)  /* Bit 3: Program Fail */
#define GD5F_SR_ECC_S0              (1 << 4)  /* Bit 4-5: ECC Status  */
#define GD5F_SR_ECC_S1              (1 << 5)

/* Block Protection register */

#define GD5F_BP_SP                  (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define GD5F_BP_COMPL               (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define GD5F_BP_INV                 (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define GD5F_BP_BP0                 (1 << 3)  /* Bit 3: Block Protection 0 */
#define GD5F_BP_BP1                 (1 << 4)  /* Bit 4: Block Protection 1 */
#define GD5F_BP_BP2                 (1 << 5)  /* Bit 5: Block Protection 2 */
#define GD5F_BP_BPRWD               (1 << 7)  /* Bit 7: Block Protection Register
                                               *        Write Disable */

/* ECC Status register */

#define GD5F_FEATURE_ECC_MASK       (0x03 << 4)
#define GD5F_FEATURE_ECC_ERROR      (0x02 << 4)
#define GD5F_FEATURE_ECC_OFFSET     4
#define GD5F_ECC_STATUS_MASK        0x0f

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct gd5f_dev_s.
 */

struct gd5f_dev_s
{
  struct mtd_dev_s mtd;
#ifdef CONFIG_MTD_GD5F_QSPI
  FAR struct qspi_dev_s *dev;
#else
  FAR struct spi_dev_s  *dev;
  uint32_t              spi_devid;
#endif
  uint16_t              nsectors;
  uint8_t               sectorshift;
  uint8_t               pageshift;
  uint8_t               eccstatus;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_MTD_GD5F_QSPI
static inline void gd5f_lock(FAR struct qspi_dev_s *dev);
static inline void gd5f_unlock(FAR struct qspi_dev_s *dev);
#else
static inline void gd5f_lock(FAR struct spi_dev_s *dev);
static inline void gd5f_unlock(FAR struct spi_dev_s *dev);
#endif

static int gd5f_readid(FAR struct gd5f_dev_s *priv);
static bool gd5f_waitstatus(FAR struct gd5f_dev_s *priv,
                            uint8_t mask,
                            bool successif);
static inline void gd5f_writeenable(FAR struct gd5f_dev_s *priv);
static inline void gd5f_writedisable(FAR struct gd5f_dev_s *priv);
static bool gd5f_sectorerase(FAR struct gd5f_dev_s *priv,
                             off_t startsector);
static void gd5f_readbuffer(FAR struct gd5f_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length);
static void gd5f_issue_page_read(FAR struct gd5f_dev_s *priv,
                                 uint32_t pageaddress);
static bool gd5f_wait_page_ready(FAR struct gd5f_dev_s *priv);

static void gd5f_write_to_cache(FAR struct gd5f_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length);
static bool gd5f_execute_write(FAR struct gd5f_dev_s *priv,
                               uint32_t position);

static inline void gd5f_set_ecc_unlocked(FAR struct gd5f_dev_s *priv,
                                         bool enable);
static inline void gd5f_enable_ecc(FAR struct gd5f_dev_s *priv);
static inline void gd5f_unlockblocks(FAR struct gd5f_dev_s *priv);

/* MTD driver methods */

static ssize_t gd5f_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks,
                          FAR uint8_t *buffer);
static ssize_t gd5f_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer);
static ssize_t gd5f_bwrite(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR const uint8_t *buffer);
static ssize_t gd5f_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer);
static int gd5f_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd,
                      unsigned long arg);
static int gd5f_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks);
static int gd5f_isbad(FAR struct mtd_dev_s *dev, off_t block);
static int gd5f_markbad(FAR struct mtd_dev_s *dev, off_t block);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Bus Abstraction Layer
 ****************************************************************************/

#ifdef CONFIG_MTD_GD5F_QSPI

static inline void gd5f_lock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, true);
  QSPI_SETFREQUENCY(dev, CONFIG_GD5F_SPIFREQUENCY);
  QSPI_SETMODE(dev, CONFIG_GD5F_SPIMODE);
  QSPI_SETBITS(dev, 8);
}

static inline void gd5f_unlock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, false);
}

static inline void gd5f_cmd(FAR struct gd5f_dev_s *priv,
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

static inline void gd5f_memread(FAR struct gd5f_dev_s *priv,
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

static inline void gd5f_memwrite(FAR struct gd5f_dev_s *priv,
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

static inline void gd5f_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);
  SPI_SETMODE(dev, CONFIG_GD5F_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_GD5F_SPIFREQUENCY);
}

static inline void gd5f_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

static inline void gd5f_cmd(FAR struct gd5f_dev_s *priv,
                            uint8_t cmd, uint32_t addr,
                            uint8_t addrlen, FAR void *buf,
                            size_t buflen, uint32_t flags)
{
  int i;

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), true);

  SPI_SEND(priv->dev, cmd);

  if ((flags & GD5F_CMD_ADDRESS) && addrlen > 0)
    {
      for (i = addrlen - 1; i >= 0; i--)
        {
          SPI_SEND(priv->dev, (addr >> (i * 8)) & 0xff);
        }
    }

  if ((flags & GD5F_CMD_READDATA) && buflen > 0)
    {
      SPI_RECVBLOCK(priv->dev, buf, buflen);
    }
  else if ((flags & GD5F_CMD_WRITEDATA) && buflen > 0)
    {
      SPI_SNDBLOCK(priv->dev, buf, buflen);
    }

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), false);
}

static inline void gd5f_memread(FAR struct gd5f_dev_s *priv,
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

static inline void gd5f_memwrite(FAR struct gd5f_dev_s *priv,
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

#endif /* CONFIG_MTD_GD5F_QSPI */

/****************************************************************************
 * Name: gd5f_readid
 ****************************************************************************/

static int gd5f_readid(FAR struct gd5f_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t deviceid;
  uint16_t capacity;
  uint8_t idbuf[2];

  finfo("priv: %p\n", priv);

  gd5f_lock(priv->dev);

  gd5f_cmd(priv, GD5F_READ_ID, 0x00, 1,
           idbuf, 2, GD5F_CMD_ADDRESS | GD5F_CMD_READDATA);

  manufacturer = idbuf[0];
  deviceid     = idbuf[1];

  gd5f_unlock(priv->dev);

  finfo("SPI NAND ID: manufacturer=%02x device=%02x\n",
        manufacturer, deviceid);

  /* Check for a valid manufacturer */

  if (manufacturer == GD5F_MANUFACTURER)
    {
      capacity = deviceid & GD5F_GD5F_CAPACITY_MASK;

      if (capacity == GD5F_CAPACITY_1GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_1GBIT;
        }
      else if (capacity == GD5F_CAPACITY_2GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_2GBIT;
        }
      else if (capacity == GD5F_CAPACITY_4GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_4GBIT;
        }
      else
        {
          return -ENODEV;
        }

      priv->sectorshift = GD5F_SECTOR_SHIFT;
      priv->pageshift   = GD5F_PAGE_SHIFT;
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: gd5f_waitstatus
 ****************************************************************************/

static bool gd5f_waitstatus(FAR struct gd5f_dev_s *priv,
                            uint8_t mask,
                            bool successif)
{
  uint8_t status;
  int polls = 0;

  do
    {
      gd5f_cmd(priv, GD5F_GET_FEATURE, GD5F_STATUS, 1,
               &status, 1,
               GD5F_CMD_ADDRESS | GD5F_CMD_READDATA);

      if ((status & GD5F_SR_OIP) == 0)
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
      ferr("waitstatus timeout mask=%02x\n", mask);
      return false;
    }

  /* Save status for ECC check -- avoids extra GET_FEATURE call */

  priv->eccstatus = status;

  finfo("Complete %02x\n", status);

  return successif ? ((status & mask) != 0) :
                     ((status & mask) == 0);
}

/****************************************************************************
 * Name:  gd5f_writeenable
 ****************************************************************************/

static inline void gd5f_writeenable(FAR struct gd5f_dev_s *priv)
{
  gd5f_cmd(priv, GD5F_WRITE_ENABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  gd5f_writedisable
 ****************************************************************************/

static inline void gd5f_writedisable(FAR struct gd5f_dev_s *priv)
{
  gd5f_cmd(priv, GD5F_WRITE_DISABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  gd5f_sectorerase (128K)
 ****************************************************************************/

static bool gd5f_sectorerase(FAR struct gd5f_dev_s *priv,
                             off_t startsector)
{
  const uint32_t block = startsector << (priv->sectorshift -
                                         priv->pageshift);

  finfo("block sector: %08lx\n", (long)block);

  gd5f_writeenable(priv);

  gd5f_cmd(priv, GD5F_BLOCK_ERASE, block, 3,
           NULL, 0, GD5F_CMD_ADDRESS);

  finfo("Erased\n");
  return gd5f_waitstatus(priv, GD5F_SR_E_FAIL, false);
}

/****************************************************************************
 * Name: gd5f_erase
 ****************************************************************************/

static int gd5f_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("Erase: startblock: %08lx nblocks: %d\n",
        (long)startblock,
        (int)nblocks);

  gd5f_lock(priv->dev);
  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (blocksleft > 0)
    {
      if (!gd5f_sectorerase(priv, startblock))
        {
          break;
        }

      startblock++;
      blocksleft--;
    }

  gd5f_unlock(priv->dev);
  return nblocks - blocksleft;
}

/****************************************************************************
 * Name: gd5f_readbuffer
 ****************************************************************************/

static void gd5f_readbuffer(FAR struct gd5f_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length)
{
  const uint16_t offset = address &
                          ((1 << priv->pageshift) - 1);

#ifdef CONFIG_MTD_GD5F_QSPI
  gd5f_memread(priv, GD5F_READ_FROM_CACHE_X4, offset,
               1, true, buffer, length);
#else
  gd5f_memread(priv, GD5F_READ_FROM_CACHE, offset,
               1, false, buffer, length);
#endif
}

/****************************************************************************
 * Name: gd5f_issue_page_read / gd5f_wait_page_ready
 ****************************************************************************/

static void gd5f_issue_page_read(FAR struct gd5f_dev_s *priv,
                                 uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  gd5f_cmd(priv, GD5F_PAGE_READ, row, 3,
           NULL, 0, GD5F_CMD_ADDRESS);
}

static bool gd5f_wait_page_ready(FAR struct gd5f_dev_s *priv)
{
  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  if ((priv->eccstatus & GD5F_FEATURE_ECC_MASK) ==
      GD5F_FEATURE_ECC_ERROR)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: gd5f_read
 ****************************************************************************/

static ssize_t gd5f_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;
  bool page_issued = false;

  finfo("Read: offset: %08lx nbytes: %d\n",
        (long)offset, (int)nbytes);

  gd5f_lock(priv->dev);
  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress =
                     (position >> priv->pageshift) <<
                      priv->pageshift;
      const uint32_t spaceleft =
                     pageaddress + (1 << priv->pageshift) -
                     position;
      const size_t chunklength =
                   bytesleft < spaceleft ? bytesleft :
                                           spaceleft;

      if (!page_issued)
        {
          gd5f_issue_page_read(priv, pageaddress);
          page_issued = true;
        }

      if (!gd5f_wait_page_ready(priv))
        {
          break;
        }

      gd5f_readbuffer(priv, position, buffer, chunklength);

      /* Pipeline: issue next PAGE_READ while we process current
       * data.  The NAND array-to-cache transfer (~25us) overlaps
       * with the caller consuming the buffer.
       */

      if (bytesleft > chunklength)
        {
          uint32_t nextpos = position + chunklength;
          uint32_t nextpage = (nextpos >> priv->pageshift) <<
                               priv->pageshift;
          gd5f_issue_page_read(priv, nextpage);
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

  gd5f_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: gd5f_bread
 ****************************************************************************/

static ssize_t gd5f_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks,
                          FAR uint8_t *buffer)
{
  ssize_t nbytes;
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;

  finfo("Bread: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  nbytes = gd5f_read(dev, startblock << priv->pageshift,
                     nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: gd5f_write_to_cache
 ****************************************************************************/

static void gd5f_write_to_cache(FAR struct gd5f_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length)
{
  const uint16_t offset = address &
                          ((1 << priv->pageshift) - 1);

#ifdef CONFIG_MTD_GD5F_QSPI
  gd5f_memwrite(priv, GD5F_PROGRAM_LOAD_X4, offset, true,
                buffer, length);
#else
  gd5f_memwrite(priv, GD5F_PROGRAM_LOAD, offset, false,
                buffer, length);
#endif
}

/****************************************************************************
 * Name: gd5f_execute_write
 ****************************************************************************/

static bool gd5f_execute_write(FAR struct gd5f_dev_s *priv,
                               uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  gd5f_cmd(priv, GD5F_PROGRAM_EXECUTE, row, 3,
           NULL, 0, GD5F_CMD_ADDRESS);

  return gd5f_waitstatus(priv, GD5F_SR_P_FAIL, false);
}

/****************************************************************************
 * Name: gd5f_write
 ****************************************************************************/

static ssize_t gd5f_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Write: offset: %08lx nbytes: %d\n",
        (long)offset, (int)nbytes);

  gd5f_lock(priv->dev);
  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress =
                    (position >> priv->pageshift) <<
                     priv->pageshift;
      const uint32_t spaceleft =
                     pageaddress + (1 << priv->pageshift) -
                     position;
      const size_t chunklength =
                   bytesleft < spaceleft ? bytesleft :
                                           spaceleft;

      gd5f_writeenable(priv);
      gd5f_write_to_cache(priv, position, buffer,
                          chunklength);
      if (!gd5f_execute_write(priv, pageaddress))
        {
          break;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  gd5f_unlock(priv->dev);

  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: gd5f_bwrite
 ****************************************************************************/

static ssize_t gd5f_bwrite(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR const uint8_t *buffer)
{
  ssize_t nbytes;

  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;

  finfo("Bwrite: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  nbytes = gd5f_write(dev, startblock << priv->pageshift,
                nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: mx25l_ioctl
 ****************************************************************************/

static int gd5f_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd, unsigned long arg)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  int ret = -EINVAL;

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
                  (FAR struct mtd_geometry_s *)
                  ((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              finfo("blocksize: %" PRIu32 " erasesize: %" PRIu32
                    " neraseblocks: %" PRIu32 "\n",
                    geo->blocksize, geo->erasesize,
                    geo->neraseblocks);
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

          ret = gd5f_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result =
               (priv->eccstatus & GD5F_FEATURE_ECC_MASK)
                >> GD5F_FEATURE_ECC_OFFSET;

          ret = OK;
        }
      break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: gd5f_isbad
 *
 * Description:
 *   Check if a block is bad by reading the bad block marker from the
 *   spare area (column 2048) of the first page in the block.
 *   Returns 0 if good, 1 if bad, or negative errno on error.
 *
 ****************************************************************************/

static int gd5f_isbad(FAR struct mtd_dev_s *dev, off_t block)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  uint8_t marker = 0xff;
  uint32_t pageaddr;
  int ret = 0;

  /* First page of the block */

  pageaddr = block << priv->sectorshift;

  gd5f_lock(priv->dev);

  /* Disable on-chip ECC before reading the bad-block marker.  With ECC
   * enabled the marker byte (0x00 on a factory-marked bad block) would be
   * treated as parity payload and either flagged as an uncorrectable error
   * or silently "corrected", producing a wrong isbad() result.
   */

  gd5f_set_ecc_unlocked(priv, false);
  if (!gd5f_waitstatus(priv, GD5F_SR_OIP, false))
    {
      ferr("isbad: ECC-off settle timeout block=%ld\n", (long)block);
      ret = -EIO;
      goto out_restore_ecc;
    }

  /* Issue PAGE READ to load page into cache */

  gd5f_issue_page_read(priv, pageaddr);
  if (!gd5f_waitstatus(priv, GD5F_SR_OIP, false))
    {
      ferr("isbad: page-read timeout block=%ld\n", (long)block);
      ret = -EIO;
      goto out_restore_ecc;
    }

  /* Read spare area byte 0 (column 2048) -- bad block marker.  memread
   * returns void; transfer faults are caught by the waitstatus above.
   * marker is pre-initialised to 0xff so a silent read corruption fails
   * "good" rather than fabricating a bad marker from stack garbage.
   */

#ifdef CONFIG_MTD_GD5F_QSPI
  gd5f_memread(priv, GD5F_READ_FROM_CACHE_X4,
               (1 << priv->pageshift), 1, true, &marker, 1);
#else
  gd5f_memread(priv, GD5F_READ_FROM_CACHE,
               (1 << priv->pageshift), 1, false, &marker, 1);
#endif

  /* 0xFF = good block, anything else = bad */

  ret = (marker != 0xff) ? 1 : 0;

out_restore_ecc:

  /* Restore ECC unconditionally so subsequent reads/writes operate
   * normally even if any step above failed.  A restore failure is
   * logged but does not overwrite a good/bad verdict.
   */

  gd5f_set_ecc_unlocked(priv, true);
  if (!gd5f_waitstatus(priv, GD5F_SR_OIP, false))
    {
      ferr("isbad: ECC-restore timeout block=%ld\n", (long)block);
      if (ret >= 0)
        {
          ret = -EIO;
        }
    }

  gd5f_unlock(priv->dev);

  return ret;
}

/****************************************************************************
 * Name: gd5f_markbad
 *
 * Description:
 *   Mark a block as bad by writing 0x00 to the spare area (column 2048)
 *   of the first page in the block.
 *
 ****************************************************************************/

static int gd5f_markbad(FAR struct mtd_dev_s *dev, off_t block)
{
#ifdef CONFIG_MTD_READONLY
  return -EACCES;
#else
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  uint8_t marker = 0x00;
  uint32_t pageaddr;
  uint32_t row;

  pageaddr = block << priv->sectorshift;
  row = pageaddr >> priv->pageshift;

  gd5f_lock(priv->dev);

  gd5f_writeenable(priv);

  /* Write 0x00 to spare area byte 0 (column 2048) */

  gd5f_memwrite(priv, GD5F_PROGRAM_LOAD,
                (1 << priv->pageshift), false, &marker, 1);

  /* Execute program */

  gd5f_cmd(priv, GD5F_PROGRAM_EXECUTE, row, 3,
           NULL, 0, GD5F_CMD_ADDRESS);

  if (!gd5f_waitstatus(priv, GD5F_SR_P_FAIL, false))
    {
      ferr("markbad program failed block=%ld\n", (long)block);
      gd5f_unlock(priv->dev);
      return -EIO;
    }

  gd5f_unlock(priv->dev);

  return OK;
#endif /* CONFIG_MTD_READONLY */
}

/****************************************************************************
 * Name:  gd5f_set_ecc_unlocked
 *
 * Description:
 *   Enable or disable the on-chip ECC engine by writing the Secure OTP
 *   feature register. The caller must already hold the SPI bus lock.
 *
 ****************************************************************************/

static inline void gd5f_set_ecc_unlocked(FAR struct gd5f_dev_s *priv,
                                         bool enable)
{
#ifdef CONFIG_MTD_GD5F_QSPI
  uint8_t secure_otp = GD5F_SOTP_QE;
#else
  uint8_t secure_otp = 0;
#endif

  if (enable)
    {
      secure_otp |= GD5F_SOTP_ECC;
    }

  gd5f_writeenable(priv);

  gd5f_cmd(priv, GD5F_SET_FEATURE, GD5F_SECURE_OTP, 1,
           &secure_otp, 1,
           GD5F_CMD_ADDRESS | GD5F_CMD_WRITEDATA);

  gd5f_writedisable(priv);
}

/****************************************************************************
 * Name:  gd5f_enable_ecc
 ****************************************************************************/

static inline void gd5f_enable_ecc(
                          FAR struct gd5f_dev_s *priv)
{
  gd5f_lock(priv->dev);
  gd5f_set_ecc_unlocked(priv, true);
  gd5f_unlock(priv->dev);
}

/****************************************************************************
 * Name:  gd5f_unlockblocks
 ****************************************************************************/

static inline void gd5f_unlockblocks(
                          FAR struct gd5f_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  gd5f_lock(priv->dev);
  gd5f_writeenable(priv);

  gd5f_cmd(priv, GD5F_SET_FEATURE, GD5F_BLOCK_PROTECTION,
           1, &blockprotection, 1,
           GD5F_CMD_ADDRESS | GD5F_CMD_WRITEDATA);

  gd5f_writedisable(priv);
  gd5f_unlock(priv->dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd5f_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for SPI interface.
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions(such as a block
 *   or character driver front end).
 *
 ****************************************************************************/

#ifndef CONFIG_MTD_GD5F_QSPI
FAR struct mtd_dev_s *gd5f_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
  FAR struct gd5f_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct gd5f_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase   = gd5f_erase;
      priv->mtd.bread   = gd5f_bread;
      priv->mtd.bwrite  = gd5f_bwrite;
      priv->mtd.ioctl   = gd5f_ioctl;
      priv->mtd.isbad   = gd5f_isbad;
      priv->mtd.markbad = gd5f_markbad;
      priv->mtd.name    = "gd5f";
      priv->dev         = dev;
      priv->spi_devid   = spi_devid;

      /* De-select the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(spi_devid), false);

      /* Reset the flash */

      gd5f_cmd(priv, GD5F_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      gd5f_waitstatus(priv, GD5F_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = gd5f_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      gd5f_enable_ecc(priv);
      gd5f_waitstatus(priv, GD5F_SR_OIP, false);
      gd5f_unlockblocks(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}

#else /* CONFIG_MTD_GD5F_QSPI */

/****************************************************************************
 * Name: gd5f_qspi_initialize
 *
 * Description:
 *   Initialize GD5F SPI NAND over QSPI interface with Quad I/O
 *   support.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *gd5f_qspi_initialize(
                                FAR struct qspi_dev_s *dev)
{
  FAR struct gd5f_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct gd5f_dev_s));
  if (priv)
    {
      priv->mtd.erase   = gd5f_erase;
      priv->mtd.bread   = gd5f_bread;
      priv->mtd.bwrite  = gd5f_bwrite;
      priv->mtd.ioctl   = gd5f_ioctl;
      priv->mtd.isbad   = gd5f_isbad;
      priv->mtd.markbad = gd5f_markbad;
      priv->mtd.name    = "gd5f";
      priv->dev         = dev;

      /* Reset the flash */

      gd5f_cmd(priv, GD5F_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      gd5f_waitstatus(priv, GD5F_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = gd5f_readid(priv);
      if (ret != OK)
        {
          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      gd5f_enable_ecc(priv);
      gd5f_waitstatus(priv, GD5F_SR_OIP, false);
      gd5f_unlockblocks(priv);
    }

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
#endif /* CONFIG_MTD_GD5F_QSPI */
