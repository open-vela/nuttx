/****************************************************************************
 * drivers/mtd/f50l.c
 *
 * ESMT F50L/F50D SPI NAND flash driver.  Covers the F50L (3.3V) and
 * F50D (1.8V) families from Elite Semiconductor (ESMT).  These chips
 * share manufacturer ID 0xC8 with GigaDevice but have no QE bit in
 * Configuration Register B0h — quad I/O is enabled via WPE=0 in
 * Protection Register A0h.
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
#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_MTD_F50L_QSPI
#  include <nuttx/spi/qspi.h>
#else
#  include <nuttx/spi/spi.h>
#endif
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_F50L_SPIMODE
#  define CONFIG_F50L_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_F50L_SPIFREQUENCY
#  define CONFIG_F50L_SPIFREQUENCY  20000000
#endif

/* GD5F Instructions ********************************************************/

/*      Command                  Value     Description       Addr   Data    */

/*                                                                    Dummy */

#define F50L_GET_FEATURE          0x0f /* Get features        1   0   1     */
#define F50L_SET_FEATURE          0x1f /* Set features        1   0   1     */
#define F50L_PAGE_READ            0x13 /* Array read          3   0   0     */
#define F50L_READ_FROM_CACHE      0x03 /* Output cache data
                                        *  on SO              2   1   1-2112 */
#define F50L_READ_ID              0x9f /* Read device ID      0   1   2     */
#define F50L_ECC_STATUS_READ      0x7c /* Internal ECC status
                                        *  output             0   1   1     */
#define F50L_BLOCK_ERASE          0xd8 /* Block erase         3   0   0     */
#define F50L_PROGRAM_EXECUTE      0x10 /* Enter block/page
                                        * address, execute    3   0   0     */
#define F50L_PROGRAM_LOAD         0x02 /* Load program data with
                                        * cache reset first   2   0   1-2112 */
#define F50L_PROGRAM_LOAD_RANDOM  0x84 /* Load program data
                                        * without cache reset 2   0   1-2112 */
#define F50L_WRITE_ENABLE         0x06 /*                     0   0   0     */
#define F50L_WRITE_DISABLE        0x04 /*                     0   0   0     */
#define F50L_RESET                0xff /* Reset the device    0   0   0     */
#define F50L_DUMMY                0x00 /* No Operation        0   0   0     */

#define F50L_READ_FROM_CACHE_X4   0x6b /* Read cache x4       2   1   1-2112 */
#define F50L_PROGRAM_LOAD_X4      0x32 /* Program load x4 (reserved for
                                        * future quad write)  2   0   1-2112 */

/* Bus abstraction flags -- values match QSPICMD_* so QSPI path
 * can forward directly; SPI path interprets them independently.
 */

#define F50L_CMD_ADDRESS    (1 << 0)
#define F50L_CMD_READDATA   (1 << 1)
#define F50L_CMD_WRITEDATA  (1 << 2)

/* Feature register *********************************************************/

/* JEDEC Read ID register values */

/* ESMT uses 0xC8 (shared with GigaDevice) on most parts, and 0x8C
 * (ESMT's own JEDEC ID) on newer silicon like F50L1G41LC.
 */

#define F50L_MANUFACTURER_C8        0xc8
#define F50L_MANUFACTURER_8C        0x8c

/* ESMT device IDs — full byte, not masked like GigaDevice */

#define F50L_DEVID_F50L1G41LB       0x01  /* 1Gb 3.3V */
#define F50L_DEVID_F50D1G41LB       0x11  /* 1Gb 1.8V */
#define F50L_DEVID_F50L1G41LC       0x21  /* 1Gb 3.3V variant */
#define F50L_DEVID_F50L2G41KA       0x41  /* 2Gb 3.3V */
#define F50L_DEVID_F50D2G41KA       0x51  /* 2Gb 1.8V */

#define F50L_NSECTORS_1GBIT         1024
#define F50L_NSECTORS_2GBIT         2048

#define F50L_SECTOR_SHIFT           17    /* 131072 byte */
#define F50L_PAGE_SHIFT             11    /* 2048 */

/* Register address */

#define F50L_SECURE_OTP             0xb0
#define F50L_STATUS                 0xc0
#define F50L_BLOCK_PROTECTION       0xa0

/* Bit definitions */

/* Configuration register (B0h) — ESMT has NO QE bit (bit 0 reserved).
 * Quad I/O is enabled via WPE=0 in Protection Register A0h.
 */

#define F50L_CFG_ECC                (1 << 4)  /* Bit 4: ECC enabled */
#define F50L_CFG_OTP_EN            (1 << 6)  /* Bit 6: OTP Enable */
#define F50L_CFG_OTP_PROT          (1 << 7)  /* Bit 7: OTP Protect */

/* Status register */

#define F50L_SR_OIP                 (1 << 0)  /* Bit 0: Operation in progress */
#define F50L_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch */
#define F50L_SR_E_FAIL              (1 << 2)  /* Bit 2: Erase fail */
#define F50L_SR_P_FAIL              (1 << 3)  /* Bit 3: Program Fail */
#define F50L_SR_ECC_S0              (1 << 4)  /* Bit 4-5: ECC Status  */
#define F50L_SR_ECC_S1              (1 << 5)

/* Block Protection register */

#define F50L_BP_SP                  (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define F50L_BP_COMPL               (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define F50L_BP_INV                 (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define F50L_BP_BP0                 (1 << 3)  /* Bit 3: Block Protection 0 */
#define F50L_BP_BP1                 (1 << 4)  /* Bit 4: Block Protection 1 */
#define F50L_BP_BP2                 (1 << 5)  /* Bit 5: Block Protection 2 */
#define F50L_BP_BPRWD               (1 << 7)  /* Bit 7: Block Protection Register
                                               *        Write Disable */

/* ECC Status register */

#define F50L_FEATURE_ECC_MASK       (0x03 << 4)
#define F50L_FEATURE_ECC_ERROR      (0x02 << 4)
#define F50L_FEATURE_ECC_OFFSET     4
#define F50L_ECC_STATUS_MASK        0x0f

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct f50l_dev_s.
 */

struct f50l_dev_s
{
  struct mtd_dev_s mtd;
#ifdef CONFIG_MTD_F50L_QSPI
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

#ifdef CONFIG_MTD_F50L_QSPI
static inline void f50l_lock(FAR struct qspi_dev_s *dev);
static inline void f50l_unlock(FAR struct qspi_dev_s *dev);
#else
static inline void f50l_lock(FAR struct spi_dev_s *dev);
static inline void f50l_unlock(FAR struct spi_dev_s *dev);
#endif

static int f50l_readid(FAR struct f50l_dev_s *priv);
static bool f50l_waitstatus(FAR struct f50l_dev_s *priv,
                            uint8_t mask,
                            bool successif);
static inline void f50l_writeenable(FAR struct f50l_dev_s *priv);
static inline void f50l_writedisable(FAR struct f50l_dev_s *priv);
static bool f50l_sectorerase(FAR struct f50l_dev_s *priv,
                             off_t startsector);
static void f50l_readbuffer(FAR struct f50l_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length);
static void f50l_issue_page_read(FAR struct f50l_dev_s *priv,
                                 uint32_t pageaddress);
static bool f50l_wait_page_ready(FAR struct f50l_dev_s *priv);

static void f50l_write_to_cache(FAR struct f50l_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length);
static bool f50l_execute_write(FAR struct f50l_dev_s *priv,
                               uint32_t position);

static inline void f50l_enable_ecc(FAR struct f50l_dev_s *priv);
static inline void f50l_unlockblocks(FAR struct f50l_dev_s *priv);

/* MTD driver methods */

static ssize_t f50l_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks,
                          FAR uint8_t *buffer);
static ssize_t f50l_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer);
static ssize_t f50l_bwrite(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR const uint8_t *buffer);
static ssize_t f50l_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer);
static int f50l_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd,
                      unsigned long arg);
static int f50l_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks);
static int f50l_isbad(FAR struct mtd_dev_s *dev, off_t block);
static int f50l_markbad(FAR struct mtd_dev_s *dev, off_t block);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Bus Abstraction Layer
 ****************************************************************************/

#ifdef CONFIG_MTD_F50L_QSPI

static inline void f50l_lock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, true);
  QSPI_SETFREQUENCY(dev, CONFIG_F50L_SPIFREQUENCY);
  QSPI_SETMODE(dev, CONFIG_F50L_SPIMODE);
  QSPI_SETBITS(dev, 8);
}

static inline void f50l_unlock(FAR struct qspi_dev_s *dev)
{
  QSPI_LOCK(dev, false);
}

static inline void f50l_cmd(FAR struct f50l_dev_s *priv,
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

static inline void f50l_memread(FAR struct f50l_dev_s *priv,
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

static inline void f50l_memwrite(FAR struct f50l_dev_s *priv,
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

static inline void f50l_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);
  SPI_SETMODE(dev, CONFIG_F50L_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_F50L_SPIFREQUENCY);
}

static inline void f50l_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

static inline void f50l_cmd(FAR struct f50l_dev_s *priv,
                            uint8_t cmd, uint32_t addr,
                            uint8_t addrlen, FAR void *buf,
                            size_t buflen, uint32_t flags)
{
  int i;

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), true);

  SPI_SEND(priv->dev, cmd);

  if ((flags & F50L_CMD_ADDRESS) && addrlen > 0)
    {
      for (i = addrlen - 1; i >= 0; i--)
        {
          SPI_SEND(priv->dev, (addr >> (i * 8)) & 0xff);
        }
    }

  if ((flags & F50L_CMD_READDATA) && buflen > 0)
    {
      SPI_RECVBLOCK(priv->dev, buf, buflen);
    }
  else if ((flags & F50L_CMD_WRITEDATA) && buflen > 0)
    {
      SPI_SNDBLOCK(priv->dev, buf, buflen);
    }

  SPI_SELECT(priv->dev,
             SPIDEV_FLASH(priv->spi_devid), false);
}

static inline void f50l_memread(FAR struct f50l_dev_s *priv,
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

static inline void f50l_memwrite(FAR struct f50l_dev_s *priv,
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

#endif /* CONFIG_MTD_F50L_QSPI */

/****************************************************************************
 * Name: f50l_readid
 ****************************************************************************/

static int f50l_readid(FAR struct f50l_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t deviceid;
  uint8_t idbuf[2];

  finfo("priv: %p\n", priv);

  f50l_lock(priv->dev);

  f50l_cmd(priv, F50L_READ_ID, 0x00, 1,
           idbuf, 2, F50L_CMD_ADDRESS | F50L_CMD_READDATA);

  manufacturer = idbuf[0];
  deviceid     = idbuf[1];

  f50l_unlock(priv->dev);

  finfo("SPI NAND ID: manufacturer=%02x device=%02x\n",
        manufacturer, deviceid);

  /* Accept ESMT manufacturer IDs: 0xC8 (shared with GigaDevice)
   * and 0x8C (ESMT's own JEDEC ID on newer silicon).
   */

  if (manufacturer != F50L_MANUFACTURER_C8 &&
      manufacturer != F50L_MANUFACTURER_8C)
    {
      return -ENODEV;
    }

  /* Match ESMT device IDs */

  switch (deviceid)
    {
      case F50L_DEVID_F50L1G41LB:   /* 1Gb 3.3V */
      case F50L_DEVID_F50D1G41LB:   /* 1Gb 1.8V */
      case F50L_DEVID_F50L1G41LC:   /* 1Gb 3.3V variant */
        priv->nsectors = F50L_NSECTORS_1GBIT;
        break;

      case F50L_DEVID_F50L2G41KA:   /* 2Gb 3.3V */
      case F50L_DEVID_F50D2G41KA:   /* 2Gb 1.8V */
        priv->nsectors = F50L_NSECTORS_2GBIT;
        break;

      default:
        return -ENODEV;
    }

  priv->sectorshift = F50L_SECTOR_SHIFT;
  priv->pageshift   = F50L_PAGE_SHIFT;
  return OK;
}

/****************************************************************************
 * Name: f50l_waitstatus
 ****************************************************************************/

static bool f50l_waitstatus(FAR struct f50l_dev_s *priv,
                            uint8_t mask,
                            bool successif)
{
  uint8_t status;
  int polls = 0;

  do
    {
      f50l_cmd(priv, F50L_GET_FEATURE, F50L_STATUS, 1,
               &status, 1,
               F50L_CMD_ADDRESS | F50L_CMD_READDATA);

      if ((status & F50L_SR_OIP) == 0)
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
 * Name:  f50l_writeenable
 ****************************************************************************/

static inline void f50l_writeenable(FAR struct f50l_dev_s *priv)
{
  f50l_cmd(priv, F50L_WRITE_ENABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  f50l_writedisable
 ****************************************************************************/

static inline void f50l_writedisable(FAR struct f50l_dev_s *priv)
{
  f50l_cmd(priv, F50L_WRITE_DISABLE, 0, 0, NULL, 0, 0);
}

/****************************************************************************
 * Name:  f50l_sectorerase (128K)
 ****************************************************************************/

static bool f50l_sectorerase(FAR struct f50l_dev_s *priv,
                             off_t startsector)
{
  const uint32_t block = startsector << (priv->sectorshift -
                                         priv->pageshift);

  finfo("block sector: %08lx\n", (long)block);

  f50l_writeenable(priv);

  f50l_cmd(priv, F50L_BLOCK_ERASE, block, 3,
           NULL, 0, F50L_CMD_ADDRESS);

  finfo("Erased\n");
  return f50l_waitstatus(priv, F50L_SR_E_FAIL, false);
}

/****************************************************************************
 * Name: f50l_erase
 ****************************************************************************/

static int f50l_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks)
{
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("Erase: startblock: %08lx nblocks: %d\n",
        (long)startblock,
        (int)nblocks);

  f50l_lock(priv->dev);
  f50l_waitstatus(priv, F50L_SR_OIP, false);

  while (blocksleft > 0)
    {
      if (!f50l_sectorerase(priv, startblock))
        {
          break;
        }

      startblock++;
      blocksleft--;
    }

  f50l_unlock(priv->dev);
  return nblocks - blocksleft;
}

/****************************************************************************
 * Name: f50l_readbuffer
 ****************************************************************************/

static void f50l_readbuffer(FAR struct f50l_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length)
{
  const uint16_t offset = address &
                          ((1 << priv->pageshift) - 1);

#ifdef CONFIG_MTD_F50L_QSPI
  f50l_memread(priv, F50L_READ_FROM_CACHE_X4, offset,
               1, true, buffer, length);
#else
  f50l_memread(priv, F50L_READ_FROM_CACHE, offset,
               1, false, buffer, length);
#endif
}

/****************************************************************************
 * Name: f50l_issue_page_read / f50l_wait_page_ready
 ****************************************************************************/

static void f50l_issue_page_read(FAR struct f50l_dev_s *priv,
                                 uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  f50l_cmd(priv, F50L_PAGE_READ, row, 3,
           NULL, 0, F50L_CMD_ADDRESS);
}

static bool f50l_wait_page_ready(FAR struct f50l_dev_s *priv)
{
  f50l_waitstatus(priv, F50L_SR_OIP, false);

  if ((priv->eccstatus & F50L_FEATURE_ECC_MASK) ==
      F50L_FEATURE_ECC_ERROR)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: f50l_read
 ****************************************************************************/

static ssize_t f50l_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;
  bool page_issued = false;

  finfo("Read: offset: %08lx nbytes: %d\n",
        (long)offset, (int)nbytes);

  f50l_lock(priv->dev);
  f50l_waitstatus(priv, F50L_SR_OIP, false);

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
          f50l_issue_page_read(priv, pageaddress);
          page_issued = true;
        }

      if (!f50l_wait_page_ready(priv))
        {
          break;
        }

      f50l_readbuffer(priv, position, buffer, chunklength);

      /* Pipeline: issue next PAGE_READ while we process current
       * data.  The NAND array-to-cache transfer (~25us) overlaps
       * with the caller consuming the buffer.
       */

      if (bytesleft > chunklength)
        {
          uint32_t nextpos = position + chunklength;
          uint32_t nextpage = (nextpos >> priv->pageshift) <<
                               priv->pageshift;
          f50l_issue_page_read(priv, nextpage);
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

  f50l_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: f50l_bread
 ****************************************************************************/

static ssize_t f50l_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks,
                          FAR uint8_t *buffer)
{
  ssize_t nbytes;
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;

  finfo("Bread: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  nbytes = f50l_read(dev, startblock << priv->pageshift,
                     nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: f50l_write_to_cache
 ****************************************************************************/

static void f50l_write_to_cache(FAR struct f50l_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length)
{
  const uint16_t offset = address &
                          ((1 << priv->pageshift) - 1);

  f50l_memwrite(priv, F50L_PROGRAM_LOAD, offset, false,
                buffer, length);
}

/****************************************************************************
 * Name: f50l_execute_write
 ****************************************************************************/

static bool f50l_execute_write(FAR struct f50l_dev_s *priv,
                               uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  f50l_cmd(priv, F50L_PROGRAM_EXECUTE, row, 3,
           NULL, 0, F50L_CMD_ADDRESS);

  return f50l_waitstatus(priv, F50L_SR_P_FAIL, false);
}

/****************************************************************************
 * Name: f50l_write
 ****************************************************************************/

static ssize_t f50l_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Write: offset: %08lx nbytes: %d\n",
        (long)offset, (int)nbytes);

  f50l_lock(priv->dev);
  f50l_waitstatus(priv, F50L_SR_OIP, false);

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

      f50l_writeenable(priv);
      f50l_write_to_cache(priv, position, buffer,
                          chunklength);
      if (!f50l_execute_write(priv, pageaddress))
        {
          break;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  f50l_unlock(priv->dev);

  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: f50l_bwrite
 ****************************************************************************/

static ssize_t f50l_bwrite(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR const uint8_t *buffer)
{
  ssize_t nbytes;

  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;

  finfo("Bwrite: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  nbytes = f50l_write(dev, startblock << priv->pageshift,
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

static int f50l_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd, unsigned long arg)
{
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
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

          ret = f50l_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result =
               (priv->eccstatus & F50L_FEATURE_ECC_MASK)
                >> F50L_FEATURE_ECC_OFFSET;

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
 * Name: f50l_isbad
 *
 * Description:
 *   Check if a block is bad by reading the bad block marker from the
 *   spare area (column 2048) of the first page in the block.
 *
 *   The on-chip ECC engine treats the entire 2112-byte page (main +
 *   spare) as protected data and will overwrite the raw marker byte
 *   with its decoded value.  ECC must be disabled while accessing the
 *   BBM byte, otherwise a bad marker (0x00) could be "corrected" back
 *   to 0xff and a good page could be misreported.  The original B0h
 *   register value is saved and restored so other bits (OTP_EN, etc.)
 *   are preserved.
 *
 *   Returns 0 if good, 1 if bad.
 *
 ****************************************************************************/

static int f50l_isbad(FAR struct mtd_dev_s *dev, off_t block)
{
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
  uint8_t marker;
  uint8_t cfg_saved;
  uint8_t cfg_noecc;
  uint32_t pageaddr;

  /* First page of the block */

  pageaddr = block << priv->sectorshift;

  f50l_lock(priv->dev);

  /* Disable ECC so the BBM byte is read raw (see description). */

  f50l_cmd(priv, F50L_GET_FEATURE, F50L_SECURE_OTP, 1,
           &cfg_saved, 1,
           F50L_CMD_ADDRESS | F50L_CMD_READDATA);

  cfg_noecc = cfg_saved & ~F50L_CFG_ECC;
  if (cfg_noecc != cfg_saved)
    {
      f50l_writeenable(priv);
      f50l_cmd(priv, F50L_SET_FEATURE, F50L_SECURE_OTP, 1,
               &cfg_noecc, 1,
               F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);
      f50l_writedisable(priv);
    }

  /* Issue PAGE READ to load page into cache */

  f50l_issue_page_read(priv, pageaddr);
  f50l_waitstatus(priv, F50L_SR_OIP, false);

  /* Read spare area byte 0 (column 2048) -- bad block marker */

#ifdef CONFIG_MTD_F50L_QSPI
  f50l_memread(priv, F50L_READ_FROM_CACHE_X4,
               (1 << priv->pageshift), 1, true, &marker, 1);
#else
  f50l_memread(priv, F50L_READ_FROM_CACHE,
               (1 << priv->pageshift), 1, false, &marker, 1);
#endif

  /* Restore ECC configuration */

  if (cfg_noecc != cfg_saved)
    {
      f50l_writeenable(priv);
      f50l_cmd(priv, F50L_SET_FEATURE, F50L_SECURE_OTP, 1,
               &cfg_saved, 1,
               F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);
      f50l_writedisable(priv);
    }

  f50l_unlock(priv->dev);

  /* 0xFF = good block, anything else = bad */

  return marker != 0xff ? 1 : 0;
}

/****************************************************************************
 * Name: f50l_markbad
 *
 * Description:
 *   Mark a block as bad by writing 0x00 to the spare area (column 2048)
 *   of the first page in the block.  ECC is disabled around the program
 *   operation so the marker byte is written raw and will not be altered
 *   by the ECC engine.  The B0h register is saved and restored.
 *
 ****************************************************************************/

static int f50l_markbad(FAR struct mtd_dev_s *dev, off_t block)
{
#ifdef CONFIG_MTD_READONLY
  return -EROFS;
#else
  FAR struct f50l_dev_s *priv = (FAR struct f50l_dev_s *)dev;
  uint8_t marker = 0x00;
  uint8_t cfg_saved;
  uint8_t cfg_noecc;
  uint32_t pageaddr;
  uint32_t row;
  int ret = OK;

  pageaddr = block << priv->sectorshift;
  row = pageaddr >> priv->pageshift;

  f50l_lock(priv->dev);

  /* Disable ECC so the BBM byte is programmed raw. */

  f50l_cmd(priv, F50L_GET_FEATURE, F50L_SECURE_OTP, 1,
           &cfg_saved, 1,
           F50L_CMD_ADDRESS | F50L_CMD_READDATA);

  cfg_noecc = cfg_saved & ~F50L_CFG_ECC;
  if (cfg_noecc != cfg_saved)
    {
      f50l_writeenable(priv);
      f50l_cmd(priv, F50L_SET_FEATURE, F50L_SECURE_OTP, 1,
               &cfg_noecc, 1,
               F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);
      f50l_writedisable(priv);
    }

  f50l_writeenable(priv);

  /* Write 0x00 to spare area byte 0 (column 2048) */

  f50l_memwrite(priv, F50L_PROGRAM_LOAD,
                (1 << priv->pageshift), false, &marker, 1);

  /* Execute program */

  f50l_cmd(priv, F50L_PROGRAM_EXECUTE, row, 3,
           NULL, 0, F50L_CMD_ADDRESS);

  if (!f50l_waitstatus(priv, F50L_SR_P_FAIL, false))
    {
      ferr("markbad program failed block=%ld\n", (long)block);
      ret = -EIO;
    }

  /* Restore ECC configuration */

  if (cfg_noecc != cfg_saved)
    {
      f50l_writeenable(priv);
      f50l_cmd(priv, F50L_SET_FEATURE, F50L_SECURE_OTP, 1,
               &cfg_saved, 1,
               F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);
      f50l_writedisable(priv);
    }

  f50l_unlock(priv->dev);

  return ret;
#endif /* CONFIG_MTD_READONLY */
}

/****************************************************************************
 * Name:  f50l_enable_ecc
 ****************************************************************************/

static inline void f50l_enable_ecc(
                          FAR struct f50l_dev_s *priv)
{
  /* ESMT has no QE bit — only enable ECC (bit 4 of B0h).
   * Quad I/O is activated by clearing WPE in A0h
   * (done in f50l_unlockblocks).
   */

  uint8_t cfg = F50L_CFG_ECC;

  f50l_lock(priv->dev);
  f50l_writeenable(priv);

  f50l_cmd(priv, F50L_SET_FEATURE, F50L_SECURE_OTP, 1,
           &cfg, 1,
           F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);

  f50l_writedisable(priv);
  f50l_unlock(priv->dev);
}

/****************************************************************************
 * Name:  f50l_unlockblocks
 ****************************************************************************/

static inline void f50l_unlockblocks(
                          FAR struct f50l_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  f50l_lock(priv->dev);
  f50l_writeenable(priv);

  f50l_cmd(priv, F50L_SET_FEATURE, F50L_BLOCK_PROTECTION,
           1, &blockprotection, 1,
           F50L_CMD_ADDRESS | F50L_CMD_WRITEDATA);

  f50l_writedisable(priv);
  f50l_unlock(priv->dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: f50l_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for SPI interface.
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions(such as a block
 *   or character driver front end).
 *
 ****************************************************************************/

#ifndef CONFIG_MTD_F50L_QSPI
FAR struct mtd_dev_s *f50l_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
  FAR struct f50l_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct f50l_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase   = f50l_erase;
      priv->mtd.bread   = f50l_bread;
      priv->mtd.bwrite  = f50l_bwrite;
      priv->mtd.ioctl   = f50l_ioctl;
      priv->mtd.isbad   = f50l_isbad;
      priv->mtd.markbad = f50l_markbad;
      priv->mtd.name    = "f50l";
      priv->dev         = dev;
      priv->spi_devid   = spi_devid;

      /* De-select the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(spi_devid), false);

      /* Reset the flash */

      f50l_cmd(priv, F50L_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      f50l_waitstatus(priv, F50L_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = f50l_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      f50l_enable_ecc(priv);
      f50l_waitstatus(priv, F50L_SR_OIP, false);
      f50l_unlockblocks(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}

#else /* CONFIG_MTD_F50L_QSPI */

/****************************************************************************
 * Name: f50l_qspi_initialize
 *
 * Description:
 *   Initialize GD5F SPI NAND over QSPI interface with Quad I/O
 *   support.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *f50l_qspi_initialize(
                                FAR struct qspi_dev_s *dev)
{
  FAR struct f50l_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = kmm_zalloc(sizeof(struct f50l_dev_s));
  if (priv)
    {
      priv->mtd.erase   = f50l_erase;
      priv->mtd.bread   = f50l_bread;
      priv->mtd.bwrite  = f50l_bwrite;
      priv->mtd.ioctl   = f50l_ioctl;
      priv->mtd.isbad   = f50l_isbad;
      priv->mtd.markbad = f50l_markbad;
      priv->mtd.name    = "f50l";
      priv->dev         = dev;

      /* Reset the flash */

      f50l_cmd(priv, F50L_RESET, 0, 0, NULL, 0, 0);

      /* Wait reset complete */

      f50l_waitstatus(priv, F50L_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = f50l_readid(priv);
      if (ret != OK)
        {
          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      f50l_enable_ecc(priv);
      f50l_waitstatus(priv, F50L_SR_OIP, false);
      f50l_unlockblocks(priv);
    }

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
#endif /* CONFIG_MTD_F50L_QSPI */
