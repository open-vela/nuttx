/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_shm.h
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

/* Contract between this core and Linux for the shared frame area.
 *
 * This file is the canonical definition of both the memory layout and the
 * message format. The Linux-side test program carries a copy; the two must be
 * kept in step, so any change here means changing that copy as well.
 *
 * The idea is that pixels never travel over rpmsg - only the notice that a
 * buffer is finished. rpmsg carries 16 bytes per frame regardless of
 * resolution, which is what makes this affordable: its buffer pool is 512-byte
 * buffers and would be hopeless for video, but is idle capacity for signalling.
 */

#ifndef __BOARDS_ARM64_RK3588_EVB7_AMP_SRC_EVB7_AMP_SHM_H
#define __BOARDS_ARM64_RK3588_EVB7_AMP_SRC_EVB7_AMP_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The shared area is amp-shmem@31000000 from the Linux device tree: 4MB,
 * declared no-map so the kernel never treats it as ordinary memory, and
 * already mapped MT_NORMAL_NC by rk3588_boot.c.
 *
 * Non-cacheable is not an optimisation choice, it is what makes this work
 * without cache maintenance on either side. An earlier cacheable mapping made
 * cross-core writes look frozen because they sat in this core's cache - see the
 * comment on the AMP_SHMEM entry in rk3588_boot.c.
 */

#define AMP_SHM_BASE       0x31000000
#define AMP_SHM_SIZE       (4 * 1024 * 1024)

#define AMP_SHM_MAGIC      0x30424641   /* "AFB0" as a little-endian word */
#define AMP_SHM_VERSION    1

/* Geometry.
 *
 * The panel is 1080x1920, but a full ARGB8888 frame is 7.91MiB and two would
 * not fit in the 4MB area. Half resolution double-buffered comes to 3.96MiB and
 * does fit, which keeps this stage free of device-tree changes: growing the
 * carveout would mean rebuilding and reflashing boot.img. Scaling back up to
 * the panel costs nothing, as both the VOP2 window and the DRM plane scale in
 * hardware.
 */

#define AMP_SHM_WIDTH      540
#define AMP_SHM_HEIGHT     960
#define AMP_SHM_BPP        4            /* ARGB8888 */
#define AMP_SHM_STRIDE     (AMP_SHM_WIDTH * AMP_SHM_BPP)
#define AMP_SHM_BUFSIZE    (AMP_SHM_STRIDE * AMP_SHM_HEIGHT)
#define AMP_SHM_NBUFFERS   2

/* The first page of the area is left alone, and that is not tidiness: something
 * outside this project writes to it.
 *
 * Measured, not assumed. Filling all 4MB with an address-derived pattern and
 * re-reading it after 60s showed exactly three words changed - at +0x0c, +0x10
 * and +0x14 - with the other 1048573 words untouched. Two of them are counters
 * incremented in place about a thousand times a second, four apart, written a
 * halfword at a time; the third holds a constant 30.
 *
 * The writer has not been identified. The region is genuinely reserved (absent
 * from /proc/iomem's System RAM, present under /proc/device-tree/reserved-memory)
 * so it is not the Linux allocator, and nothing in the kernel, u-boot or NuttX
 * trees references amp-shmem. The 1kHz rate does match this core's tick
 * (CONFIG_USEC_PER_TICK=1000), which is a lead rather than a conclusion.
 *
 * Since the footprint is small, stable and at the very start, skipping the whole
 * first page costs 4KB and removes the question from the data path. Anything
 * placed at offset 0 would be corrupted in three words - which is exactly what
 * happened to the geometry fields on the first attempt, while the checksums of
 * the buffers themselves came back correct.
 */

#define AMP_SHM_RESERVED_HEAD 4096

#define AMP_SHM_HDR_OFFSET (AMP_SHM_RESERVED_HEAD)
#define AMP_SHM_HDR_SIZE   4096
#define AMP_SHM_BUF_OFFSET(n) (AMP_SHM_HDR_OFFSET + AMP_SHM_HDR_SIZE + \
                               (n) * AMP_SHM_BUFSIZE)

/* Messages. Linux asks for a frame, this core answers when one is written, and
 * Linux reports back what it read. The reply is what turns this from "we think
 * the memory is shared" into a measurement: both sides checksum the same bytes
 * independently, one through this core's mapping and one through /dev/mem, and
 * the sums either agree or they do not.
 */

#define AMP_SHM_CMD_RENDER 1            /* Linux -> here: draw a frame     */
#define AMP_SHM_CMD_READY  2            /* here -> Linux: frame written    */
#define AMP_SHM_CMD_ACK    3            /* Linux -> here: what I read      */
#define AMP_SHM_CMD_TOUCH  4            /* Linux -> here: a touch event    */
#define AMP_SHM_CMD_HELLO  5            /* Linux -> here: I am listening   */

/* Why a hello is necessary rather than merely tidy.
 *
 * This core learns the peer's endpoint address only from the first message it
 * receives - see rpmsg_virtio_rx_callback(), which fills in ept->dest_addr when
 * it is still RPMSG_ADDR_ANY. Linux's rpmsg_char driver creates its endpoint
 * when user space opens /dev/rpmsgN and sends nothing at that point, so without
 * an explicit hello this core has no address to send to and every frame
 * notification is dropped.
 *
 * That is exactly what happened once an application on this side started
 * generating frames on its own: 36 frames were published to the control block
 * and never announced, and the screen stayed black until a touch event - a
 * message from Linux - incidentally taught this core the address.
 *
 * The earlier stages never caught it because every one of their test procedures
 * happened to have Linux speak first: the frame test asks for frames, and the
 * touch test is Linux-to-here by nature. A path that only works when the peer
 * happens to transmit first is not a working path.
 */

/* Touch contact state, matching the three states the framework distinguishes.
 * Kept as its own small set rather than reusing the TOUCH_* bits from
 * nuttx/input/touchscreen.h, because those are a NuttX header the Linux side
 * cannot include.
 */

#define AMP_TOUCH_DOWN     0
#define AMP_TOUCH_MOVE     1
#define AMP_TOUCH_UP       2

#define AMP_SHM_EPT_NAME   "rpmsg-raw"

/* Which buffer the framebuffer driver hands to applications.
 *
 * One buffer, not two: /dev/fb0 is mapped straight into the application's
 * address space, so a second buffer would only help if something here flipped
 * between them, and nothing does yet. Tearing is therefore possible if Linux
 * reads while an application is mid-draw; the frame sequence and checksum in the
 * control block make that visible rather than mysterious when it matters.
 */

#define AMP_SHM_FB_INDEX   0

/* "rpmsg-raw" is not an arbitrary choice: it is the only entry in the Linux
 * rpmsg_char driver's id table, so announcing this name is what makes the
 * kernel hand the channel to user space as /dev/rpmsgN with no kernel patch at
 * all. Contrast with the console channel, where using a custom name meant
 * adding an entry to rpmsg_nsh_tty's table.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Control block at AMP_SHM_BASE + AMP_SHM_HDR_OFFSET. Written by this core, read
 * by Linux.
 *
 * Everything is a fixed-width little-endian word so the layout does not depend
 * on either side's compiler. Linux discovers the geometry from here rather than
 * being told it, so changing resolution needs no change on that side.
 */

begin_packed_struct struct amp_shm_ctrl_s
{
  uint32_t magic;                       /* AMP_SHM_MAGIC once initialised   */
  uint32_t version;                     /* AMP_SHM_VERSION                  */
  uint32_t width;
  uint32_t height;
  uint32_t stride;                      /* bytes per row                    */
  uint32_t bpp;                         /* bytes per pixel                  */
  uint32_t nbuffers;
  uint32_t bufsize;                     /* bytes in one buffer              */
  uint32_t bufoffset[AMP_SHM_NBUFFERS]; /* from AMP_SHM_BASE                */
  uint32_t frame_seq;                   /* frames written since boot        */
  uint32_t ready_index;                 /* buffer holding the latest frame  */

  /* Checksum of that buffer, or zero to mean "not computed".
   *
   * Zero is a deliberate sentinel rather than a real sum. Checksumming a frame
   * means reading the whole buffer back out of non-cacheable memory, which is
   * affordable for a test that publishes a few frames on request and is not
   * affordable for a display path: at this resolution it is 2MB of uncached
   * reads on this side and another 2MB on Linux's, per frame.
   *
   * So the test pattern still publishes a sum - it gets one for free while
   * writing the pixels - and the framebuffer path publishes zero. Verification
   * lives in the mode built for verifying.
   */

  uint32_t ready_sum;
} end_packed_struct;

/* rpmsg payload: 16 bytes, same in both directions. */

begin_packed_struct struct amp_shm_msg_s
{
  uint32_t cmd;
  uint32_t seq;                         /* echoes frame_seq                 */
  uint32_t index;                       /* which buffer                     */
  uint32_t sum;                         /* checksum, per the sender         */
} end_packed_struct;

/* Touch events travel over the same endpoint, distinguished by cmd.
 *
 * Deliberately also 16 bytes, so a receiver can read one fixed-size message and
 * then look at cmd, instead of having to know the length before the read. The
 * two structures are different views of the same wire format.
 *
 * Pixels need shared memory; touch does not. A contact is a dozen bytes and
 * there are a few hundred per second at most, which is idle capacity for an
 * rpmsg pool built from 512-byte buffers - the exact opposite of the video case,
 * where that pool would be hopeless.
 *
 * x and y are already in this core's framebuffer coordinates. Linux does the
 * conversion because Linux is the side that knows both the panel geometry and
 * the touch controller's range; sending raw controller values would force this
 * core to learn about a device it cannot see, and would put the forward and
 * inverse scaling in two different programs where they could drift apart.
 */

begin_packed_struct struct amp_touch_msg_s
{
  uint32_t cmd;                         /* AMP_SHM_CMD_TOUCH                */
  uint32_t seq;                         /* events sent since boot           */
  uint16_t x;                           /* framebuffer coordinates          */
  uint16_t y;
  uint8_t  id;                          /* contact id, stable while touching */
  uint8_t  state;                       /* AMP_TOUCH_DOWN / MOVE / UP       */
  uint16_t pressure;                    /* 0 when the controller has none   */
  uint32_t reserved;
} end_packed_struct;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_shm_init
 *
 * Description:
 *   Initialise the control block and arm the rpmsg endpoint. Call after
 *   rk3588_rptun_init so the tunnel exists.
 *
 ****************************************************************************/

int evb7_amp_shm_init(const char *cpuname);

/****************************************************************************
 * Name: evb7_amp_shm_flush
 *
 * Description:
 *   Publish the framebuffer buffer: checksum it, describe it in the control
 *   block, and notify Linux. Called by the framebuffer driver when an
 *   application says it has finished drawing.
 *
 ****************************************************************************/

void evb7_amp_shm_flush(void);

/****************************************************************************
 * Name: evb7_amp_touch_event
 *
 * Description:
 *   Hand a touch event received over rpmsg to the touchscreen driver. Called
 *   from the shared-frame endpoint callback; defined in evb7_amp_touch.c and
 *   only present when the touchscreen support is enabled.
 *
 ****************************************************************************/

#if defined(CONFIG_INPUT_TOUCHSCREEN)
void evb7_amp_touch_event(const struct amp_touch_msg_s *msg);
#endif

#endif /* __BOARDS_ARM64_RK3588_EVB7_AMP_SRC_EVB7_AMP_SHM_H */
