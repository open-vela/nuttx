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
  uint32_t ready_sum;                   /* checksum of that buffer          */
} end_packed_struct;

/* rpmsg payload: 16 bytes, same in both directions. */

begin_packed_struct struct amp_shm_msg_s
{
  uint32_t cmd;
  uint32_t seq;                         /* echoes frame_seq                 */
  uint32_t index;                       /* which buffer                     */
  uint32_t sum;                         /* checksum, per the sender         */
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

#endif /* __BOARDS_ARM64_RK3588_EVB7_AMP_SRC_EVB7_AMP_SHM_H */
