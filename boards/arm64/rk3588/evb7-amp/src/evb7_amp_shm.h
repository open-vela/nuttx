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

/* Contract between this core and Linux: touch events in, and the geometry Linux
 * needs to scale them.
 *
 * This file is the canonical definition of both the memory layout and the
 * message format. The Linux-side program carries a copy; the two must be kept in
 * step, so any change here means changing that copy as well.
 *
 * Frames used to travel through here. They no longer do: this core programs the
 * VOP's Esmart3 window itself and the hardware scans out of this core's own RAM
 * (see evb7_amp_vop.c and evb7_amp_fb.c), so nothing about the display path
 * touches shared memory or rpmsg any more. What is left is the direction that
 * genuinely needs a message channel - touch, which this core cannot read for
 * itself because the controller's interrupt shares a GPIO bank with the Type-C
 * power delivery controller's.
 *
 * The frame protocol that used to live here - a render request, a ready
 * notification, a checksum acknowledgement, two 2MB buffers and a control block
 * describing them - is gone rather than kept for a rainy day. It was the right
 * design for "Linux composites what this core draws", and that is no longer what
 * happens; leaving it in place would mean two plausible-looking display paths
 * with only one of them real.
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
 *
 * Only two pages of the 4MB are used now that the frame buffers are gone: the
 * first is skipped (see AMP_SHM_RESERVED_HEAD) and the second holds the control
 * block. The carveout is left at its declared size because shrinking it means
 * changing the device tree and reflashing boot.img, which is not worth doing for
 * memory nothing else is asking for.
 */

#define AMP_SHM_BASE       0x31000000
#define AMP_SHM_SIZE       (4 * 1024 * 1024)

/* The magic changes with the layout, and that is the only thing that protects
 * against an old program reading a new control block.
 *
 * A version field alone does not, because only a program that knows to check it
 * will. The first cut of this change kept the magic and bumped the version, and
 * the result on hardware was worse than a clean failure: the previous host binary
 * found the magic it expected, read magic/version/width/height/stride/bpp - whose
 * order had not changed - and carried on with correct geometry, then read
 * nbuffers from an offset that is now past the end of the structure and got a
 * stale 2 left in the carveout by the previous run. The panel came up and touch
 * silently did not, which is the hardest kind of failure to place.
 *
 * So: bump the magic whenever the layout changes. An old program then stops at
 * its own magic check with a message, and a new program stops at the version
 * check if the firmware is the older side. Both directions fail loudly.
 */

#define AMP_SHM_MAGIC      0x31424641   /* "AFB1" as a little-endian word */
#define AMP_SHM_VERSION    2

/* Geometry of this core's framebuffer.
 *
 * Half the panel's 1080x1920, scaled back up by the VOP window in hardware. It
 * lives here rather than only in the framebuffer driver because it is part of
 * the contract: Linux converts touch coordinates from the controller's range
 * into these, so that the forward and inverse scaling do not end up in two
 * programs where they can drift apart.
 */

#define AMP_SHM_WIDTH      540
#define AMP_SHM_HEIGHT     960
#define AMP_SHM_BPP        4            /* ARGB8888 */
#define AMP_SHM_STRIDE     (AMP_SHM_WIDTH * AMP_SHM_BPP)

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

/* Messages. Both are Linux to here; nothing goes the other way any more.
 *
 * The gaps in the numbering are the frame protocol that used to be here -
 * RENDER(1), READY(2) and ACK(3). They are not reused, so a stale binary on
 * either side gets "unknown cmd" rather than being silently misinterpreted as
 * something else.
 */

#define AMP_SHM_CMD_TOUCH  4            /* Linux -> here: a touch event    */
#define AMP_SHM_CMD_HELLO  5            /* Linux -> here: I am listening   */

/* The hello is now only a connection trace, and that is a change worth noting.
 *
 * It used to be load-bearing: this core learns the peer's endpoint address only
 * from the first message it receives (rpmsg_virtio_rx_callback() fills in
 * ept->dest_addr while it is still RPMSG_ADDR_ANY), and Linux's rpmsg_char
 * driver transmits nothing when user space opens /dev/rpmsgN. So without a hello
 * this core had no address to send frame notifications to, and once an
 * application here started drawing on its own, 36 frames were published and
 * never announced - a black screen until a touch event incidentally taught this
 * core the address.
 *
 * Nothing is sent from here any more, so that failure mode is gone with the
 * frame protocol. The message is kept because it is a cheap and unambiguous
 * marker in the log that the channel came up at all.
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
 *
 * It is only geometry now. The frame bookkeeping that used to follow -
 * nbuffers, bufsize, bufoffset[], frame_seq, ready_index and a checksum - went
 * with the frame protocol. The magic is unchanged but the version is not, so a
 * mismatched pair of binaries is caught rather than reading the geometry
 * correctly and then disagreeing about everything after it.
 */

begin_packed_struct struct amp_shm_ctrl_s
{
  uint32_t magic;                       /* AMP_SHM_MAGIC once initialised   */
  uint32_t version;                     /* AMP_SHM_VERSION                  */
  uint32_t width;
  uint32_t height;
  uint32_t stride;                      /* bytes per row                    */
  uint32_t bpp;                         /* bytes per pixel                  */
} end_packed_struct;

/* Every message on this endpoint is 16 bytes and starts with cmd, so a receiver
 * can read one fixed-size message and then decide how to interpret it, instead
 * of having to know the length before the read. This is the header view; the
 * touch structure below is the same 16 bytes seen in full.
 */

begin_packed_struct struct amp_shm_hdr_s
{
  uint32_t cmd;
  uint32_t pad[3];
} end_packed_struct;

/* Touch events, distinguished by cmd.
 *
 * Touch does not need shared memory the way pixels did. A contact is a dozen
 * bytes and there are a few hundred per second at most, which is idle capacity
 * for an rpmsg pool built from 512-byte buffers - the exact opposite of the
 * video case, where that pool would have been hopeless.
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
 *   Publish the geometry in the control block and arm the rpmsg endpoint that
 *   carries touch. Call after rk3588_rptun_init so the tunnel exists.
 *
 ****************************************************************************/

int evb7_amp_shm_init(const char *cpuname);

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
