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
 *
 * Camera frames do travel through here, and that is not a reversal of the above.
 * The display moved out because this core can program the VOP window itself; the
 * camera cannot be taken the same way. Its pipeline is imx415 -> csi2_dphy0 ->
 * mipi2_csi2 -> rkcif -> rkisp0, which needs the same kind of deep Linux-side
 * bring-up the DSI panel does, and what it produces is already an image sitting
 * in memory. So the data genuinely originates on the Linux side and the handover
 * point is genuinely memory. The test has always been "which side can actually
 * drive the hardware", never "is shared memory allowed".
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
#define AMP_SHM_VERSION    4

/* The magic is deliberately NOT bumped for the camera addition, which needs
 * saying because the paragraph above says to bump it whenever the layout
 * changes.
 *
 * What that rule guards against is an old reader taking fields from the wrong
 * offsets. The camera did not change this structure: every field below is the
 * same size, in the same order, at the same offset, and the camera state lives in
 * a separate block with its own magic. An old reader therefore gets correct
 * values for everything it knows how to read, which is the opposite of the A-13
 * failure that motivated the rule - there, a field had been removed and the old
 * reader picked up a stale word from past the end of the structure.
 *
 * The version still moves, 2 -> 3, and that is what catches a mismatched pair.
 * It now means "this side understands the camera blocks as well". Both directions
 * stop loudly: the Linux program compares it and says which side to rebuild.
 */

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

/* Camera area. Linux writes all of it; this core only reads.
 *
 *   0x000000  skipped, 4KB          - something outside this project writes here
 *   0x001000  fb control block      - written here, read by Linux
 *   0x002000  camera descriptor     - written by Linux, read here
 *   0x100000  camera buffer 0       - 1MB slot
 *   0x200000  camera buffer 1       - 1MB slot
 *   0x300000  spare to end of 4MB   - room for a third buffer
 *
 * Two blocks rather than camera fields appended to the control block, because the
 * control block's contract is "written here, read by Linux" and the camera's
 * geometry is something only Linux knows - it is the result of the VIDIOC_S_FMT
 * that Linux issues against the ISP. Adding those fields to the existing block
 * would give one structure two writers on two cores, which is a class of bug that
 * is very hard to see from either side. Keeping them apart also leaves
 * amp_shm_ctrl_s byte-for-byte unchanged, so an older host binary still reads the
 * geometry correctly instead of half-correctly.
 *
 * The buffer slots are 1MB and 1MB-aligned rather than sized to the frame. The
 * alignment makes the offsets checkable at a glance, and the slack means changing
 * capture resolution does not move anything. There is no cost: the 4MB carveout
 * already exists and nothing else is asking for it.
 *
 * A slot holds any frame up to 1MB, so 640x360x4 (921600) fits with room to
 * spare. This core validates stride * height against the slot size rather than
 * trusting the descriptor, since a bad descriptor would otherwise read - or worse,
 * be copied - past the end of the area.
 */

#define AMP_CAM_DESC_OFFSET  (AMP_SHM_HDR_OFFSET + AMP_SHM_HDR_SIZE)
#define AMP_CAM_DESC_SIZE    4096

#define AMP_CAM_NBUFFERS     2
#define AMP_CAM_SLOT_SIZE    0x100000
#define AMP_CAM_BUF0_OFFSET  0x100000
#define AMP_CAM_BUF1_OFFSET  0x200000

/* Default capture geometry. 16:9 to match the sensor, both dimensions a multiple
 * of 16 because the ISP's scaler wants aligned output, and small enough that the
 * per-frame copy on this side stays well inside the frame budget.
 *
 * These are only defaults for the Linux side to start from. The actual geometry
 * is published in the descriptor and read from there, so capturing at a different
 * size needs no change on this side.
 */

#define AMP_CAM_DEF_WIDTH    512
#define AMP_CAM_DEF_HEIGHT   288
#define AMP_CAM_BPP          4          /* XRGB8888, as the framebuffer is */

#define AMP_CAM_MAGIC        0x314d4143 /* "CAM1" as a little-endian word */
#define AMP_CAM_VERSION      1

/* Detection results. Written by Linux, read here.
 *
 * A third block rather than fields bolted onto the camera descriptor, for the
 * same reason that one is separate from the framebuffer control block: one
 * writer each. This one is also a different rate and a different lifetime - the
 * camera descriptor's geometry is written once and its indices change thirty
 * times a second, while this changes about ten times a second in its entirety.
 *
 * It sits in the page after the camera descriptor, which leaves the megabyte
 * from 0x3000 to the first frame slot still untouched and the spare megabyte
 * after the last slot still spare.
 *
 * Why shared memory at all, when touch - which is also small - goes entirely in
 * rpmsg messages: a set of boxes is a snapshot, not a stream of events. What a
 * consumer needs is every box from one frame and no box from another, and the
 * count varies. Messages are a fixed sixteen bytes here, so a variable-length
 * set would have to be reassembled from several of them with its own framing and
 * its own way of going wrong. The seqlock already in use for frames gives
 * exactly the all-or-nothing read this wants.
 */

#define AMP_DET_DESC_OFFSET  (AMP_CAM_DESC_OFFSET + AMP_CAM_DESC_SIZE)
#define AMP_DET_DESC_SIZE    4096

/* Sixty-four, because that is what the producer's post-processing caps itself at
 * (OBJ_NUMB_MAX_SIZE in the vendor yolov5 post-process). Matching it means this
 * side never has to decide what to drop.
 */

#define AMP_DET_MAXBOX       64

#define AMP_DET_MAGIC        0x31544544 /* "DET1" as a little-endian word */
#define AMP_DET_VERSION      1

/* Messages. Both are Linux to here; nothing goes the other way any more.
 *
 * The gaps in the numbering are the frame protocol that used to be here -
 * RENDER(1), READY(2) and ACK(3). They are not reused, so a stale binary on
 * either side gets "unknown cmd" rather than being silently misinterpreted as
 * something else.
 */

#define AMP_SHM_CMD_TOUCH  4            /* Linux -> here: a touch event    */
#define AMP_SHM_CMD_HELLO  5            /* Linux -> here: I am listening   */
#define AMP_SHM_CMD_CAMERA 6            /* Linux -> here: a frame is ready */
#define AMP_SHM_CMD_DETECT 7            /* Linux -> here: new detections   */

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

/* Camera descriptor at AMP_SHM_BASE + AMP_CAM_DESC_OFFSET. Written by Linux,
 * read here.
 *
 * seq and ready are the publication protocol, and the order they are touched in
 * matters. The shared area is mapped MT_NORMAL_NC, which removes the need for
 * cache maintenance on either side - that part is why this works at all, see the
 * AMP_SHMEM entry in rk3588_boot.c - but Normal-NC says nothing about ordering.
 * Accesses to it can still be observed out of order, so both sides use explicit
 * barriers:
 *
 *   Linux publishing:  write pixels -> barrier -> write ready -> barrier -> ++seq
 *   this core reading:  read seq -> barrier -> read ready and pixels ->
 *                       barrier -> read seq again and compare
 *
 * The re-read is a seqlock, and it is here because "unlikely" and "cannot happen"
 * are different things. Linux swaps buffers every 33ms at 30fps and the copy on
 * this side takes a couple of milliseconds, so a torn frame is rare - but when it
 * does happen it shows up as an occasional band of the wrong image, which is
 * about the hardest symptom to attribute after the fact.
 *
 * width/height/stride are here rather than fixed in this header because Linux is
 * the side that chooses them: they are whatever the ISP accepted from
 * VIDIOC_S_FMT. This core validates them against the slot size before using them
 * instead of trusting them, because a wrong stride would otherwise be copied out
 * of bounds.
 */

begin_packed_struct struct amp_cam_desc_s
{
  uint32_t magic;                       /* AMP_CAM_MAGIC once streaming     */
  uint32_t version;                     /* AMP_CAM_VERSION                  */
  uint32_t width;
  uint32_t height;
  uint32_t stride;                      /* bytes per row                    */
  uint32_t bpp;                         /* bytes per pixel, XRGB8888 -> 4   */
  uint32_t nbuffers;                    /* AMP_CAM_NBUFFERS                 */
  uint32_t bufsize;                     /* bytes actually used in a slot    */
  uint32_t bufoffset[AMP_CAM_NBUFFERS]; /* from AMP_SHM_BASE                */
  uint32_t seq;                         /* frames published since streaming */
  uint32_t ready;                       /* index of the newest full buffer  */
} end_packed_struct;

/* Frame notification. Sixteen bytes like every other message on this endpoint.
 *
 * The index in here is a hint and this core does not act on it: it reads the
 * descriptor instead. rpmsg messages can be dropped when the pool is busy, and
 * two frames can be published between two wakeups. Treating the message as a
 * wakeup and the descriptor as the truth means a lost notification costs a
 * dropped frame, whereas trusting the message would mean displaying the buffer
 * Linux is currently writing into.
 */

begin_packed_struct struct amp_cam_msg_s
{
  uint32_t cmd;                         /* AMP_SHM_CMD_CAMERA               */
  uint32_t seq;                         /* matches desc->seq at send time   */
  uint32_t ready;                       /* hint only; descriptor is truth   */
  uint32_t reserved;
} end_packed_struct;

/* One detected object.
 *
 * Coordinates are in the published camera frame's space - the width and height
 * the camera descriptor advertises - not in the framebuffer's and not in the
 * model's. That split is deliberate and it is the same one touch uses: Linux
 * knows the relationship between the two ISP streams and the letterboxing the
 * model needed, so Linux resolves all of that; this side knows where on screen
 * it chose to draw the image, so it adds only that offset. Publishing
 * framebuffer coordinates would require Linux to know a placement it has no way
 * to see, and publishing model coordinates would put the letterbox arithmetic on
 * both sides of the link where the two copies could drift.
 *
 * score is 0..100 rather than a float, because a percentage is all anything here
 * displays and a float in a cross-core structure invites questions about
 * representation that a byte does not.
 */

begin_packed_struct struct amp_det_box_s
{
  uint16_t x;                           /* left, in camera frame pixels     */
  uint16_t y;                           /* top                              */
  uint16_t w;
  uint16_t h;
  uint8_t  cls;                         /* COCO class id; 0 is person       */
  uint8_t  score;                       /* confidence, 0..100               */
  uint16_t reserved;
} end_packed_struct;

/* Detection descriptor at AMP_SHM_BASE + AMP_DET_DESC_OFFSET. Written by Linux,
 * read here.
 *
 * Published under a full seqlock, which is a stronger scheme than the frames
 * use, and the difference is not incidental.
 *
 * There are two frame slots, so Linux writes the one that is not published and a
 * reader never looks at memory being written; the sequence number changing once
 * at publication is enough. There is one detection block, updated in place, so
 * the same reader could copy half an old set and half a new one while the
 * sequence number still held its original value.
 *
 * Hence: the writer increments seq to an odd value, writes the boxes and the
 * count, then increments it to the next even one. A reader that finds seq odd
 * knows a write is in progress and starts over; a reader that finds it even and
 * unchanged after the copy has a set that was simultaneously true.
 *
 *   producer   ++seq (odd) -> barrier -> boxes, count -> barrier -> ++seq (even)
 *   consumer   seq even? -> barrier -> read -> barrier -> seq unchanged?
 *
 * The boxes lag the picture, and that is by design rather than a defect to be
 * chased. Detection runs at about a third of the display rate and one inference
 * takes some tens of milliseconds, so at the moment a frame is drawn the newest
 * available boxes describe a frame up to about 125ms older. For a person walking
 * that is a visible fraction of a step. Closing the gap means running detection
 * faster - there is headroom for it - not changing anything here.
 *
 * latency_us is what the producer took end to end for this set. It is here so
 * that "the boxes are stale" can be told apart from "the boxes are wrong"
 * without instrumenting the other side.
 */

begin_packed_struct struct amp_det_desc_s
{
  uint32_t magic;                        /* AMP_DET_MAGIC once producing    */
  uint32_t version;                      /* AMP_DET_VERSION                 */
  uint32_t seq;                          /* even is stable, odd is writing  */
  uint32_t count;                        /* 0 .. AMP_DET_MAXBOX             */
  uint32_t width;                        /* coordinate space of the boxes   */
  uint32_t height;
  uint32_t latency_us;                   /* producer's own end-to-end time  */
  uint32_t reserved;
  struct amp_det_box_s box[AMP_DET_MAXBOX];
} end_packed_struct;

/* Doorbell for a new set. Sixteen bytes like every other message here, and like
 * the camera notification it is only a wakeup: the count in it is a hint and the
 * descriptor is the truth, so a message lost to a busy pool costs a set that is
 * never drawn rather than a set read while it was being written.
 */

begin_packed_struct struct amp_det_msg_s
{
  uint32_t cmd;                         /* AMP_SHM_CMD_DETECT               */
  uint32_t seq;                         /* matches desc->seq at send time   */
  uint32_t count;                       /* hint only; descriptor is truth   */
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

/****************************************************************************
 * Name: evb7_amp_cam_init
 *
 * Description:
 *   Arm the camera consumer. Safe to call before Linux starts streaming - the
 *   descriptor is checked on every access rather than once at init, because the
 *   producer is another operating system that can start, stop and restart
 *   underneath this one.
 *
 ****************************************************************************/

int evb7_amp_cam_init(void);

/****************************************************************************
 * Name: evb7_amp_cam_notify
 *
 * Description:
 *   Wake whoever is waiting for a frame. Called from the shared endpoint
 *   callback on AMP_SHM_CMD_CAMERA.
 *
 ****************************************************************************/

void evb7_amp_cam_notify(const struct amp_cam_msg_s *msg);

/****************************************************************************
 * Name: evb7_amp_cam_wait
 *
 * Description:
 *   Block until a frame notification arrives or timeout_ms elapses; negative
 *   waits forever. Returns OK, -ETIMEDOUT, or a negated errno.
 *
 ****************************************************************************/

int evb7_amp_cam_wait(int timeout_ms);

/****************************************************************************
 * Name: evb7_amp_cam_copy
 *
 * Description:
 *   Copy the newest complete frame into dst, clipped to maxw/maxh, writing one
 *   row every dststride bytes. Returns -EAGAIN if Linux kept overwriting it,
 *   -ENODEV if Linux is not streaming.
 *
 ****************************************************************************/

int evb7_amp_cam_copy(void *dst, uint32_t dststride,
                      uint32_t maxw, uint32_t maxh,
                      uint32_t *width, uint32_t *height, uint32_t *seq);

/****************************************************************************
 * Name: evb7_amp_cam_stats
 *
 * Description:
 *   Transport counters. Any pointer may be NULL.
 *
 ****************************************************************************/

void evb7_amp_cam_stats(uint32_t *notify, uint32_t *copied, uint32_t *torn,
                        uint32_t *bad, uint32_t *dropped);

/****************************************************************************
 * Name: evb7_amp_cam_geom
 *
 * Description:
 *   Current capture geometry as Linux published it, with the stride a reader
 *   will see rather than the one in shared memory. -ENODEV until Linux streams.
 *
 ****************************************************************************/

int evb7_amp_cam_geom(uint32_t *width, uint32_t *height, uint32_t *seq);

/****************************************************************************
 * Name: evb7_amp_cam_register
 *
 * Description:
 *   Register the frame device, normally /dev/amcam0. Called from
 *   evb7_amp_cam_init; separate so a different path can be used in a test.
 *
 ****************************************************************************/

int evb7_amp_cam_register(const char *path);

/****************************************************************************
 * Name: evb7_amp_det_notify
 *
 * Description:
 *   Note that a new set of detections has been published. Called from the
 *   shared endpoint callback on AMP_SHM_CMD_DETECT.
 *
 *   Nothing is read here: this runs on the rpmsg receive path, and the reader is
 *   whoever asks for the results, not this callback.
 *
 ****************************************************************************/

void evb7_amp_det_notify(const struct amp_det_msg_s *msg);

/****************************************************************************
 * Name: evb7_amp_det_read
 *
 * Description:
 *   Copy the newest complete set of detections, up to maxbox of them, with the
 *   same seqlock the frames use. Returns the number copied, -EAGAIN if Linux
 *   kept replacing it, or -ENODEV if nothing is producing.
 *
 ****************************************************************************/

int evb7_amp_det_read(struct amp_det_box_s *dst, unsigned int maxbox,
                      uint32_t *seq, uint32_t *width, uint32_t *height,
                      uint32_t *latency_us);

/****************************************************************************
 * Name: evb7_amp_det_stats
 *
 * Description:
 *   Detection transport counters, kept apart from the frame ones because they
 *   run at a different rate. Any pointer may be NULL.
 *
 ****************************************************************************/

void evb7_amp_det_stats(uint32_t *notify, uint32_t *read, uint32_t *torn,
                        uint32_t *bad);

#endif /* __BOARDS_ARM64_RK3588_EVB7_AMP_SRC_EVB7_AMP_SHM_H */
