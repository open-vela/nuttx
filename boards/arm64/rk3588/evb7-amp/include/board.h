/****************************************************************************
 * boards/arm64/rk3588/evb7_amp/include/board.h
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

#ifndef __BOARDS_ARM64_RK3588_EVB7_AMP_INCLUDE_BOARD_H
#define __BOARDS_ARM64_RK3588_EVB7_AMP_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/fs/ioctl.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

typedef enum
{
    BOARD_LED1 = 0,  /* Green LED */
    BOARD_LED2 = 1,  /* Red LED */
    BOARD_LED3 = 2,  /* Blue LED */
    BOARD_LEDS       /* Number of LEDs */
} led_typedef_enum;

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/pinephone_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        LED1  LED2  LED3
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON     OFF   OFF  */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    OFF   ON   */
#define LED_INIRQ          4 /* In an interrupt          ON     ON    OFF  */
#define LED_SIGNAL         5 /* In a signal handler      ON     OFF   ON   */
#define LED_ASSERTION      6 /* An assertion failed      OFF    ON    ON   */
#define LED_PANIC          7 /* The system has crashed   FLASH  ON    ON   */
#define LED_IDLE           8 /* MCU is is sleep mode     OFF    FLASH OFF  */

/* Camera device, /dev/amcam0 ***********************************************/

/* Frames captured by Linux and handed over through the shared carveout. See
 * src/evb7_amp_shm.h for why this direction uses shared memory when the display
 * no longer does, and src/evb7_amp_cam.c for the transport itself.
 *
 * These are here, in the board's public header, rather than in src/ so an
 * application can reach them through <arch/board/board.h> and there is exactly
 * one definition of each. Copying the ioctl numbers into the application is the
 * one thing not to do: the shared-memory header carries a duplicate of its layout
 * on the Linux side only because Linux cannot include a NuttX header, and that
 * excuse does not exist between two NuttX translation units.
 *
 * read() returns one whole frame, tightly packed at width * bpp, and blocks until
 * there is a frame newer than the one it last returned. A short buffer gets a
 * truncated frame rather than an error.
 */

/* 0xfe00 is unused by include/nuttx/fs/ioctl.h, whose bases run to 0x4600 and
 * then jump to 0x8b00, 0x8c00 and 0xff00. Defined here instead of added there
 * because this is one board's device and does not belong in a shared header. The
 * trade is that a future upstream base at 0xfe00 would collide, which is why the
 * number is at the far end rather than next in sequence.
 */

#ifndef __ASSEMBLY__

#define _AMPCAMBASE        (0xfe00)
#define _AMPCAMIOC(nr)     _IOC(_AMPCAMBASE, nr)

#define AMPCAMIOC_GETGEOM  _AMPCAMIOC(1)  /* arg: struct ampcam_geom_s *   */
#define AMPCAMIOC_GETSTATS _AMPCAMIOC(2)  /* arg: struct ampcam_stats_s *  */
#define AMPCAMIOC_GETDET   _AMPCAMIOC(3)  /* arg: struct ampcam_detect_s * */

/* Geometry of what read() will return. A zero width means Linux has not started
 * streaming. That is reported rather than failed, because "no producer yet" is
 * the normal state at boot and a caller usually wants to wait, not give up.
 */

struct ampcam_geom_s
{
  uint32_t width;
  uint32_t height;
  uint32_t stride;                 /* bytes per row as read() returns them */
  uint32_t bpp;                    /* 4, XRGB8888, same as the framebuffer */
  uint32_t seq;                    /* sequence of the most recent frame    */
};

/* Transport counters, kept apart because they mean different things and want
 * different fixes: notify far ahead of copied is this core not keeping up, torn
 * is the publication race, bad means Linux published a descriptor that does not
 * validate - a bug over there rather than a timing problem here.
 */

struct ampcam_stats_s
{
  uint32_t notify;                 /* notifications received               */
  uint32_t copied;                 /* frames handed to a reader            */
  uint32_t torn;                   /* reads restarted, frame changed       */
  uint32_t bad;                    /* descriptor failed validation         */
  uint32_t dropped;                /* published by Linux, never read here  */

  /* Detections, counted apart from frames because they arrive at about a third
   * of the frame rate: folded together, a healthy detector would look like a
   * failing frame path.
   */

  uint32_t det_notify;
  uint32_t det_read;
  uint32_t det_torn;
  uint32_t det_bad;
};

/* One detected object, in the coordinate space of the frame read() returns.
 *
 * Not framebuffer coordinates: Linux resolves the sensor, the second ISP stream
 * and the model's letterboxing down to this frame's pixels, and the caller adds
 * wherever it decided to draw the frame. Each side owns the transform it can
 * actually see.
 */

struct ampcam_box_s
{
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  uint8_t  cls;                    /* COCO class id; 0 is person           */
  uint8_t  score;                  /* confidence, 0..100                   */
  uint16_t reserved;
};

#define AMPCAM_MAXBOX 64           /* matches the producer's own cap        */

/* A whole set of detections, or none.
 *
 * count is -1 rather than 0 when nothing is producing, so that "no detector" and
 * "detector running, nothing in frame" are distinguishable. Conflating them
 * would make an empty scene indistinguishable from a dead pipeline, which is the
 * same mistake as a report that says nothing when its ioctl fails.
 *
 * seq changes when the set does, so a caller redrawing every frame can tell
 * whether the boxes are new. latency_us is the producer's own end-to-end time,
 * which separates "stale" from "wrong" without instrumenting the other side.
 */

struct ampcam_detect_s
{
  int32_t  count;                  /* boxes valid, or -1 if not producing  */
  uint32_t seq;
  uint32_t width;                  /* coordinate space of the boxes        */
  uint32_t height;
  uint32_t latency_us;
  struct ampcam_box_s box[AMPCAM_MAXBOX];
};

#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM64_RK3588_EVB7_AMP_INCLUDE_BOARD_H */
