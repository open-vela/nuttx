/****************************************************************************
 * include/nuttx/usb/usbhost_uvc.h
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

#ifndef __INCLUDE_NUTTX_USB_USBHOST_UVC_H
#define __INCLUDE_NUTTX_USB_USBHOST_UVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/uvc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UVC Video Class version */

#define UVC_VERSION_1_0             0x0100
#define UVC_VERSION_1_1             0x0110
#define UVC_VERSION_1_5             0x0150

/* UVC Interface subclass codes */

#define UVC_SUBCLASS_CONTROL        0x01
#define UVC_SUBCLASS_STREAMING      0x02
#define UVC_SUBCLASS_INTERFACE_COLLECTION 0x03

/* UVC Descriptor Types */

#define UVC_CS_INTERFACE            0x24
#define UVC_CS_ENDPOINT             0x25

/* UVC VideoControl Interface Descriptor Subtypes */

#define UVC_VC_DESCRIPTOR_UNDEFINED 0x00
#define UVC_VC_HEADER              0x01
#define UVC_VC_INPUT_TERMINAL      0x02
#define UVC_VC_OUTPUT_TERMINAL     0x03
#define UVC_VC_SELECTOR_UNIT       0x04
#define UVC_VC_PROCESSING_UNIT     0x05
#define UVC_VC_EXTENSION_UNIT      0x06

/* UVC VideoStreaming Interface Descriptor Subtypes */

#define UVC_VS_DESCRIPTOR_UNDEFINED 0x00
#define UVC_VS_INPUT_HEADER        0x01
#define UVC_VS_OUTPUT_HEADER       0x02
#define UVC_VS_STILL_IMAGE_FRAME   0x03
#define UVC_VS_FORMAT_UNCOMPRESSED 0x04
#define UVC_VS_FRAME_UNCOMPRESSED  0x05
#define UVC_VS_FORMAT_MJPEG        0x06
#define UVC_VS_FRAME_MJPEG         0x07

/* UVC Request Codes */

#define UVC_SET_CUR                0x01
#define UVC_GET_CUR                0x81
#define UVC_GET_MIN                0x82
#define UVC_GET_MAX                0x83
#define UVC_GET_RES                0x84
#define UVC_GET_LEN                0x85
#define UVC_GET_INFO               0x86
#define UVC_GET_DEF                0x87

/* UVC VideoStreaming Interface Control Selectors */

#define UVC_VS_PROBE_CONTROL       0x01
#define UVC_VS_COMMIT_CONTROL      0x02

/* UVC Payload Header Flags */

/* UVC 1.5 Table 2-6: Payload Header bmHeaderInfo bit definitions */

#define UVC_HEADER_FID             (1 << 0)  /* Frame ID */
#define UVC_HEADER_EOF             (1 << 1)  /* End of Frame */
#define UVC_HEADER_PTS             (1 << 2)  /* Presentation Time Stamp */
#define UVC_HEADER_SCR             (1 << 3)  /* Source Clock Reference */
                                             /* bit 4: reserved */
#define UVC_HEADER_STI             (1 << 5)  /* Still Image */
#define UVC_HEADER_ERR             (1 << 6)  /* Error */
#define UVC_HEADER_EOH             (1 << 7)  /* End of Header */

/* UVC Probe/Commit Control Structure Size (UVC 1.0) */

#define UVC_PROBE_COMMIT_SIZE      26

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* UVC Format Description */

struct uvc_format_desc_s
{
  uint8_t  format_index;       /* bFormatIndex */
  uint8_t  num_frames;         /* Number of frame descriptors */
  uint32_t fourcc;             /* V4L2 pixel format */
  uint8_t  guid[16];           /* Format GUID */
  uint8_t  bits_per_pixel;
};

/* UVC Frame Description */

struct uvc_frame_desc_s
{
  uint8_t  frame_index;        /* bFrameIndex */
  uint16_t width;
  uint16_t height;
  uint32_t min_bitrate;
  uint32_t max_bitrate;
  uint32_t max_video_frame_buf_size;
  uint32_t default_frame_interval;
  uint8_t  frame_interval_type;
  uint32_t *intervals;         /* Array of frame intervals in 100ns units */
};

/* UVC Device Information */

struct uvc_device_info_s
{
  /* VideoControl interface */

  uint8_t  vc_interface;       /* VC interface number */
  uint16_t uvc_version;        /* UVC specification version */
  uint32_t clock_frequency;    /* Device clock frequency */

  /* VideoStreaming interface */

  uint8_t  vs_interface;       /* VS interface number */
  uint8_t  vs_alt_setting;     /* Streaming alt setting */
  uint8_t  ep_addr;            /* Streaming endpoint address */
  uint16_t ep_maxpacketsize;   /* Max packet size */

  /* Format/Frame information */

  uint8_t  num_formats;        /* Number of supported formats */
  struct uvc_format_desc_s *formats;  /* Array of format descriptors */

  uint8_t  num_frames;         /* Total number of frame descriptors */
  struct uvc_frame_desc_s *frames;    /* Array of frame descriptors */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_uvc_init
 *
 * Description:
 *   Initialize the USB Video Class host driver. This function should be
 *   called from board-specific initialization code to register support
 *   for UVC devices.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int usbhost_uvc_init(void);

#endif /* __INCLUDE_NUTTX_USB_USBHOST_UVC_H */
