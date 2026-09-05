/****************************************************************************
 * include/nuttx/usb/uvc.h
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

#ifndef __INCLUDE_NUTTX_USB_UVC_H
#define __INCLUDE_NUTTX_USB_UVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UVC Interface Subclass Codes (UVC 1.5 Table A-2) */

#define USB_UVC_SUBCLASS_UNDEFINED              0x00
#define USB_UVC_SUBCLASS_CONTROL                0x01
#define USB_UVC_SUBCLASS_STREAMING              0x02
#define USB_UVC_SUBCLASS_INTERFACE_COLLECTION    0x03

/* UVC Interface Protocol Codes */

#define USB_UVC_PROTOCOL_UNDEFINED              0x00
#define USB_UVC_PROTOCOL_15                     0x01

/* Video Interface Class-Specific Descriptor Types (UVC 1.5 Table A-4) */

#define USB_UVC_DESC_TYPE_CS_INTERFACE          0x24
#define USB_UVC_DESC_TYPE_CS_ENDPOINT           0x25

/* VideoControl Interface Descriptor Subtypes (UVC 1.5 Table A-5) */

#define USB_UVC_VC_DESC_SUBTYPE_UNDEFINED       0x00
#define USB_UVC_VC_DESC_SUBTYPE_HEADER          0x01
#define USB_UVC_VC_DESC_SUBTYPE_INPUT_TERMINAL  0x02
#define USB_UVC_VC_DESC_SUBTYPE_OUTPUT_TERMINAL 0x03
#define USB_UVC_VC_DESC_SUBTYPE_SELECTOR_UNIT   0x04
#define USB_UVC_VC_DESC_SUBTYPE_PROCESSING_UNIT 0x05
#define USB_UVC_VC_DESC_SUBTYPE_EXTENSION_UNIT  0x06
#define USB_UVC_VC_DESC_SUBTYPE_ENCODING_UNIT   0x07

/* VideoStreaming Interface Descriptor Subtypes (UVC 1.5 Table A-6) */

#define USB_UVC_VS_DESC_SUBTYPE_UNDEFINED           0x00
#define USB_UVC_VS_DESC_SUBTYPE_INPUT_HEADER        0x01
#define USB_UVC_VS_DESC_SUBTYPE_OUTPUT_HEADER       0x02
#define USB_UVC_VS_DESC_SUBTYPE_STILL_IMAGE_FRAME   0x03
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_UNCOMPRESSED 0x04
#define USB_UVC_VS_DESC_SUBTYPE_FRAME_UNCOMPRESSED  0x05
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_MJPEG        0x06
#define USB_UVC_VS_DESC_SUBTYPE_FRAME_MJPEG         0x07
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_MPEG2TS      0x0a
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_DV           0x0c
#define USB_UVC_VS_DESC_SUBTYPE_COLORFORMAT         0x0d
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_FRAME_BASED  0x10
#define USB_UVC_VS_DESC_SUBTYPE_FRAME_FRAME_BASED   0x11
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_H264         0x13
#define USB_UVC_VS_DESC_SUBTYPE_FRAME_H264          0x14
#define USB_UVC_VS_DESC_SUBTYPE_FORMAT_VP8          0x15
#define USB_UVC_VS_DESC_SUBTYPE_FRAME_VP8           0x16

/* UVC Request Codes (UVC 1.5 Table A-8) */

#define USB_UVC_RC_UNDEFINED                    0x00
#define USB_UVC_SET_CUR                         0x01
#define USB_UVC_SET_CUR_ALL                     0x11
#define USB_UVC_GET_CUR                         0x81
#define USB_UVC_GET_CUR_ALL                     0x91
#define USB_UVC_GET_MIN                         0x82
#define USB_UVC_GET_MIN_ALL                     0x92
#define USB_UVC_GET_MAX                         0x83
#define USB_UVC_GET_MAX_ALL                     0x93
#define USB_UVC_GET_RES                         0x84
#define USB_UVC_GET_RES_ALL                     0x94
#define USB_UVC_GET_LEN                         0x85
#define USB_UVC_GET_LEN_ALL                     0x95
#define USB_UVC_GET_INFO                        0x86
#define USB_UVC_GET_INFO_ALL                    0x96
#define USB_UVC_GET_DEF                         0x87
#define USB_UVC_GET_DEF_ALL                     0x97

/* VideoControl Interface Control Selectors (UVC 1.5 Table A-9) */

#define USB_UVC_VC_CONTROL_UNDEFINED            0x00
#define USB_UVC_VC_VIDEO_POWER_MODE_CONTROL     0x01
#define USB_UVC_VC_REQUEST_ERROR_CODE_CONTROL   0x02

/* Camera Terminal Control Selectors (UVC 1.5 Table A-10) */

#define USB_UVC_CT_CONTROL_UNDEFINED                0x00
#define USB_UVC_CT_SCANNING_MODE_CONTROL            0x01
#define USB_UVC_CT_AE_MODE_CONTROL                  0x02
#define USB_UVC_CT_AE_PRIORITY_CONTROL              0x03
#define USB_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL   0x04
#define USB_UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL   0x05
#define USB_UVC_CT_FOCUS_ABSOLUTE_CONTROL           0x06
#define USB_UVC_CT_FOCUS_RELATIVE_CONTROL           0x07
#define USB_UVC_CT_FOCUS_AUTO_CONTROL               0x08
#define USB_UVC_CT_IRIS_ABSOLUTE_CONTROL            0x09
#define USB_UVC_CT_IRIS_RELATIVE_CONTROL            0x0a
#define USB_UVC_CT_ZOOM_ABSOLUTE_CONTROL            0x0b
#define USB_UVC_CT_ZOOM_RELATIVE_CONTROL            0x0c
#define USB_UVC_CT_PANTILT_ABSOLUTE_CONTROL         0x0d
#define USB_UVC_CT_PANTILT_RELATIVE_CONTROL         0x0e
#define USB_UVC_CT_ROLL_ABSOLUTE_CONTROL            0x0f
#define USB_UVC_CT_ROLL_RELATIVE_CONTROL            0x10
#define USB_UVC_CT_PRIVACY_CONTROL                  0x11

/* Processing Unit Control Selectors (UVC 1.5 Table A-11) */

#define USB_UVC_PU_CONTROL_UNDEFINED                    0x00
#define USB_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL       0x01
#define USB_UVC_PU_BRIGHTNESS_CONTROL                   0x02
#define USB_UVC_PU_CONTRAST_CONTROL                     0x03
#define USB_UVC_PU_GAIN_CONTROL                         0x04
#define USB_UVC_PU_POWER_LINE_FREQUENCY_CONTROL         0x05
#define USB_UVC_PU_HUE_CONTROL                          0x06
#define USB_UVC_PU_SATURATION_CONTROL                   0x07
#define USB_UVC_PU_SHARPNESS_CONTROL                    0x08
#define USB_UVC_PU_GAMMA_CONTROL                        0x09
#define USB_UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL    0x0a
#define USB_UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0b
#define USB_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL     0x0c
#define USB_UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL 0x0d
#define USB_UVC_PU_DIGITAL_MULTIPLIER_CONTROL           0x0e
#define USB_UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL     0x0f
#define USB_UVC_PU_HUE_AUTO_CONTROL                     0x10
#define USB_UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL        0x11
#define USB_UVC_PU_ANALOG_LOCK_STATUS_CONTROL           0x12

/* VideoStreaming Interface Control Selectors (UVC 1.5 Table A-12) */

#define USB_UVC_VS_CONTROL_UNDEFINED                0x00
#define USB_UVC_VS_PROBE_CONTROL                    0x01
#define USB_UVC_VS_COMMIT_CONTROL                   0x02
#define USB_UVC_VS_STILL_PROBE_CONTROL              0x03
#define USB_UVC_VS_STILL_COMMIT_CONTROL             0x04
#define USB_UVC_VS_STILL_IMAGE_TRIGGER_CONTROL      0x05
#define USB_UVC_VS_STREAM_ERROR_CODE_CONTROL        0x06
#define USB_UVC_VS_GENERATE_KEY_FRAME_CONTROL       0x07
#define USB_UVC_VS_UPDATE_FRAME_SEGMENT_CONTROL     0x08
#define USB_UVC_VS_SYNCH_DELAY_CONTROL              0x09

/* Terminal Types (UVC 1.5 Table B-1) */

#define USB_UVC_TT_VENDOR_SPECIFIC                  0x0100
#define USB_UVC_TT_STREAMING                        0x0101

/* Input Terminal Types (UVC 1.5 Table B-2) */

#define USB_UVC_ITT_VENDOR_SPECIFIC                 0x0200
#define USB_UVC_ITT_CAMERA                          0x0201
#define USB_UVC_ITT_MEDIA_TRANSPORT_INPUT           0x0202

/* UVC Payload Header bmHeaderInfo bits */

#define USB_UVC_PAYLOAD_HDR_PTS                     (1 << 0) /* Presentation Time Stamp */
#define USB_UVC_PAYLOAD_HDR_SCR                     (1 << 1) /* Source Clock Reference */
#define USB_UVC_PAYLOAD_HDR_RES                     (1 << 2) /* Reserved */
#define USB_UVC_PAYLOAD_HDR_STI                     (1 << 3) /* Still Image */
#define USB_UVC_PAYLOAD_HDR_ERR                     (1 << 4) /* Error */
#define USB_UVC_PAYLOAD_HDR_EOH                     (1 << 5) /* End of Header */
#define USB_UVC_PAYLOAD_HDR_FID                     (1 << 6) /* Frame ID */
#define USB_UVC_PAYLOAD_HDR_EOF                     (1 << 7) /* End of Frame */

/* Video Probe and Commit Controls hint bits */

#define USB_UVC_PROBE_HINT_FRAME_INTERVAL           (1 << 0)
#define USB_UVC_PROBE_HINT_KEY_FRAME_RATE           (1 << 1)
#define USB_UVC_PROBE_HINT_PFRAME_RATE              (1 << 2)
#define USB_UVC_PROBE_HINT_COMP_QUALITY             (1 << 3)
#define USB_UVC_PROBE_HINT_COMP_WINDOW_SIZE         (1 << 4)

/* UVC 1.1 MJPEG GUID: {32595559-0000-0010-8000-00AA00389B71} */

#define USB_UVC_FORMAT_MJPEG_GUID \
  { 'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00, \
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 }

/* UVC 1.1 Uncompressed YUY2 GUID: {32595559-0000-0010-8000-00AA00389B71} */

#define USB_UVC_FORMAT_YUY2_GUID \
  { 'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00, \
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 }

/* UVC 1.1 Uncompressed NV12 GUID: {3231564E-0000-0010-8000-00AA00389B71} */

#define USB_UVC_FORMAT_NV12_GUID \
  { 'N', 'V', '1', '2', 0x00, 0x00, 0x10, 0x00, \
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Video Control Header Descriptor (UVC 1.5 Table 3-3) */

struct uvc_vc_header_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VC_DESC_SUBTYPE_HEADER */
  uint16_t bcdUVC;                /* UVC specification version */
  uint16_t wTotalLength;          /* Total class-specific descriptor size */
  uint32_t dwClockFrequency;      /* Device clock frequency in Hz */
  uint8_t  bInCollection;         /* Number of streaming interfaces */
  uint8_t  baInterfaceNr[];       /* Interface numbers of streaming IFs */
};

/* Input Terminal Descriptor (UVC 1.5 Table 3-5) */

struct uvc_input_terminal_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VC_DESC_SUBTYPE_INPUT_TERMINAL */
  uint8_t  bTerminalID;
  uint16_t wTerminalType;
  uint8_t  bAssocTerminal;
  uint8_t  iTerminal;
};

/* Camera Terminal Descriptor (UVC 1.5 Table 3-6) */

struct uvc_camera_terminal_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VC_DESC_SUBTYPE_INPUT_TERMINAL */
  uint8_t  bTerminalID;
  uint16_t wTerminalType;         /* USB_UVC_ITT_CAMERA */
  uint8_t  bAssocTerminal;
  uint8_t  iTerminal;
  uint16_t wObjectiveFocalLengthMin;
  uint16_t wObjectiveFocalLengthMax;
  uint16_t wObjectiveFocalLength;
  uint8_t  bControlSize;
  uint8_t  bmControls[];
};

/* Processing Unit Descriptor (UVC 1.5 Table 3-8) */

struct uvc_processing_unit_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VC_DESC_SUBTYPE_PROCESSING_UNIT */
  uint8_t  bUnitID;
  uint8_t  bSourceID;
  uint16_t wMaxMultiplier;
  uint8_t  bControlSize;
  uint8_t  bmControls[];
  /* uint8_t  iProcessing; -- variable offset */
};

/* Video Streaming Input Header Descriptor (UVC 1.5 Table 3-14) */

struct uvc_vs_input_header_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VS_DESC_SUBTYPE_INPUT_HEADER */
  uint8_t  bNumFormats;           /* Number of video payload format descriptors */
  uint16_t wTotalLength;          /* Total class-specific descriptor size */
  uint8_t  bEndpointAddress;      /* Isochronous endpoint address */
  uint8_t  bmInfo;                /* Dynamic format change etc. */
  uint8_t  bTerminalLink;         /* Terminal ID of the output terminal */
  uint8_t  bStillCaptureMethod;
  uint8_t  bTriggerSupport;
  uint8_t  bTriggerUsage;
  uint8_t  bControlSize;
  uint8_t  bmaControls[];
};

/* VS Format Uncompressed Descriptor (UVC 1.5 Table 3-1) */

struct uvc_format_uncompressed_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VS_DESC_SUBTYPE_FORMAT_UNCOMPRESSED */
  uint8_t  bFormatIndex;
  uint8_t  bNumFrameDescriptors;
  uint8_t  guidFormat[16];
  uint8_t  bBitsPerPixel;
  uint8_t  bDefaultFrameIndex;
  uint8_t  bAspectRatioX;
  uint8_t  bAspectRatioY;
  uint8_t  bmInterlaceFlags;
  uint8_t  bCopyProtect;
};

/* VS Frame Uncompressed Descriptor (UVC 1.5 Table 3-2) */

struct uvc_frame_uncompressed_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VS_DESC_SUBTYPE_FRAME_UNCOMPRESSED */
  uint8_t  bFrameIndex;
  uint8_t  bmCapabilities;
  uint16_t wWidth;
  uint16_t wHeight;
  uint32_t dwMinBitRate;
  uint32_t dwMaxBitRate;
  uint32_t dwMaxVideoFrameBufferSize;
  uint32_t dwDefaultFrameInterval;
  uint8_t  bFrameIntervalType;
  uint32_t dwFrameInterval[];     /* If bFrameIntervalType==0: min/max/step; else: discrete */
};

/* VS Format MJPEG Descriptor (UVC 1.5 Table 3-16) */

struct uvc_format_mjpeg_desc_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;       /* USB_UVC_DESC_TYPE_CS_INTERFACE */
  uint8_t  bDescriptorSubtype;    /* USB_UVC_VS_DESC_SUBTYPE_FORMAT_MJPEG */
  uint8_t  bFormatIndex;
  uint8_t  bNumFrameDescriptors;
  uint8_t  bmFlags;
  uint8_t  bDefaultFrameIndex;
  uint8_t  bAspectRatioX;
  uint8_t  bAspectRatioY;
  uint8_t  bmInterlaceFlags;
  uint8_t  bCopyProtect;
};

/* VS Frame MJPEG Descriptor (UVC 1.5 Table 3-17) — same layout as
 * uvc_frame_uncompressed_desc_s
 */

#define uvc_frame_mjpeg_desc_s uvc_frame_uncompressed_desc_s

/* UVC Video Probe and Commit Controls (UVC 1.5 Table 4-47) */

begin_packed_struct struct uvc_probe_commit_s
{
  uint16_t bmHint;
  uint8_t  bFormatIndex;
  uint8_t  bFrameIndex;
  uint32_t dwFrameInterval;
  uint16_t wKeyFrameRate;
  uint16_t wPFrameRate;
  uint16_t wCompQuality;
  uint16_t wCompWindowSize;
  uint16_t wDelay;
  uint32_t dwMaxVideoFrameSize;
  uint32_t dwMaxPayloadTransferSize;
  uint32_t dwClockFrequency;     /* UVC 1.0 only */
  uint8_t  bmFramingInfo;        /* UVC 1.1+ */
  uint8_t  bPreferedVersion;     /* UVC 1.1+ */
  uint8_t  bMinVersion;          /* UVC 1.1+ */
  uint8_t  bMaxVersion;          /* UVC 1.1+ */
} end_packed_struct;

/* UVC Payload Header (UVC 1.5 Figure 2-2) — minimal header */

struct uvc_payload_header_s
{
  uint8_t  bHeaderLength;
  uint8_t  bmHeaderInfo;
  /* Optional fields follow: dwPresentationTime, scrSourceClock, etc. */
};

#endif /* __INCLUDE_NUTTX_USB_UVC_H */
