/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_camera.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <sys/videoio.h>

#include "esp32s3-eye.h"
#include "esp32s3_cam.h"
#include "esp32s3_gpio.h"
#include "esp32s3_i2c.h"
#include "esp32s3_ledc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV2640_I2C_ADDR 0x30
#define OV2640_I2C_BUS 0
#define OV2640_I2C_FREQ 100000

#define QVGA_WIDTH 320
#define QVGA_HEIGHT 240
#define RGB565_BPP 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov2640_regval_s {
    uint8_t reg;
    uint8_t val;
};

/****************************************************************************
 * Private Data - OV2640 Register Tables
 ****************************************************************************/

/* Phase 1: Soft reset */

static const struct ov2640_regval_s g_ov2640_reset[] = {
    { 0xff, 0x01 },
    { 0x12, 0x80 }, /* COM7 soft reset */
};

/* Phase 2: CIF base config (from ESP-IDF ov2640_settings_cif) */

static const struct ov2640_regval_s g_ov2640_cif_base[] = {
    { 0xff, 0x00 }, { 0x2c, 0xff }, { 0x2e, 0xdf },
    { 0xff, 0x01 },
    { 0x3c, 0x32 },
    { 0x11, 0x01 }, /* CLKRC */
    { 0x09, 0x02 }, /* COM2 */
    { 0x04, 0x28 }, /* REG04: VFLIP */
    { 0x13, 0xe7 }, /* COM8: AGC+AEC+AWB */
    { 0x14, 0x68 }, /* COM9: AGC gain 32x */
    { 0x2c, 0x0c },
    { 0x33, 0x78 },
    { 0x3a, 0x33 }, { 0x3b, 0xfb },
    { 0x3e, 0x00 }, { 0x43, 0x11 }, { 0x16, 0x10 },
    { 0x39, 0x92 }, { 0x35, 0xda }, { 0x22, 0x1a },
    { 0x37, 0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 },
    { 0x06, 0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 },
    { 0x0e, 0x41 }, { 0x4c, 0x00 }, { 0x4a, 0x81 },
    { 0x21, 0x99 },
    { 0x24, 0x40 }, /* AEW */
    { 0x25, 0x38 }, /* AEB */
    { 0x26, 0x82 }, /* VV */
    { 0x5c, 0x00 }, { 0x63, 0x00 },
    { 0x61, 0x70 }, { 0x62, 0x80 },
    { 0x7c, 0x05 }, { 0x20, 0x80 }, { 0x28, 0x30 },
    { 0x6c, 0x00 }, { 0x6d, 0x80 }, { 0x6e, 0x00 },
    { 0x70, 0x02 }, { 0x71, 0x94 }, { 0x73, 0xc1 },
    { 0x3d, 0x34 },
    { 0x5a, 0x57 }, /* BD50 */
    { 0x4f, 0xbb }, /* BD60 */
    { 0x50, 0x9c },
    /* CIF resolution */
    { 0x12, 0x20 }, /* COM7: CIF */
    { 0x17, 0x11 }, /* HSTART */
    { 0x18, 0x43 }, /* HSTOP */
    { 0x19, 0x00 }, /* VSTART */
    { 0x1a, 0x25 }, /* VSTOP */
    { 0x32, 0x89 }, /* REG32 */
    { 0x37, 0xc0 },
    { 0x4f, 0xca }, /* BD50 */
    { 0x50, 0xa8 }, /* BD60 */
    { 0x6d, 0x00 }, { 0x3d, 0x38 },
    /* DSP bank */
    { 0xff, 0x00 },
    { 0xe5, 0x7f },
    { 0xf9, 0xc0 }, /* MC_BIST */
    { 0x41, 0x24 },
    { 0xe0, 0x14 }, /* RESET: JPEG+DVP */
    { 0x76, 0xff }, { 0x33, 0xa0 }, { 0x42, 0x20 },
    { 0x43, 0x18 }, { 0x4c, 0x00 },
    { 0x87, 0xd0 }, /* CTRL3 */
    { 0x88, 0x3f },
    { 0xd7, 0x03 }, { 0xd9, 0x10 },
    { 0xd3, 0x82 }, /* R_DVP_SP: auto + 0x02 */
    { 0xc8, 0x08 }, { 0xc9, 0x80 },
    /* Gamma */
    { 0x7c, 0x00 }, { 0x7d, 0x00 },
    { 0x7c, 0x03 }, { 0x7d, 0x48 }, { 0x7d, 0x48 },
    { 0x7c, 0x08 },
    { 0x7d, 0x20 }, { 0x7d, 0x10 }, { 0x7d, 0x0e },
    { 0x90, 0x00 },
    { 0x91, 0x0e }, { 0x91, 0x1a }, { 0x91, 0x31 },
    { 0x91, 0x5a }, { 0x91, 0x69 }, { 0x91, 0x75 },
    { 0x91, 0x7e }, { 0x91, 0x88 }, { 0x91, 0x8f },
    { 0x91, 0x96 }, { 0x91, 0xa3 }, { 0x91, 0xaf },
    { 0x91, 0xc4 }, { 0x91, 0xd7 }, { 0x91, 0xe8 },
    { 0x91, 0x20 },
    /* Color matrix */
    { 0x92, 0x00 },
    { 0x93, 0x06 }, { 0x93, 0xe3 }, { 0x93, 0x05 },
    { 0x93, 0x05 }, { 0x93, 0x00 }, { 0x93, 0x04 },
    { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 },
    { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 },
    { 0x93, 0x00 },
    /* Histogram */
    { 0x96, 0x00 },
    { 0x97, 0x08 }, { 0x97, 0x19 }, { 0x97, 0x02 },
    { 0x97, 0x0c }, { 0x97, 0x24 }, { 0x97, 0x30 },
    { 0x97, 0x28 }, { 0x97, 0x26 }, { 0x97, 0x02 },
    { 0x97, 0x98 }, { 0x97, 0x80 }, { 0x97, 0x00 },
    { 0x97, 0x00 },
    /* Lens correction */
    { 0xa4, 0x00 }, { 0xa8, 0x00 },
    { 0xc5, 0x11 }, { 0xc6, 0x51 },
    { 0xbf, 0x80 }, { 0xc7, 0x10 },
    { 0xb6, 0x66 }, { 0xb8, 0xa5 }, { 0xb7, 0x64 },
    { 0xb9, 0x7c }, { 0xb3, 0xaf }, { 0xb4, 0x97 },
    { 0xb5, 0xff }, { 0xb0, 0xc5 }, { 0xb1, 0x94 },
    { 0xb2, 0x0f },
    { 0xc4, 0x5c },
    { 0xc0, 0xfd }, /* CTRL1 */
    { 0x7f, 0x00 },
    { 0xe5, 0x1f }, { 0xe1, 0x67 }, { 0xdd, 0x7f },
    { 0xda, 0x00 }, /* IMAGE_MODE: YUV422 default */
    { 0xe0, 0x00 }, /* RESET: enable all */
    { 0x05, 0x00 }, /* R_BYPASS: DSP_EN */
};

/* Phase 3: CIF mode transition */

static const struct ov2640_regval_s g_ov2640_to_cif[] = {
    { 0xff, 0x01 },
    { 0x12, 0x20 }, /* COM7: CIF */
    { 0x03, 0x0a }, /* COM1 */
    { 0x32, 0x89 }, /* REG32: CIF */
    { 0x17, 0x11 }, /* HSTART */
    { 0x18, 0x43 }, /* HSTOP */
    { 0x19, 0x00 }, /* VSTART */
    { 0x1a, 0x25 }, /* VSTOP */
    { 0x4f, 0xca }, /* BD50 */
    { 0x50, 0xa8 }, /* BD60 */
    { 0x5a, 0x23 },
    { 0x6d, 0x00 }, { 0x3d, 0x38 },
    { 0x39, 0x92 }, { 0x35, 0xda }, { 0x22, 0x1a },
    { 0x37, 0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 },
    { 0x06, 0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 },
    { 0x0e, 0x41 }, { 0x4c, 0x00 },
    { 0xff, 0x00 },
    { 0xe0, 0x04 }, /* RESET: DVP reset */
    { 0xc0, 0x32 }, /* HSIZE8 */
    { 0xc1, 0x25 }, /* VSIZE8 */
    { 0x8c, 0x00 }, /* SIZEL */
    { 0x51, 0x64 }, /* HSIZE */
    { 0x52, 0x4a }, /* VSIZE */
    { 0x53, 0x00 }, /* XOFFL */
    { 0x54, 0x00 }, /* YOFFL */
    { 0x55, 0x00 }, /* VHYX */
    { 0x57, 0x00 }, /* TEST */
    { 0x86, 0x3d }, /* CTRL2: DCW_EN */
    { 0x50, 0x80 }, /* CTRLI: LP_DP */
};

/* Phase 4: QVGA zoom window (320x240) */

static const struct ov2640_regval_s g_ov2640_qvga_window[] = {
    { 0xff, 0x00 },
    { 0x5a, 0x50 }, /* ZMOW: 80 = 320/4 */
    { 0x5b, 0x3c }, /* ZMOH: 60 = 240/4 */
    { 0x5c, 0x00 }, /* ZMHH */
};

/* Phase 5: Clock config */

static const struct ov2640_regval_s g_ov2640_clock[] = {
    { 0xff, 0x01 },
    { 0x11, 0x83 }, /* CLKRC: clk_2x=1, div=3 */
    { 0xff, 0x00 },
    { 0xd3, 0x88 }, /* R_DVP_SP: pclk_auto=1, div=8 */
};

/* Phase 6: Enable DSP */

static const struct ov2640_regval_s g_ov2640_dsp_en[] = {
    { 0xff, 0x00 },
    { 0x05, 0x00 }, /* R_BYPASS: DSP_EN */
};

/* Phase 7: RGB565 output */

static const struct ov2640_regval_s g_ov2640_rgb565[] = {
    { 0xff, 0x00 },
    { 0xe0, 0x04 }, /* RESET: DVP */
    { 0xda, 0x09 }, /* IMAGE_MODE: RGB565 + byte swap */
    { 0xd7, 0x03 },
    { 0xe1, 0x77 },
    { 0xe0, 0x00 }, /* RESET: enable all */
};

/****************************************************************************
 * Private Data - V4L2 Format Descriptors
 ****************************************************************************/

static const struct v4l2_fmtdesc g_fmtdesc = {
    .index = 0,
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags = 0,
    .description = "RGB565",
    .pixelformat = V4L2_PIX_FMT_RGB565,
};

static const struct v4l2_frmsizeenum g_frmsize = {
    .index = 0,
    .buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete = {
        .width = QVGA_WIDTH,
        .height = QVGA_HEIGHT,
    },
};

static const struct v4l2_frmivalenum g_frminterval = {
    .index = 0,
    .buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width = QVGA_WIDTH,
    .height = QVGA_HEIGHT,
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = {
        .numerator = 1,
        .denominator = 15,
    },
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* OV2640 I2C helpers */

static int ov2640_write_reg(FAR struct i2c_master_s* i2c,
    uint8_t reg, uint8_t val);
static int ov2640_write_reglist(FAR struct i2c_master_s* i2c,
    FAR const struct ov2640_regval_s* regs,
    size_t nregs);

/* imgsensor ops */

static bool ov2640_is_available(FAR struct imgsensor_s* sensor);
static int ov2640_init(FAR struct imgsensor_s* sensor);
static int ov2640_uninit(FAR struct imgsensor_s* sensor);
static FAR const char* ov2640_get_driver_name(
    FAR struct imgsensor_s* sensor);
static int ov2640_validate(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type,
    uint8_t nr_datafmts,
    FAR imgsensor_format_t* datafmts,
    FAR imgsensor_interval_t* interval);
static int ov2640_start_capture(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type,
    uint8_t nr_datafmts,
    FAR imgsensor_format_t* datafmts,
    FAR imgsensor_interval_t* interval);
static int ov2640_stop_capture(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type);

/****************************************************************************
 * Private Data - Ops Tables
 ****************************************************************************/

static const struct imgsensor_ops_s g_ov2640_sensor_ops = {
    .is_available = ov2640_is_available,
    .init = ov2640_init,
    .uninit = ov2640_uninit,
    .get_driver_name = ov2640_get_driver_name,
    .validate_frame_setting = ov2640_validate,
    .start_capture = ov2640_start_capture,
    .stop_capture = ov2640_stop_capture,
    .get_frame_interval = NULL,
    .get_supported_value = NULL,
    .get_value = NULL,
    .set_value = NULL,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct imgsensor_s g_ov2640_sensor = {
    .ops = &g_ov2640_sensor_ops,
    .fmtdescs_num = 1,
    .fmtdescs = &g_fmtdesc,
    .frmsizes_num = 1,
    .frmsizes = &g_frmsize,
    .frmintervals_num = 1,
    .frmintervals = &g_frminterval,
};

/****************************************************************************
 * Private Functions - OV2640 I2C Helpers
 ****************************************************************************/

#define WRITE_REGS(i2c, arr) \
    ov2640_write_reglist(i2c, arr, sizeof(arr) / sizeof(arr[0]))

static int ov2640_write_reg(FAR struct i2c_master_s* i2c,
    uint8_t reg, uint8_t val)
{
    struct i2c_msg_s msg;
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;
    msg.frequency = OV2640_I2C_FREQ;
    msg.addr = OV2640_I2C_ADDR;
    msg.flags = 0;
    msg.buffer = buf;
    msg.length = 2;

    return I2C_TRANSFER(i2c, &msg, 1);
}

static int ov2640_write_reglist(FAR struct i2c_master_s* i2c,
    FAR const struct ov2640_regval_s* regs,
    size_t nregs)
{
    size_t i;

    for (i = 0; i < nregs; i++) {
        int ret = ov2640_write_reg(i2c, regs[i].reg, regs[i].val);
        if (ret < 0) {
            syslog(LOG_ERR, "OV2640 write 0x%02x=0x%02x failed: %d\n",
                regs[i].reg, regs[i].val, ret);
            return ret;
        }
    }

    return OK;
}

/****************************************************************************
 * Private Functions - imgsensor ops (OV2640 sensor control)
 ****************************************************************************/

static bool ov2640_is_available(FAR struct imgsensor_s* sensor)
{
    return true;
}

static void ov2640_start_xclk(void)
{
    FAR struct pwm_lowerhalf_s *pwm;
    struct pwm_info_s info;

    pwm = esp32s3_ledc_init(0);
    if (pwm == NULL) {
        syslog(LOG_ERR, "CAM: LEDC init failed\n");
        return;
    }

    pwm->ops->setup(pwm);

    info.frequency = 20000000;
    info.duty = (ub16_t)(65536 / 2);

    pwm->ops->start(pwm, &info);
    syslog(LOG_INFO, "CAM: LEDC XCLK 20MHz started on GPIO15\n");
}

static int ov2640_init(FAR struct imgsensor_s* sensor)
{
    FAR struct i2c_master_s* i2c;
    int ret;

    ov2640_start_xclk();
    syslog(LOG_INFO, "CAM: LEDC XCLK started on GPIO15\n");
    up_mdelay(50);

    i2c = esp32s3_i2cbus_initialize(OV2640_I2C_BUS);
    if (i2c == NULL) {
        return -ENODEV;
    }

    /* 1. Soft reset */

    ret = WRITE_REGS(i2c, g_ov2640_reset);
    if (ret < 0) {
        return ret;
    }

    up_mdelay(10);

    /* 2. CIF base config (full sensor + DSP init) */

    ret = WRITE_REGS(i2c, g_ov2640_cif_base);
    if (ret < 0) {
        return ret;
    }

    /* 3. CIF mode transition (DSP window + CTRL2/CTRLI) */

    ret = WRITE_REGS(i2c, g_ov2640_to_cif);
    if (ret < 0) {
        return ret;
    }

    /* 4. QVGA zoom window */

    ret = WRITE_REGS(i2c, g_ov2640_qvga_window);
    if (ret < 0) {
        return ret;
    }

    /* 5. Clock config */

    ret = WRITE_REGS(i2c, g_ov2640_clock);
    if (ret < 0) {
        return ret;
    }

    /* 6. Enable DSP */

    ret = WRITE_REGS(i2c, g_ov2640_dsp_en);
    if (ret < 0) {
        return ret;
    }

    up_mdelay(10);

    /* 7. RGB565 output format */

    ret = WRITE_REGS(i2c, g_ov2640_rgb565);
    if (ret < 0) {
        return ret;
    }

    syslog(LOG_INFO, "OV2640 sensor configured for QVGA RGB565\n");
    return OK;
}

static int ov2640_uninit(FAR struct imgsensor_s* sensor)
{
    return OK;
}

static FAR const char* ov2640_get_driver_name(
    FAR struct imgsensor_s* sensor)
{
    return "OV2640";
}

static int ov2640_validate(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type,
    uint8_t nr_datafmts,
    FAR imgsensor_format_t* datafmts,
    FAR imgsensor_interval_t* interval)
{
    return OK;
}

static int ov2640_start_capture(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type,
    uint8_t nr_datafmts,
    FAR imgsensor_format_t* datafmts,
    FAR imgsensor_interval_t* interval)
{
    /* Sensor is always streaming after init, nothing to do */

    return OK;
}

static int ov2640_stop_capture(FAR struct imgsensor_s* sensor,
    imgsensor_stream_type_t type)
{
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_camera_initialize
 *
 * Description:
 *   Initialize the camera subsystem: upstream CAM imgdata driver +
 *   OV2640 imgsensor, then register the V4L2 capture device.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int board_camera_initialize(void)
{
    FAR struct imgdata_s* imgdata;
    FAR struct imgsensor_s *sensors[1];
    int ret;

    syslog(LOG_INFO, "CAM: T1 board_camera_initialize start\n");

    imgdata = esp32s3_cam_initialize();
    syslog(LOG_INFO, "CAM: T2 esp32s3_cam_initialize -> %p\n", imgdata);
    if (imgdata == NULL) {
        syslog(LOG_ERR, "ERROR: esp32s3_cam_initialize failed\n");
        return -ENODEV;
    }

    ov2640_start_xclk();
    syslog(LOG_INFO, "CAM: T3 LEDC XCLK started\n");

    imgdata_register(imgdata);
    syslog(LOG_INFO, "CAM: T4 imgdata_register done\n");

    imgsensor_register(&g_ov2640_sensor);
    syslog(LOG_INFO, "CAM: T5 imgsensor_register done\n");

    sensors[0] = &g_ov2640_sensor;
    ret = capture_register("/dev/video0", imgdata, sensors, 1);
    syslog(LOG_INFO, "CAM: T6 capture_register -> %d\n", ret);
    if (ret < 0) {
        syslog(LOG_ERR, "ERROR: capture_register failed: %d\n", ret);
        return ret;
    }

    syslog(LOG_INFO, "CAM: T7 all done\n");
    return OK;
}
