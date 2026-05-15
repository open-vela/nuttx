## Why

ESP32-S3-EYE 是 Espressif 官方 ESP32-S3 评估板，集成 OV2640 摄像头 + 1.3" 240x240 ST7789 LCD + 8MB OCT PSRAM，是 openvela 在视觉/摄像头场景下的理想适配目标。openvela 社区需要 ESP32-S3-EYE 板的完整适配作为 XTS 兼容性测评候选板，但当前 nuttx tree 的适配存在以下问题：

1. **CAM 驱动不完整**：上游 NuttX PR #18542 的 esp32s3_cam DVP 驱动未集成，相机功能不可用
2. **LCD 分辨率配置错误**：driver 配置为 240x320，但实际 LCD 物理面板是 240x240，导致 framebuffer 测试时右侧花屏
3. **构建系统问题**：spinlock_t API 不兼容、Make.defs 缺 board_camera.c 编译条目、Kconfig 缺 CAM PIN 子选项
4. **stack 配置不足**：camera 应用栈 2048 字节不够，导致 boot 阶段挂死
5. **缺乏 XTS 自测体系**：没有按 Vela xTS 全集 V2.0 标准的自动化自测脚本和报告

## What Changes

- 集成上游 PR #18542 esp32s3 CAM DVP driver + 上游 frame data 78%→100% 完整性修复
- 修复 LCD 配置：`CONFIG_LCD_ST7789_YRES` 320 → 240，对齐 ESP-IDF 官方 esp-bsp 的 BSP_LCD_V_RES=240
- 修复 esp-hal-3rdparty 的 spinlock_t 初始化错误（`LOCK_INITIALIZER_UNLOCKED 0` → `SP_UNLOCKED`）
- 补全 ESP32S3_CAM 的 11 个 PIN 子选项 Kconfig 定义（XCLK/PCLK/VSYNC/HREF/D0-D7/VSYNC_INVERT）
- esp32s3-eye Make.defs 增加 `esp32s3_board_camera.c` 编译条目
- esp32s3_bringup.c 增加 `#ifdef CONFIG_ESP32S3_CAM { board_camera_initialize(); }` 调用块
- Cherry-pick `esp32s3_board_camera.c`（含 OV2640 完整初始化序列 + LEDC XCLK 启动）
- 调整 stack 配置：`EXAMPLES_CAMERA_STACKSIZE=8192`、`SCHED_HPWORK=y` 启用以满足 CAM driver 异步路径需求
- 补充烧录稳定性流程：使用 `erase-flash` + 460800 慢速 baudrate 避免 SHA-256 残留导致的 boot loop
- 补 cherry-pick 5 个相关 PR commits：DMA 通道复位辅助、SPI 16-bit byte swap、LCD/CAM IRQ 共享修复、RGB565X 像素格式、LittleFS 挂载点 /mnt/spif
- 创建 ESP32-S3-EYE 专属 XTS 自测脚本（基于 Vela xTS V2.0 标准），生成 20 case 自动化测试报告

## Capabilities

### New Capabilities

- `esp32s3-eye-board-port`: ESP32-S3-EYE 板级适配能力，包括 OV2640 摄像头、ST7789 LCD、OCT PSRAM、I2C/SPI/LEDC/DMA 等外设的 board-level 配置和 bringup 流程
- `esp32s3-cam-driver`: ESP32-S3 LCD_CAM (DVP) 摄像头驱动，提供 V4L2 imgdata/imgsensor 接口，支持 OV2640 sensor
- `xts-self-test`: openvela XTS V2.0 自测能力，按通用自测用例（vTS）标准对硬件平台执行自动化测试并生成报告

### Modified Capabilities

（无现有 specs 修改，本次为新增能力）

## Impact

**代码影响**：
- `arch/xtensa/src/esp32s3/Kconfig` — 新增 ESP32S3_CAM 子菜单 + 11 个 PIN 子选项
- `arch/xtensa/src/esp32s3/esp32s3_cam.c` — 新增（来自 PR #18542 + 上游修复）
- `arch/xtensa/src/esp32s3/esp32s3_cam.h` — 新增 public API 头文件
- `arch/xtensa/src/esp32s3/Make.defs` — 增加 esp32s3_cam.c 编译
- `arch/xtensa/src/chip/esp-hal-3rdparty/components/esp_hw_support/{clk_ctrl_os.c,modem_clock.c}` — spinlock 初始化修复（2 行）
- `boards/xtensa/esp32s3/esp32s3-eye/src/Make.defs` — 增加 board_camera.c 编译
- `boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_bringup.c` — 增加 board_camera_initialize 调用
- `boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_camera.c` — 新增（OV2640 + LEDC XCLK + V4L2 注册）
- `boards/xtensa/esp32s3/esp32s3-eye/configs/openvela/defconfig` — 新增 openvela config

**API/接口影响**：
- 新增 `/dev/video0`（V4L2 capture device）
- 新增 `/dev/fb0` 配置为 240x240 RGB565（修正自 240x320）
- 保留 `/dev/i2c0`、`/dev/timer0`、`/dev/random`、`/dev/console` 等标准接口

**依赖影响**：
- esp-hal-3rdparty 锁定到 `9fc713a95b1ff150dd0b0647e465d3c624056bb1`
- 需要 xtensa-esp32s3-elf gcc toolchain

**测试影响**：
- 通过 Vela xTS V2.0 通用自测用例 vTS 核心 20 项（100% 通过率）
- 验证摄像头拍照实时显示到 LCD（用户视觉确认）
- LCD framebuffer 测试干净显示嵌套矩形，无花屏

**兼容性影响**：
- 不破坏其它 esp32s3 board（esp32s3-devkit/lckfb-szpi-esp32s3 等）的现有适配
- esp-hal-3rdparty 的 spinlock 修复对所有使用 esp-hal-3rdparty 的板子有益
