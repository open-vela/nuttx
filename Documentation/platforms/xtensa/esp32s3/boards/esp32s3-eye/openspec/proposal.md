## Why

ESP32-S3-EYE 是 Espressif 官方 ESP32-S3 评估板（ESP32-S3-WROOM-1-N4 + 8MB OCT PSRAM），集成 OV2640 摄像头、1.3" 240x240 ST7789 LCD、I2S MEMS 麦克风、MicroSD 卡槽、WiFi、Bluetooth LE 等丰富外设，是 openvela 在视觉/AI 场景下的理想适配候选板。openvela 社区需要 ESP32-S3-EYE 板的**完整外设适配**作为 XTS 兼容性测评候选板，但当前 openvela trunk 存在以下空白和缺陷：

1. **CAM 驱动未集成**：上游 NuttX PR #18542 的 esp32s3_cam DVP 驱动未集成，相机功能不可用
2. **LCD 配置错误**：driver 配置 240x320，但实际物理面板是 240x240，fb 测试右侧花屏
3. **构建系统缺陷**：spinlock_t API 不兼容、Make.defs 缺 board_camera.c 编译条目、Kconfig 缺 CAM PIN 子选项
4. **stack 配置不足**：camera 应用栈 2048 字节不够导致 boot hang
5. **WiFi 已 work 但缺验证**：vendor defconfig 启用了 WiFi 但没有 XTS 测试覆盖
6. **SDMMC 未启用**：MicroSD 卡未适配，缺 board init + GPIO config + 1-bit 模式
7. **I2S 麦克风未启用**：MEMS 麦克风未适配，缺 I2S0 RX 配置 + audio 注册
8. **BLE 完全无法启用**：NuttX `ESP32S3_BLE` Kconfig 没 select `ESPRESSIF_BLE`，导致 esp-hal-3rdparty sdkconfig.h 中 BT 段不生效；esp32s3_ble_adapter.c 中 `sched_lock()` 返回值用法不兼容 NuttX SMP API
9. **缺乏 XTS 自测体系**：没有按 Vela xTS 全集 V2.0 标准的自动化自测脚本和报告

## What Changes

### Phase 1（基础适配 — Camera + LCD + Build 修复）

- 集成上游 PR #18542 esp32s3 CAM DVP driver + 上游 frame data 78%→100% 完整性修复
- 修复 LCD 配置：`CONFIG_LCD_ST7789_YRES` 320 → 240（对齐 ESP-IDF 官方 esp-bsp 的 BSP_LCD_V_RES=240）
- 修复 esp-hal-3rdparty spinlock_t 初始化（`LOCK_INITIALIZER_UNLOCKED 0 → SP_UNLOCKED`）
- 补全 ESP32S3_CAM 的 11 个 PIN 子选项 Kconfig 定义
- esp32s3-eye Make.defs + bringup.c 集成 CAM
- 调整 stack：`EXAMPLES_CAMERA_STACKSIZE=8192`、`SCHED_HPWORK=y`
- 烧录稳定性流程：`erase-flash` + 460800 慢速 baudrate
- 补 cherry-pick 5 个 PR commits（DMA helper / SPI byteswap / IRQ fix / RGB565X / LittleFS mount）

### Phase 2（全外设扩展 — WiFi + SDMMC + I2S + BLE）

- **WiFi**: vendor defconfig 已启用，扫到 15 个真实 AP
- **SDMMC**: 启用 `CONFIG_ESP32S3_SDMMC=y` + 1-bit MMC 模式（CLK=39, CMD=38, D0=40）+ `CONFIG_SDIO_WIDTH_D1_ONLY=y` + FAT 文件系统
- **I2S MEMS 麦克风**: 启用 `CONFIG_ESP32S3_I2S0_RX=y` + `CONFIG_AUDIO_I2S=y`，BCLK=GPIO41 / WS=GPIO42 / DIN=GPIO2，bringup 调 `board_i2sdev_initialize` 注册 `/dev/audio/pcm_in0`
- **Bluetooth LE**: 修复两个上游缺陷：
  - **Kconfig 修复**：`config ESP32S3_BLE` 增加 `select ESPRESSIF_BLE` + 新增 hidden `config ESPRESSIF_BLE` 触发 sdkconfig.h 的 BT 段
  - **API 修复**：esp32s3_ble_adapter.c 的 `ret = sched_lock()` / `sched_unlock()` 改为不带返回值
  - 启用 `DRIVERS_BLUETOOTH=y` + `NET_BLUETOOTH=y` + `WIRELESS_BLUETOOTH=y` + `BTSAK=y` + `ALLOW_BSD_COMPONENTS=y`

### Phase 3（XTS 测试 + 文档）

- ESP32-S3-EYE 专属 XTS 自测脚本（基于 Vela xTS V2.0）
- V1: 20 case（系统内核 + 系统应用 + 驱动 BSP + EYE 基础）
- V3: **26 case 全 PASS**（V1 + 6 新外设：audio、mmcsd1、mount、wifi scan/ifconfig、ble info/scan）
- 真实信号验证：WiFi 15 AP / BLE 5 设备（含 Apple+小米品牌广播）/ SD 卡 31GB 数据
- 测试报告发布飞书云文档

## Capabilities

### New Capabilities

- `esp32s3-eye-board-port`: ESP32-S3-EYE 板级适配能力，覆盖 14 个硬件外设（CAM/LCD/PSRAM/Flash/SDMMC/I2S 麦克风/I2C/SPI/LEDC/DMA/Timer/RNG/USB-JTAG/IRQ）+ WiFi/BLE 无线
- `esp32s3-cam-driver`: ESP32-S3 LCD_CAM (DVP) 摄像头驱动 + V4L2 imgdata/imgsensor 接口 + OV2640 sensor
- `esp32s3-ble-support`: ESP32-S3 Bluetooth LE 适配，包括 NuttX Kconfig 修复（ESP32S3_BLE → ESPRESSIF_BLE select）+ esp-hal-3rdparty BT controller + NimBLE host stack 集成 + sched_lock API 修复
- `xts-self-test`: openvela XTS V2.0 自测能力（26 case 自动化跑测 + 报告生成 + 飞书发布）

### Modified Capabilities

无（本次为全新增能力）

## Impact

### nuttx tree 内修改文件

- `arch/xtensa/src/esp32s3/Kconfig` — 新增 ESP32S3_CAM 11 PIN 子选项 + ESPRESSIF_BLE hidden symbol + ESP32S3_BLE select
- `arch/xtensa/src/esp32s3/esp32s3_cam.c` — 新增 (PR #18542)
- `arch/xtensa/src/esp32s3/esp32s3_cam.h` — 新增 public API
- `arch/xtensa/src/esp32s3/Make.defs` — 增加 esp32s3_cam.c 编译
- `arch/xtensa/src/esp32s3/esp32s3_ble_adapter.c` — sched_lock/unlock 返回值修复
- `boards/.../esp32s3-eye/src/Make.defs` — 增加 board_camera.c 编译
- `boards/.../esp32s3-eye/src/esp32s3_bringup.c` — board_camera_initialize + board_i2sdev_initialize 调用
- `boards/.../esp32s3-eye/src/esp32s3-eye.h` — board_i2sdev_initialize prototype
- `boards/.../esp32s3-eye/src/esp32s3_board_camera.c` — 新增 (OV2640 + LEDC XCLK + V4L2 注册)
- `boards/.../esp32s3-eye/configs/openvela/defconfig` — 新增（全外设 config）
- `boards/.../esp32s3-eye/scripts/patches/0001-esp-hal-3rdparty-fix-spinlock-init.patch` — 新增
- `Documentation/.../esp32s3-eye/index.rst` — openvela section
- `Documentation/.../esp32s3-eye/openspec/` — OpenSpec 完整文档

### esp-hal-3rdparty 修改（patch 形式）

- `components/esp_hw_support/clk_ctrl_os.c` — spinlock 修复
- `components/esp_hw_support/modem_clock.c` — spinlock 修复

### API/接口影响

**新增设备节点**：
- `/dev/video0` — V4L2 capture device（OV2640）
- `/dev/fb0` — 240x240 RGB565 framebuffer（ST7789）
- `/dev/audio/pcm_in0` — I2S MEMS 麦克风音频输入
- `/dev/mmcsd1` — MicroSD 卡块设备
- `/dev/bnep0` — BLE 网络接口

**新增 NSH commands**：
- `camera <num>` — 拍照
- `fb` — framebuffer 测试
- `wapi <ifname> <cmd>` — WiFi 工具
- `bt <ifname> <cmd>` — BLE 工具

### 依赖影响

- esp-hal-3rdparty 锁定到 `9fc713a95b1ff150dd0b0647e465d3c624056bb1`
- xtensa-esp32s3-elf gcc toolchain
- esptool.py

### 测试影响

- **Vela xTS V2.0 + EYE 全外设扩展 26 case，100% PASS**
- WiFi 真实环境扫描验证（15 AP）
- BLE 真实环境扫描验证（5 设备含 Apple/小米 vendor ID）
- SD 卡 31GB 数据读取验证

### 兼容性影响

- **不破坏其它 esp32s3 board**（esp32s3-devkit / lckfb-szpi-esp32s3 / esp32s3-korvo-2 等）
- **ESPRESSIF_BLE Kconfig 修复对其它 ESP32-S3 板有益**（如 esp32s3-devkit:blewifi 同样受益）
- **esp-hal-3rdparty spinlock 修复对所有使用该库的 ESP32 板有益**
- **sched_lock() 返回值修复**符合 NuttX 上游 API 现状

### 已知未适配项

- **QMA7981 加速度计**：硬件存在，I2C bus 0 地址 0x12 已扫到，但 NuttX 没有 QMA7981 driver（需要新写 sensor driver，超出本 change 范围）
