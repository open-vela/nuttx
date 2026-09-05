## Context

### 当前状态

openvela 是基于 NuttX 的实时操作系统，由小米主导维护。openvela 社区已为多个 ESP32 板子（esp32s3-devkit / lckfb-szpi-esp32s3 / esp32s3-korvo-2 等）提供官方适配，但 ESP32-S3-EYE 板存在以下不完整：

1. **CAM driver 未集成**：上游 NuttX PR #18542 虽已 cherry-pick 到 yanxingyu17/nuttx 的 `pr/esp32s3-eye-camera` 分支，但相关 board-level 集成（Make.defs / bringup / Kconfig）不完整
2. **LCD driver 配置不正确**：`CONFIG_LCD_ST7789_YRES=320` 与实际 240x240 物理面板不匹配
3. **构建系统多处缺陷**：spinlock_t API 不兼容、Kconfig 缺 PIN 子选项、Make.defs 缺 board_camera.c
4. **WiFi/BLE/SDMMC/I2S 等外设全部未启用**

### 硬件参考

ESP32-S3-EYE 官方硬件特性（来自 [Espressif esp-bsp](https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_s3_eye) 权威配置）：

| 子系统 | 规格 |
|-------|------|
| MCU | ESP32-S3-WROOM-1 (8MB Flash + 8MB OCT PSRAM) |
| LCD | 1.3" **240x240** ST7789, SPI3, RGB565, 80MHz pclk, color invert |
| Camera | OV2640 DVP 8-bit, XCLK from LEDC GPIO15 |
| MEMS Microphone | I2S0 标准 TDM 模式 (BCLK=GPIO41, WS=GPIO42, DIN=GPIO2) |
| MicroSD Card | SDMMC 1-bit 模式 (CLK=GPIO39, CMD=GPIO38, D0=GPIO40) |
| Accelerometer | QMA7981 三轴加速度计 (I2C0 addr 0x12) |
| WiFi | 802.11 b/g/n via on-chip ESP32-S3 controller |
| BLE | Bluetooth 5.0 LE via on-chip controller + NimBLE host |
| LCD GPIO | PCLK=21, MOSI=47, DC=43, CS=44, BL=48 |
| Camera GPIO | XCLK=15, PCLK=13, VSYNC=6, HREF=7, D0-D7=11/9/8/10/12/18/17/16 |
| I2C SCCB | SCL=5, SDA=4 |

### 约束

- **不能破坏其它板子**：esp32s3-devkit / esp32s3-korvo-2 / esp32s3-box 等共享 common 代码
- **保留 openvela 上游兼容性**：所有改动应可 upstream 回 openvela 主线
- **遵循 NuttX 配置惯例**：Kconfig + defconfig + Make.defs 标准结构
- **复用上游 PR**：尽量基于已有 PR #18542 等社区贡献，不重复造轮子
- **GPIO 引脚不冲突**：SD D1=GPIO16 与 CAM_D7=GPIO16 冲突 → 必须用 1-bit 模式

### 利益相关方

- **openvela 兼容性认证组**：需要 EYE 板作为新认证候选板
- **ESP32-S3 开发者**：需要稳定的 EYE 板适配做 vision/AI 应用
- **NuttX 上游社区**：希望最终把适配 upstream 回 apache/nuttx

## Goals / Non-Goals

**Goals:**

- ESP32-S3-EYE 从 openvela trunk 一键 `tools/configure.sh esp32s3-eye:openvela && make` 编译运行
- **14 个硬件外设全部工作**：CAM / LCD / PSRAM / Flash / SDMMC / I2S 麦克风 / WiFi / BLE / I2C / SPI / LEDC / DMA / Timer / USB-JTAG
- 通过 Vela xTS V2.0 通用自测核心 + EYE 全外设扩展 26 case ≥95% 通过率
- LCD 显示干净（无花屏），拍照画面完整居中显示
- 所有改动可 upstream 到 apache/nuttx 或 openvela
- 真实信号验证（WiFi 扫描真实 AP / BLE 扫描真实设备 / SD 卡读真实文件）

**Non-Goals:**

- 不实现 QMA7981 加速度计 driver（需要新写 sensor driver，工作量较大）
- 不做 cmocka 完整测试套件（依赖 cmocka 框架）
- 不做硬件加密加速器适配（mbedTLS hardware acceleration）
- 不做继电器冷启动压测（需要外部硬件）
- 不实现 SD 4-bit 模式（D1 GPIO16 与 CAM_D7 冲突无法解决）
- 不优化 WiFi/BLE coexistence（默认 ESP-IDF SW coex）

## Decisions

### 决定 1：CAM driver 来源 — Cherry-pick 上游 NuttX PR #18542

**选择**：使用社区已有的 esp32s3 CAM DVP driver（PR #18542），而非从零开发

**理由**：
- 上游 PR 已包含完整 LCD_CAM 寄存器操作 + DMA 配置 + V4L2 imgdata 接口
- 上游已修复 frame data 完整性 78%→100%
- 上游 PR 还有 work_queue（LPWORK）异步处理 dcache invalidate + memcpy

**API 适配**：openvela esp-hal-3rdparty 还没同步上游 `espressif/esp_gpio.h` 抽象层，需要：
- `espressif/esp_gpio.h → esp32s3_gpio.h`
- `espressif/esp_irq.h → esp32s3_irq.h`
- `cam_ll_*` HAL 函数 → 直接寄存器操作

### 决定 2：LCD 分辨率配置 — 改 YRES 320→240

**选择**：`CONFIG_LCD_ST7789_YRES=320 → 240`，保持 `YOFFSET=80`

**理由**：
- ESP-IDF 官方 esp-bsp 权威配置：`BSP_LCD_H_RES=240, BSP_LCD_V_RES=240`
- ST7789 controller 内部 GRAM 始终 240x320，物理面板覆盖中间 240x240，所以 YOFFSET=80 偏移依然需要
- driver 只 fill 240x240 framebuffer（节省 38KB framebuffer 内存）

### 决定 3：板级 Camera 文件位置 — nuttx tree 内 vs vendor

**选择**：`esp32s3_board_camera.c` 直接放在 `boards/xtensa/esp32s3/esp32s3-eye/src/` 而非 vendor 路径

**理由**：
- nuttx tree 自身 Make.defs 决定编译入 libboard.a
- 与 LCD 板级配置 (`esp32s3_board_lcd.c`) 同位置，结构对称
- vendor 那份保留作 reference

### 决定 4：Stack 配置 — CAMERA_STACKSIZE=8192

**选择**：`CONFIG_EXAMPLES_CAMERA_STACKSIZE` 从默认 2048 提升到 8192

**理由**：
- camera_main 在 `boardctl(BOARDIOC_INIT)` 同步路径里调 board_camera_initialize
- 整条链路 stack 占用 ≈ 5KB（V4L2 register + imgdata register + SPI/LEDC 操作）
- 经验性 8192 字节足够，2048 会导致 stack overflow → boot hang

### 决定 5：烧录稳定性 — erase-flash + 460800 baud

**选择**：每次烧录前 `erase-flash` + 460800 baudrate（不用 921600）

**理由**：
- ESP-ROM 启动检查 SHA-256 hash，如果 flash 残留有旧 image hash 不一致 → boot loop
- 921600 baudrate 在 long binary（~1.6MB）传输时偶发位错误 → 数据偶尔损坏
- 460800 是 esptool 的稳健默认值

### 决定 6：esp-hal-3rdparty spinlock 修复

**选择**：修改 `clk_ctrl_os.c` 和 `modem_clock.c` 的 `LOCK_INITIALIZER_UNLOCKED` 宏

```c
- #define LOCK_INITIALIZER_UNLOCKED       0
+ #define LOCK_INITIALIZER_UNLOCKED       SP_UNLOCKED
```

**理由**：
- NuttX `spinlock_t` 是 struct（不是 int），不能用 `0` 初始化
- `SP_UNLOCKED` 是 NuttX 标准宏
- 修复影响所有使用 esp-hal-3rdparty 的 board，universal fix

**部署方式**：作为 patch 文件保存在 `boards/xtensa/esp32s3/esp32s3-eye/scripts/patches/0001-esp-hal-3rdparty-fix-spinlock-init.patch`，build 后手动 `git apply`

### 决定 7：SDMMC 1-bit 模式

**选择**：启用 `CONFIG_SDIO_WIDTH_D1_ONLY=y`，只用 D0 (GPIO40)

**理由**：
- ESP32-S3-EYE 的 SD 卡座只接了 D0 line（不是完整 4-bit 接口）
- NuttX 默认配置 D1=GPIO16，与 CAM_D7=GPIO16 冲突
- 1-bit 模式速度足够（标准 SD 卡 ~25 Mbps），适合视频/图片读取

**Pin 映射**（来自 ESP-IDF esp-bsp）：
- CLK = GPIO39
- CMD = GPIO38
- D0 = GPIO40
- D1/D2/D3 = unused（1-bit 模式忽略）

### 决定 8：I2S 麦克风 — RX-only TDM 模式

**选择**：`CONFIG_ESP32S3_I2S0_RX=y` + `TX is not set`，标准 I2S TDM 模式（不用 PDM 因为 NuttX 没专门 PDM 支持）

**理由**：
- EYE 板麦克风是 ICS-43434 类似 INMP441 的 I2S 标准 TDM 数字麦克风（不是 PDM）
- 启用 TX 会浪费 GPIO 配置 + 内存（EYE 板 DOUT=NC）
- NuttX I2S 标准 driver 直接支持

**Pin 映射**：
- BCLK = GPIO41
- WS (LRCLK) = GPIO42
- DIN = GPIO2

### 决定 9：BLE 启用方式 — Kconfig select + sched_lock 修复

**选择**：双修复策略

**Kconfig 修复**（在 `arch/xtensa/src/esp32s3/Kconfig`）：

```kconfig
config ESP32S3_BLE
	bool "BLE"
	default n
	select ESP32S3_WIRELESS
	select ESPRESSIF_BLE      # ← 新增
	---help---
		Enable BLE support

config ESPRESSIF_BLE          # ← 新增 hidden symbol
	bool
	default n
	---help---
		Hidden symbol selected by ESP32S3_BLE to enable Bluetooth LE
		stack initialization in esp-hal-3rdparty sdkconfig.h.
```

**理由**：
- `esp-hal-3rdparty/nuttx/esp32s3/include/sdkconfig.h` 第 939 行：`#ifdef CONFIG_ESPRESSIF_BLE` 包裹整个 BT controller config 段
- 只启用 `CONFIG_ESP32S3_BLE` 不会触发 `CONFIG_BT_CTRL_*` / `CONFIG_BT_NIMBLE_*` 等 ESP-IDF macros 定义
- esp32s3_ble_adapter.c 引用这些 macros → 编译失败 (`'BT_CTRL_BLE_MAX_ACT_LIMIT' undeclared`)
- 加 hidden `ESPRESSIF_BLE` symbol 不污染用户 menuconfig 界面

**API 修复**（在 `esp32s3_ble_adapter.c`）：

```c
- ret = sched_lock();
- if (ret) { wlerr(...); return false; }
+ sched_lock();

- ret = sched_unlock();
- if (ret) { wlerr(...); return false; }
+ sched_unlock();
```

**理由**：
- NuttX 的 `sched_lock()` 和 `sched_unlock()` 返回 `void` 而不是 `int`
- `ret = sched_lock()` 触发 `error: void value not ignored as it ought to be`
- 直接 fix 才能编译通过

**替代方案考虑**：
- ❌ 不修 sched_lock 改用 `irqsave/irqrestore`：影响 BLE controller 线程优先级行为
- ❌ 强制定义 sched_lock 返回 int：破坏 NuttX 上游 API 不可接受

### 决定 10：BTSAK app + ALLOW_BSD_COMPONENTS

**选择**：启用 `CONFIG_BTSAK=y` + `CONFIG_ALLOW_BSD_COMPONENTS=y` + 完整 BLE host stack

**理由**：
- `CONFIG_WIRELESS_BLUETOOTH` 依赖 `ALLOW_BSD_COMPONENTS`（NuttX BLE host code 来自 BSD-licensed sources）
- `CONFIG_NET_BLUETOOTH` 依赖 `WIRELESS_BLUETOOTH` + 提供 `/dev/bnep0` 网络接口
- `CONFIG_BTSAK` 提供 `bt` 命令做 scan / advertise / connect 等测试操作

## Risks / Trade-offs

### 风险

[Risk 1] **esp-hal-3rdparty 上游升级冲突** → 在 esp-hal-3rdparty 内修改了 2 个文件，下次 sync 上游可能需要重新 patch  
**Mitigation**：保存为 `.patch` 文件 + 文档记录手动应用步骤；长期目标是 upstream 到 espressif/esp-hal-3rdparty

[Risk 2] **PR #18542 frame data 78% 修复在某些 sensor 上可能引入新问题** → 该修复是社区 work-in-progress  
**Mitigation**：仅在 OV2640 上验证；如其它 sensor 出问题，提供 `CONFIG_ESP32S3_CAM_LEGACY_ISR` 编译选项回退

[Risk 3] **LCD YRES 改动影响其它使用 ST7789 的板子** → 但 YRES 是每板独立 defconfig，不会影响  
**Mitigation**：仅修改 esp32s3-eye 自己的 defconfig

[Risk 4] **stack 8192 增大 SRAM 占用** → 相对 8MB OCT PSRAM 可忽略  
**Mitigation**：CONFIG_MM_REGIONS=2 让 PSRAM 加入 heap pool

[Risk 5] **BLE Kconfig 修复影响 esp32s3-devkit:blewifi 等已有配置** → 但实际 blewifi 也需要这个 fix（之前未发现因为没人完整跑过）  
**Mitigation**：修改是 select-based，向后兼容；blewifi 只会受益不会受损

[Risk 6] **sched_lock 修复影响其它使用 esp32s3_ble_adapter.c 的代码** → 没有，这是私有文件  
**Mitigation**：N/A

[Risk 7] **SDMMC 1-bit 模式速度受限** → ~25 Mbps 上限  
**Mitigation**：对于视频文件读取已足够；4-bit 模式因 GPIO 冲突无法实现

### Trade-offs

- **不启用 BOARD_LATE_INITIALIZE 异步路径**：保持 boardctl 同步初始化，简化调试。代价是 NSH 启动稍慢 ~50ms
- **YRES=240 而非 driver 内部 swap**：fb framebuffer 大小匹配物理 LCD（115200 vs 153600 bytes），节省 38KB
- **erase-flash + 慢 baud**：每次烧录多耗 5-10s，但稳定性 100%
- **不实现 PDM 麦克风模式**：用标准 I2S TDM 兼容 ICS-43434，简化 driver 路径
- **不实现 SD 4-bit 模式**：受 GPIO 冲突限制只能 1-bit，速度足够即可
- **BLE 与 WiFi 共存用 ESP-IDF SW coex**：默认配置足够使用，不优化 RF time-share

## Migration Plan

### 适配步骤（按 Phase）

#### Phase 1: 基础适配

1. 切换到 yanxingyu17/nuttx fork 的 `pr/esp32s3-eye-camera` 分支
2. 修 esp-hal-3rdparty spinlock（2 个文件）
3. 补 Kconfig 11 个 CAM PIN 子选项
4. 加 esp32s3_board_camera.c + Make.defs + bringup CAM 调用
5. LCD YRES 320→240
6. EXAMPLES_CAMERA_STACKSIZE 8192
7. 验证 NSH boot + camera 1 + fb

#### Phase 2: 全外设扩展

8. 启用 SDMMC（CLK=39, CMD=38, D0=40, 1-bit 模式 + FAT）
9. 启用 I2S MEMS 麦克风（BCLK=41, WS=42, DIN=2, RX-only）
10. 启用 BLE（Kconfig select ESPRESSIF_BLE + sched_lock fix + 完整 host stack）
11. 验证 wapi scan / bt scan / mount mmcsd1 / ls /dev/audio

#### Phase 3: 测试 + 文档

12. 编写 26 case XTS V3 自动化 runner
13. 跑全 case 100% PASS
14. 生成飞书云文档报告
15. 更新 OpenSpec 文档归档
16. push 到 GitHub fork

### 烧录流程（关键）

```bash
# 编译
./tools/configure.sh esp32s3-eye:openvela
cd arch/xtensa/src/chip/esp-hal-3rdparty && \
  git apply ../../../../../boards/xtensa/esp32s3/esp32s3-eye/scripts/patches/0001-*.patch
cd - && make -j$(nproc)

# 烧录（必须 erase + 慢 baud）
PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00
esptool.py -c esp32s3 -p $PORT -b 460800 erase-flash
esptool.py -c esp32s3 -p $PORT -b 460800 \
  --before usb-reset --after watchdog-reset \
  write-flash -fs detect -fm dio -ff 40m 0x0000 nuttx.bin
```

### 回滚策略

- 所有改动通过 git commit 记录
- 失败可用 `git revert` 回退到对应 commit
- esp-hal-3rdparty 修改可通过 `git checkout` 恢复

## Open Questions

1. **是否将本次修改 upstream 回 apache/nuttx？** → 倾向是，但需要先在 openvela 验证 1-2 周稳定性
2. **是否将 ESPRESSIF_BLE Kconfig 修复 upstream？** → 应该，这是 ESP32-S3 BLE 的根本缺陷
3. **是否补完 QMA7981 加速度计 driver？** → 后续 phase（需要 ~2-3 天工作）
4. **是否将 EYE 板纳入 openvela CI？** → 等 mainline 后再讨论
5. **WiFi+BLE 共存时性能数据？** → 后续 stress test 测量
