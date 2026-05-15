## Context

### 当前状态

openvela 是基于 NuttX 的实时操作系统，由小米主导维护。openvela 社区已为多个 ESP32 板子（esp32s3-devkit、lckfb-szpi-esp32s3 等）提供官方适配，但 ESP32-S3-EYE 板存在以下不完整：

1. **CAM driver 未集成**：上游 NuttX PR #18542（esp32s3 CAM DVP driver）虽已 cherry-pick 到 yanxingyu17/nuttx 的 `pr/esp32s3-eye-camera` 分支，但相关 board-level 集成（Make.defs / bringup / Kconfig）不完整
2. **LCD driver 配置不正确**：`CONFIG_LCD_ST7789_YRES=320` 与实际 240x240 物理面板不匹配
3. **构建系统多处缺陷**：spinlock_t API 不兼容、Kconfig 缺 PIN 子选项、Make.defs 缺 board_camera.c

### 硬件参考

ESP32-S3-EYE 官方硬件特性（来自 [Espressif esp-bsp](https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_s3_eye) 权威配置）：

| 子系统 | 规格 |
|-------|------|
| MCU | ESP32-S3-WROOM-1 (8MB Flash + 8MB OCT PSRAM) |
| LCD | 1.3" **240x240** ST7789, SPI3, RGB565, 80MHz pclk, color invert |
| Camera | OV2640 DVP 8-bit, XCLK from LEDC GPIO15 |
| LCD GPIO | PCLK=21, MOSI=47, DC=43, CS=44, BL=48 |
| Camera GPIO | XCLK=15, PCLK=13, VSYNC=6, HREF=7, D0-D7=11/9/8/10/12/18/17/16 |
| I2C SCCB | SCL=5, SDA=4 |

### 约束

- **不能破坏其它板子**：esp32s3-devkit、esp32s3-korvo-2、esp32s3-box 等共享 common 代码
- **保留 openvela 上游兼容性**：所有改动应该可以 upstream 回 openvela 主线
- **遵循 NuttX 配置惯例**：使用 Kconfig + defconfig + Make.defs 标准结构
- **复用上游 PR**：尽量基于已有的 PR #18542 等社区贡献，不重复造轮子

### 利益相关方

- **openvela 兼容性认证组**：需要 EYE 板作为新的认证候选板
- **ESP32-S3 开发者**：需要稳定的 EYE 板适配以做 vision/AI 应用
- **NuttX 上游社区**：希望最终把适配 upstream 回 apache/nuttx

## Goals / Non-Goals

**Goals:**
- ESP32-S3-EYE 板能从 openvela trunk 编译出 `nuttx.bin` 直接烧录运行
- 所有核心硬件（CAM/LCD/PSRAM/Flash/I2C/SPI/UART/LEDC）工作
- 通过 Vela xTS V2.0 通用自测用例核心子集（≥95% 通过率）
- LCD 显示干净（无花屏），拍照画面完整居中显示
- 所有配置改动都可 upstream 到 apache/nuttx 或 openvela

**Non-Goals:**
- 不实现 ESP32-S3-EYE 的 SD card / 麦克风 / 加速度计驱动（这些不是 XTS 必测项）
- 不实现 WiFi / BLE 适配（vendor defconfig 默认关闭，先保证核心能力）
- 不做 cmocka 完整测试套件（依赖 cmocka 框架，需要单独适配）
- 不做硬件加密加速器适配（mbedTLS hardware acceleration）
- 不做继电器冷启动压测（需要外部硬件）

## Decisions

### 决定 1：CAM driver 来源 — Cherry-pick 上游 NuttX PR #18542

**选择**：使用社区已有的 esp32s3 CAM DVP driver（PR #18542），而非从零开发

**理由**：
- 上游 PR 已包含完整的 LCD_CAM 寄存器操作 + DMA 配置 + V4L2 imgdata 接口
- 上游 wangjianyu3 同事已修复 frame data 完整性 78%→100%（关键修复）
- 上游 PR 还有 work_queue（LPWORK）异步处理 dcache invalidate + memcpy

**替代方案**：
- ❌ 从零开发：开发 + 验证周期长，且无 upstream path
- ❌ 用 esp-camera 三方库：依赖 ESP-IDF runtime，不适合 NuttX

**API 适配**：因为 openvela esp-hal-3rdparty 还没同步上游的 `espressif/esp_gpio.h` 抽象层，需要：
- `espressif/esp_gpio.h` → `esp32s3_gpio.h`
- `espressif/esp_irq.h` → `esp32s3_irq.h`  
- `cam_ll_*` HAL 函数 → 直接寄存器操作

### 决定 2：LCD 分辨率配置 — 改 YRES 320→240

**选择**：将 `CONFIG_LCD_ST7789_YRES=320` 改为 `240`，保持 `YOFFSET=80`

**理由**：
- ESP-IDF 官方 esp-bsp 权威配置：`BSP_LCD_H_RES=240, BSP_LCD_V_RES=240`
- ST7789 controller 内部 GRAM 始终 240x320，物理面板覆盖中间 240x240，所以 YOFFSET=80 偏移依然需要
- driver 只 fill 240x240 framebuffer 后，st7789 driver 会做完整窗口设置 → 屏幕完整刷新无残留

**替代方案**：
- ❌ 保持 YRES=320 + 改 fb demo 限制：动 example 代码不干净
- ❌ 在板子源码里 wrap 一个虚拟 LCD：增加复杂度，违反 NuttX driver 习惯

### 决定 3：板级 Make.defs 集成 vs vendor 重复

**选择**：把 `esp32s3_board_camera.c` 直接放在 `boards/xtensa/esp32s3/esp32s3-eye/src/` 而不是 vendor 路径

**理由**：
- nuttx tree 自身 Make.defs 决定编译入 libboard.a
- 与 LCD 板级配置 (`esp32s3_board_lcd.c`) 同位置，结构对称
- vendor 那份保留作为 reference，不破坏 vendor tree

### 决定 4：Stack 配置 — CAMERA_STACKSIZE=8192

**选择**：将 `CONFIG_EXAMPLES_CAMERA_STACKSIZE` 从默认 2048 提升到 8192

**理由**：
- camera_main 在 `boardctl(BOARDIOC_INIT)` 同步路径里调用 board_camera_initialize
- 整条链路 stack 占用 ≈ 5KB（含 V4L2 register、imgdata register、SPI 操作、LEDC 配置）
- 经验性 8192 字节足够，2048 会导致 stack overflow → boot hang

**替代方案**：
- BOARD_LATE_INITIALIZE=y 让 bringup 在 idle task：测试发现仍然挂死（idle task stack 也限制）
- 用 SCHED_HPWORK 异步：要求 PR #18542 的 work_queue 路径配合，复杂度高

### 决定 5：烧录稳定性流程 — erase-flash + 460800 baud

**选择**：每次烧录前先 `erase-flash`，使用 460800 baudrate（不用 921600）

**理由**：
- ESP-ROM 启动检查 SHA-256 hash，如果 flash 残留有旧 image hash 不一致 → boot loop
- 921600 baudrate 在 long binary（~1.4MB）传输时偶发位错误 → 写入数据偶尔损坏
- 460800 是 esptool 的稳健默认值

### 决定 6：esp-hal-3rdparty spinlock 修复

**选择**：修改 `clk_ctrl_os.c` 和 `modem_clock.c` 的 `LOCK_INITIALIZER_UNLOCKED` 宏

```c
- #define LOCK_INITIALIZER_UNLOCKED       0
+ #define LOCK_INITIALIZER_UNLOCKED       SP_UNLOCKED
```

**理由**：
- NuttX 的 `spinlock_t` 是 struct（不是 int），不能用 `0` 初始化
- `SP_UNLOCKED` 是 NuttX 标准宏，所有版本兼容
- 修复影响所有使用 esp-hal-3rdparty 的 board，是 universal fix

## Risks / Trade-offs

### 风险

[Risk 1] **esp-hal-3rdparty 上游升级冲突** → 由于我们在 esp-hal-3rdparty 内修改了 2 个文件，下次 sync 上游 esp-hal-3rdparty 可能需要重新 patch  
**Mitigation**：把这 2 行修改提交到 espressif/esp-hal-3rdparty 上游，或在 nuttx 顶层维护 patch 文件

[Risk 2] **PR #18542 frame data 78% 修复在某些 sensor 上可能引入新问题** → 该修复是社区 work-in-progress  
**Mitigation**：仅在 OV2640 上验证；如其它 sensor 出问题，提供 `CONFIG_ESP32S3_CAM_LEGACY_ISR` 编译选项回退

[Risk 3] **LCD YRES 改动影响其它使用 ST7789 的板子** → 但 YRES 是每板独立 defconfig，不会影响  
**Mitigation**：仅修改 esp32s3-eye 自己的 defconfig，其它板独立

[Risk 4] **stack 8192 增大 SRAM 占用** → 相对 8MB OCT PSRAM 可忽略  
**Mitigation**：通过 CONFIG_MM_REGIONS=2 让 PSRAM 加入 heap pool

### Trade-offs

- **不实现 BOARD_LATE_INITIALIZE 异步路径**：保持 boardctl 同步初始化，简化 bringup 调试。代价是 NSH 启动稍慢（多 ~50ms 同步初始化）
- **YRES=240 而非 driver 内部 swap**：让 fb framebuffer 大小匹配物理 LCD（115200 bytes vs 153600 bytes），节省 38KB framebuffer 内存
- **使用 erase-flash + 慢 baud 而非 fast partial flash**：每次烧录多耗 5-10s，但稳定性 100% 提升

## Migration Plan

### 适配步骤

1. **配置基线**：基于 vendor `esp32s3-eye:openvela` defconfig
2. **打 patch**：
   - 修 esp-hal-3rdparty 的 spinlock 初始化（2 行）
   - 补 Kconfig 的 11 个 CAM PIN 子选项
   - 补 Make.defs 的 board_camera.c 编译条目
   - 补 bringup.c 的 board_camera_initialize 调用
3. **拷贝文件**：
   - cherry-pick PR #18542 的 esp32s3_cam.c / esp32s3_cam.h
   - 拷贝 esp32s3_board_camera.c
4. **修 LCD 配置**：YRES 320→240
5. **调整 stack**：CAMERA_STACKSIZE 2048→8192
6. **验证流程**：
   - `make distclean && tools/configure.sh esp32s3-eye:openvela && make`
   - `esptool.py erase-flash` + `write-flash` (460800 baud)
   - 验证 NSH 启动 + ls /dev 含 video0/fb0/i2c0
   - 验证 fb 命令显示干净嵌套矩形（无花屏）
   - 验证 camera 1 拍照画面显示在 LCD

### 回滚策略

- 所有改动通过 git commit 记录
- 失败可用 `git revert` 回退
- esp-hal-3rdparty 修改可通过 git checkout 恢复

## Open Questions

1. **是否将本次修改 upstream 回 apache/nuttx？** → 倾向是，但需要先在 openvela 验证 1-2 周稳定性
2. **是否补完 SD/MIC/IMU/BLE 适配？** → 后续 phase，本次优先核心能力
3. **是否将 EYE 板纳入 openvela CI？** → 等 esp32s3-eye 适配 mainline 后再讨论
