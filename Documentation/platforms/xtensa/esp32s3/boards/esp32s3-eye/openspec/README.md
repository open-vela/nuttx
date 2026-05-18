## ESP32-S3-EYE openvela 全外设适配 — OpenSpec Change 记录 (v3)

本目录记录了 ESP32-S3-EYE 板从零适配 openvela 到 **14 外设全工作 + 通过完整 XTS 自测**的工程过程，遵循 OpenSpec 标准化文档结构。

### 文档结构

| 文件 | 内容 |
|------|------|
| `proposal.md` | **为什么** — 9 大问题清单、3 阶段改动列表、4 个新增 capabilities、影响面 |
| `design.md` | **怎么做** — 10 个关键技术决策、7 个风险权衡、迁移计划 |
| `tasks.md` | **执行清单** — 16 个任务组（已完成 14 组，未来 1 组待办） |
| `specs/esp32s3-eye-board-port/spec.md` | **板级适配规格** — 编译/启动/LCD/Camera/SDMMC/I2S/WiFi 等 11 个可测试需求 |
| `specs/esp32s3-cam-driver/spec.md` | **CAM 驱动规格** — DVP driver 接口、V4L2 集成、GPIO 配置 8 个需求 |
| `specs/esp32s3-ble-support/spec.md` | **BLE 适配规格** — Kconfig 链路完整性、SMP API 兼容、host stack 6 个需求 |
| `specs/xts-self-test/spec.md` | **XTS 自测规格** — 26 case 自动化、报告生成、飞书发布、真实信号验证 11 个需求 |

### 关键技术决策（Quick Reference）

1. **CAM driver**：cherry-pick 上游 NuttX PR #18542（含 frame data 78%→100% 修复）
2. **LCD 分辨率**：240x240（匹配 Espressif esp-bsp `BSP_LCD_V_RES=240`），非默认 240x320
3. **Stack 配置**：`EXAMPLES_CAMERA_STACKSIZE=8192`（2048 默认值会导致 boot hang）
4. **烧录流程**：必须 `erase-flash` + 460800 baud（避免 SHA-256 残留 boot loop）
5. **esp-hal-3rdparty 修复**：spinlock_t 初始化 `0` → `SP_UNLOCKED`（NuttX struct 不可零初始化）
6. **SDMMC 1-bit 模式**：避免 D1=GPIO16 与 CAM_D7=GPIO16 冲突
7. **I2S 麦克风 RX-only**：EYE 板 DOUT=NC，关 TX 节省资源
8. **BLE Kconfig 修复**：`ESP32S3_BLE` 增加 `select ESPRESSIF_BLE` 触发 sdkconfig.h 的 BT 段
9. **BLE API 修复**：`sched_lock()` / `sched_unlock()` NuttX 返回 void，不能 `ret = sched_lock()`
10. **完整 BLE host stack**：`ALLOW_BSD_COMPONENTS=y` + `WIRELESS_BLUETOOTH=y` + `BTSAK=y`

### 验证结果（v3 - 100% 全外设通过）

#### 14 个硬件外设全部工作

| 类别 | 外设 | 验证 |
|------|------|------|
| CPU/Mem | OCT PSRAM 8MB | `free` 显示 8.6MB Umem |
| CPU/Mem | LittleFS Flash 8MB | `/mnt/spif` 挂载 |
| Camera | OV2640 + V4L2 + DVP | `/dev/video0` + `camera 1` 拍照 |
| Display | ST7789 LCD 240x240 | `/dev/fb0` + 拍照画面正确显示 |
| Storage | SDMMC + 31GB SD | `/dev/mmcsd1` + VFAT 挂载 |
| Audio | I2S MEMS 麦克风 | `/dev/audio/pcm_in0` 注册 |
| Wireless | WiFi 802.11 b/g/n | 扫到 15 个真实 AP |
| Wireless | Bluetooth LE 5.0 | 扫到 5 个真实 BLE 设备 |
| Bus | I2C0 / SPI3 | OV2640 + ST7789 工作 |
| PWM | LEDC | 20MHz GPIO15 OV2640 XCLK |
| DMA/IRQ | CAM/SPI 隐式 | start/stop_capture 正常 |
| Console | USB-Serial JTAG | `/dev/ttyACM0` |
| Time | Timer / RNG | `/dev/timer0`, `/dev/random` |

#### XTS 自测：26/26 PASS（100%）

- ✅ 系统内核（5 case）：mm/ostest/getprime/hello/rand
- ✅ 系统应用（3 case）：uname/free/df
- ✅ 驱动 BSP（5 case）：echo/ls dev/I2C-SPI/UART/RNG
- ✅ 性能（1 case）：uptime
- ✅ EYE 基础（5 case）：camera init/fb test/camera 1/i2c bus/fb0
- ✅ EYE 全外设（7 case）：audio/mmcsd1/SD mount/wapi scan/ifconfig/bt info/bt scan

#### 真实信号验证

- **WiFi**：扫到 15 AP（含 MIPublic / MILAB / MiPlay 等小米办公室 WiFi）
- **BLE**：扫到 5 设备（含 Apple `0x004c` + 小米 `0x0006` vendor ID 广播包）
- **SD 卡**：读取 31GB 真实视频/图片文件（230+ 个文件）

#### 测试报告：[飞书云文档 V3](https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh)

### 提交历史

```
4a6156b esp32s3: enable BLE support via ESPRESSIF_BLE Kconfig + sched_lock fix
8cb6b88 boards/esp32s3-eye: enable I2S0 RX for MEMS microphone
899e691 boards/esp32s3-eye/openvela: enable WiFi + SDMMC for full peripheral support
136f1f3 Documentation/esp32s3-eye: add OpenSpec change records (v1)
26f4e9d Documentation/esp32s3-eye: add openvela config + flash/build guide
cb4ac84 boards/esp32s3-eye: add openvela defconfig + spinlock patch
7d64102 boards/esp32s3-eye: integrate CAM driver + LCD 240x240 fix
d9cebf0 esp32s3/spiflash: change LittleFS mount point to /mnt/spif (cherry-pick)
23c4412 esp32s3/cam: cherry-pick upstream CAM DVP driver (PR #18542)
0ede5ea esp32s3/dma: add channel reset helper for CAM driver (cherry-pick)
debb513 esp32s3/spi: add 16-bit byte swap for LCD RGB565 display (cherry-pick)
66721f4 esp32s3/lcd: fix shared interrupt register handling with CAM (cherry-pick)
f7a9653 video: add RGB565X (big-endian RGB565) pixel format support (cherry-pick)
```

### 相关链接

- **GitHub fork branch**: `pr/esp32s3-eye-openvela-port-v2` on https://github.com/yanxingyu17/nuttx
- **上游 NuttX CAM PR**: https://github.com/apache/nuttx/pull/18542
- **ESP-IDF BSP 参考**: https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_s3_eye
- **Vela xTS 全集 V2.0**: https://mi.feishu.cn/sheets/U86PsF3O0hk7uytSlk5c8PzHnBc
- **测试报告**: https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh

### 下一步工作

未在本 change 范围的后续任务（详见 `tasks.md` 第 16 节）：

- 适配 QMA7981 加速度计 driver（需新写 sensor driver）
- 接入 cmocka 完整测试套件以补全 vTS 1.1.x 系列
- 跑完 cTS 内核自测（1123 项）+ 应用自测（796 项）
- 提交 openvela 兼容性认证申请
- 上游 ESPRESSIF_BLE Kconfig 修复到 apache/nuttx
- 上游 esp-hal-3rdparty spinlock 修复到 espressif/esp-hal-3rdparty
- 上游 esp32s3_ble_adapter.c sched_lock 修复到 apache/nuttx
