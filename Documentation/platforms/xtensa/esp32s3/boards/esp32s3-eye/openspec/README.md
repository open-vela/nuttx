## ESP32-S3-EYE openvela 适配 — OpenSpec Change 记录

本目录记录了 ESP32-S3-EYE 板从零适配 openvela 到通过 Vela xTS V2.0 自测的完整工程过程，遵循 OpenSpec 标准化文档结构。

### 文档结构

| 文件 | 内容 |
|------|------|
| `proposal.md` | **为什么** — 问题背景、改动列表、新增 capabilities、影响面 |
| `design.md` | **怎么做** — 关键技术决策、风险权衡、迁移计划 |
| `tasks.md` | **执行计划** — 13 个任务组的实施清单（已完成项目用 `[x]` 标记）|
| `specs/esp32s3-eye-board-port/spec.md` | **板级适配规格** — 编译/启动/LCD/Camera/文件系统的可测试需求 |
| `specs/esp32s3-cam-driver/spec.md` | **CAM 驱动规格** — DVP driver 接口、V4L2 集成、GPIO 配置需求 |
| `specs/xts-self-test/spec.md` | **XTS 自测规格** — 20 case 自动化测试、报告生成、飞书发布 |

### 关键技术决策（Quick Reference）

1. **CAM driver**：cherry-pick 上游 NuttX PR #18542（含 frame data 78%→100% 修复）
2. **LCD 分辨率**：240x240 (匹配 Espressif esp-bsp `BSP_LCD_V_RES=240`)，非默认的 240x320
3. **Stack 配置**：`EXAMPLES_CAMERA_STACKSIZE=8192`（2048 默认值会导致 boot hang）
4. **烧录流程**：必须 `erase-flash` + 460800 baud（避免 SHA-256 残留 boot loop）
5. **esp-hal-3rdparty 修复**：spinlock_t 初始化 `0` → `SP_UNLOCKED`（NuttX struct 不可零初始化）

### 验证结果

- ✅ NSH 启动稳定，全部设备节点（`/dev/{video0,fb0,i2c0,timer0,...}`）就绪
- ✅ OV2640 拍照成功 → 240x240 居中显示在 ST7789 LCD
- ✅ fb 命令显示干净嵌套同心矩形（无右侧花屏）
- ✅ Vela xTS V2.0 通用自测核心 20 case，**100% PASS**
- ✅ 测试报告已发布飞书：https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh

### 提交历史

```
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
- **上游 NuttX PR**: https://github.com/apache/nuttx/pull/18542 (esp32s3 CAM DVP driver)
- **ESP-IDF BSP 参考**: https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_s3_eye
- **Vela xTS 全集 V2.0**: https://mi.feishu.cn/sheets/U86PsF3O0hk7uytSlk5c8PzHnBc

### 下一步工作

未在本 change 范围的后续任务（详见 `tasks.md` 第 13 节）：

- 补 SD card / 麦克风 / IMU 驱动适配
- 启用 WiFi / BLE
- 接入 cmocka 完整测试套件
- 完成 cTS 内核自测（1123 项）+ 应用自测（796 项）
- 提交 openvela 兼容性认证申请
