## 1. 准备工作

- [x] 1.1 切换到 yanxingyu17/nuttx fork 的 `pr/esp32s3-eye-camera` 分支（含 6 cherry-pick PR commits）
- [x] 1.2 验证 esp-hal-3rdparty 锁定到 `9fc713a95b1ff150dd0b0647e465d3c624056bb1`
- [x] 1.3 拷贝 vendor `openvela` defconfig 到 `boards/xtensa/esp32s3/esp32s3-eye/configs/openvela/defconfig`
- [x] 1.4 安装 xtensa-esp32s3-elf gcc toolchain 到 `/opt/xtensa-esp32s3-elf/`

## 2. esp-hal-3rdparty spinlock 兼容性修复

- [x] 2.1 修改 `clk_ctrl_os.c` 第 27 行 `LOCK_INITIALIZER_UNLOCKED 0 → SP_UNLOCKED`
- [x] 2.2 修改 `modem_clock.c` 第 35 行同样的宏
- [x] 2.3 保存为 patch 文件 `boards/.../esp32s3-eye/scripts/patches/0001-esp-hal-3rdparty-fix-spinlock-init.patch`
- [x] 2.4 验证两个文件可以编译通过

## 3. 补全 ESP32S3_CAM Kconfig 子选项

- [x] 3.1 在 `arch/xtensa/src/esp32s3/Kconfig` 加 `if ESP32S3_CAM ... endif` 块
- [x] 3.2 添加 11 个 PIN 子选项：XCLK_PIN(15), PCLK_PIN(13), VSYNC_PIN(6), HREF_PIN(7), D0-D7_PIN(11/9/8/10/12/18/17/16)
- [x] 3.3 添加 VSYNC_INVERT bool 选项
- [x] 3.4 验证 olddefconfig 后子选项保留在 .config

## 4. 集成 esp32s3 CAM driver

- [x] 4.1 确认 `arch/xtensa/src/esp32s3/esp32s3_cam.c` 存在（PR #18542 cherry-pick）
- [x] 4.2 确认 `arch/xtensa/src/esp32s3/esp32s3_cam.h` 存在
- [x] 4.3 在 Make.defs 增加 `CHIP_CSRCS += esp32s3_cam.c`
- [x] 4.4 验证 libarch.a 包含 `esp32s3_cam_initialize` 符号

## 5. 板级 Camera 支持

- [x] 5.1 拷贝 `esp32s3_board_camera.c` 到 `boards/.../esp32s3-eye/src/`
- [x] 5.2 实现 `board_camera_initialize`：esp32s3_cam_initialize → ov2640_start_xclk → imgdata_register → imgsensor_register → capture_register("/dev/video0", ...)
- [x] 5.3 在 board Make.defs 增加 `ifeq ($(CONFIG_ESP32S3_CAM),y) CSRCS += esp32s3_board_camera.c endif`
- [x] 5.4 在 esp32s3_bringup.c 增加 `#ifdef CONFIG_ESP32S3_CAM` 块
- [x] 5.5 验证 libboard.a 包含 `board_camera_initialize`

## 6. LCD 240x240 配置修复

- [x] 6.1 修改 `CONFIG_LCD_ST7789_YRES=320 → 240`
- [x] 6.2 保留 `CONFIG_LCD_ST7789_YOFFSET=80`（ST7789 GRAM 内部偏移）
- [x] 6.3 保留 `CONFIG_LCD_ST7789_INVCOLOR=y` 和 `CONFIG_LCD_RPORTRAIT=y`
- [x] 6.4 验证 `fb` 命令打印 `xres: 240, yres: 240, fblen: 115200`
- [x] 6.5 验证 LCD 显示干净嵌套矩形（无右侧花屏）

## 7. Stack 与 Scheduler 配置

- [x] 7.1 设置 `CONFIG_EXAMPLES_CAMERA_STACKSIZE=8192`
- [x] 7.2 启用 `CONFIG_SCHED_HPWORK=y` + `HPWORKPRIORITY=224` + `HPWORKSTACKSIZE=2048`
- [x] 7.3 设置 `CONFIG_VIDEO_REQBUFS_COUNT_MAX=5`
- [x] 7.4 验证 boot 不再因 stack overflow 挂死

## 8. WiFi 验证（vendor defconfig 已启用）

- [x] 8.1 验证 `CONFIG_ESP32S3_WIFI=y` + `CONFIG_WIRELESS_WAPI=y` 在 .config
- [x] 8.2 启动后 `ifconfig` 显示 `wlan0`
- [x] 8.3 `wapi scan wlan0` 扫到至少 10 个真实 AP（验证 15 个含 MIPublic/MILAB）
- [x] 8.4 真实环境验证（小米办公室）

## 9. SDMMC 适配

- [x] 9.1 启用 `CONFIG_ESP32S3_SDMMC=y`
- [x] 9.2 配置 GPIO：CLK=39, CMD=38, D0=40
- [x] 9.3 启用 `CONFIG_SDIO_WIDTH_D1_ONLY=y`（避免 D1=GPIO16 与 CAM_D7 冲突）
- [x] 9.4 启用 FAT 文件系统：`CONFIG_FS_FAT=y` + `FAT_LFN=y` + `FAT_LCNAMES=y`
- [x] 9.5 验证 `/dev/mmcsd1` 设备节点出现
- [x] 9.6 验证 `mount -t vfat /dev/mmcsd1 /mnt/sd` 成功
- [x] 9.7 验证 `df` 显示正确容量（31GB SD 卡）
- [x] 9.8 验证可读取真实数据（230+ 真实文件）

## 10. I2S MEMS 麦克风适配

- [x] 10.1 启用 `CONFIG_ESP32S3_I2S=y` + `ESP32S3_I2S0=y` + `ESP32S3_I2S0_RX=y`
- [x] 10.2 关闭 TX：`# CONFIG_ESP32S3_I2S0_TX is not set`（EYE 板无扬声器）
- [x] 10.3 配置 GPIO：BCLK=41, WS=42, DIN=2
- [x] 10.4 启用 `CONFIG_AUDIO=y` + `AUDIO_I2S=y` + `DRIVERS_AUDIO=y`
- [x] 10.5 esp32s3-eye.h 增加 `board_i2sdev_initialize` prototype
- [x] 10.6 esp32s3_bringup.c 增加 I2S 初始化调用
- [x] 10.7 验证 `/dev/audio/pcm_in0` 设备节点出现

## 11. Bluetooth LE 适配

- [x] 11.1 修改 `arch/xtensa/src/esp32s3/Kconfig`：`config ESP32S3_BLE` 增加 `select ESPRESSIF_BLE`
- [x] 11.2 新增 hidden `config ESPRESSIF_BLE` symbol
- [x] 11.3 修复 `esp32s3_ble_adapter.c` 的 `sched_lock()` / `sched_unlock()` 返回值滥用（删 `ret =` 赋值 + error check）
- [x] 11.4 启用 `CONFIG_ESP32S3_BLE=y`
- [x] 11.5 启用 `CONFIG_ALLOW_BSD_COMPONENTS=y`（WIRELESS_BLUETOOTH 依赖）
- [x] 11.6 启用完整 BLE host stack：`DRIVERS_BLUETOOTH=y` + `NET_BLUETOOTH=y` + `WIRELESS_BLUETOOTH=y` + `BTSAK=y`
- [x] 11.7 验证编译通过（无 `BT_CTRL_PINNED_TO_CORE undeclared` 错误）
- [x] 11.8 验证 `bt bnep0 info` 显示 BDAddr（WiFi MAC + 2）
- [x] 11.9 验证 `bt bnep0 scan` 扫到至少 3 个真实 BLE 设备
- [x] 11.10 真实环境验证（扫到 Apple `0x004c` + 小米 `0x0006` vendor ID）

## 12. 编译与烧录验证

- [x] 12.1 执行 `make distclean && tools/configure.sh esp32s3-eye:openvela`
- [x] 12.2 重新打 patch（spinlock + Kconfig + Make.defs + bringup）
- [x] 12.3 执行 `make -j$(nproc)`，nuttx.bin 大小约 1.6MB
- [x] 12.4 烧录流程：`fuser -k $PORT && esptool.py -b 460800 erase-flash && write-flash`
- [x] 12.5 验证 NSH 启动 + 全部设备节点（fb0/i2c0/timer0/video0/audio/mmcsd1）
- [x] 12.6 验证 fb 命令显示干净 240x240 嵌套矩形（无右侧花屏）
- [x] 12.7 验证 camera 1 命令拍照画面显示在 LCD（用户视觉确认）

## 13. XTS V3 自测脚本（26 case）

- [x] 13.1 解析飞书 XTS 表格《Vela xTS全集-V2.xlsx》提取 vTS 通用自测用例 42 项
- [x] 13.2 标注每个 case 在 EYE 板的可行性
- [x] 13.3 编写 Python serial 自动化 runner（`/tmp/xts_runner_v3.py`）
- [x] 13.4 V1: 20 个核心 case
- [x] 13.5 V3: 26 case（新增 6 个 EYE 全外设 case：audio / mmcsd1 / mount / wifi scan / wifi ifconfig / ble info / ble scan）
- [x] 13.6 实跑 26 case，**100% PASS**
- [x] 13.7 生成 `/tmp/xts_results_v3.json` 测试结果数据

## 14. 测试报告发布

- [x] 14.1 报告内容：测试基本信息 / 总览 / 外设适配成果表 / 详细 case 表 / 关键证据 / 硬件验证 / Binary 信息 / GitHub 同步 / 结论
- [x] 14.2 通过 `feishu-mcp update-doc overwrite` 更新飞书文档
- [x] 14.3 文档 URL: https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh

## 15. 代码同步与文档归档

- [x] 15.1 创建 openspec change `esp32s3-eye-openvela-port`
- [x] 15.2 编写 proposal.md / design.md / specs/ / tasks.md（v1 基础版本）
- [x] 15.3 提交本次所有改动到 yanxingyu17/nuttx fork 的 `pr/esp32s3-eye-openvela-port-v2` 分支
- [x] 15.4 推送 commits 到 GitHub fork（13 个 commit：6 cherry-pick + 7 新增）
- [x] 15.5 在 nuttx tree 内 `Documentation/.../esp32s3-eye/openspec/` 归档 OpenSpec 文档
- [x] 15.6 v2 更新 openspec 加 SDMMC/I2S/BLE 完整适配内容
- [x] 15.7 同步最终 openspec 到 nuttx Documentation 并 push v3

## 16. 后续工作（不在本次 change 范围）

- [ ] 16.1 适配 QMA7981 加速度计 driver（NuttX 没现成 driver，需要新写 sensor driver）
- [ ] 16.2 接入 cmocka 完整测试套件以补全 vTS 1.1.x 系列
- [ ] 16.3 跑完 cTS 内核自测 1123 项 + 应用自测 796 项
- [ ] 16.4 提交 openvela 兼容性认证申请
- [ ] 16.5 上游 ESPRESSIF_BLE Kconfig 修复到 apache/nuttx
- [ ] 16.6 上游 esp-hal-3rdparty spinlock 修复到 espressif/esp-hal-3rdparty
- [ ] 16.7 上游 esp32s3_ble_adapter.c sched_lock 修复到 apache/nuttx
- [ ] 16.8 WiFi+BLE 共存性能基准测试
- [ ] 16.9 SD 卡写性能优化（当前 1-bit 模式 ~25 Mbps）
- [ ] 16.10 ADC 按钮 + RGB LED + IMU 适配
