## 1. 准备工作

- [x] 1.1 切换到 yanxingyu17/nuttx fork 的 `pr/esp32s3-eye-camera` 分支（含 6 个 cherry-pick PR commits）
- [x] 1.2 验证 esp-hal-3rdparty 锁定到 commit `9fc713a95b1ff150dd0b0647e465d3c624056bb1`
- [x] 1.3 拷贝 vendor `openvela` defconfig 到 `boards/xtensa/esp32s3/esp32s3-eye/configs/openvela/defconfig`
- [x] 1.4 安装 xtensa-esp32s3-elf gcc toolchain 到 `/opt/xtensa-esp32s3-elf/`

## 2. 修复 esp-hal-3rdparty spinlock 兼容性

- [x] 2.1 修改 `arch/xtensa/src/chip/esp-hal-3rdparty/components/esp_hw_support/clk_ctrl_os.c` 第 27 行 `LOCK_INITIALIZER_UNLOCKED 0` → `SP_UNLOCKED`
- [x] 2.2 修改 `arch/xtensa/src/chip/esp-hal-3rdparty/components/esp_hw_support/modem_clock.c` 第 35 行同样的宏
- [x] 2.3 验证两个文件可以编译通过

## 3. 补全 ESP32S3_CAM Kconfig 子选项

- [x] 3.1 在 `arch/xtensa/src/esp32s3/Kconfig` 的 ESP32S3_CAM 配置后增加 `if ESP32S3_CAM ... endif` 块
- [x] 3.2 在块内添加 11 个 PIN 子选项：XCLK_PIN(default 15), PCLK_PIN(13), VSYNC_PIN(6), HREF_PIN(7), D0-D7_PIN(11/9/8/10/12/18/17/16)
- [x] 3.3 添加 VSYNC_INVERT bool 选项
- [x] 3.4 验证 `make olddefconfig` 后子选项保留在 .config 中

## 4. 集成 esp32s3 CAM driver

- [x] 4.1 确认 `arch/xtensa/src/esp32s3/esp32s3_cam.c` 存在（从 PR #18542 cherry-pick）
- [x] 4.2 确认 `arch/xtensa/src/esp32s3/esp32s3_cam.h` 存在（含 `esp32s3_cam_initialize` prototype）
- [x] 4.3 在 `arch/xtensa/src/esp32s3/Make.defs` 增加 `CHIP_CSRCS += esp32s3_cam.c`
- [x] 4.4 验证 libarch.a 包含 esp32s3_cam.o + 关键符号 `esp32s3_cam_initialize`

## 5. 板级 Camera 支持

- [x] 5.1 拷贝 vendor `esp32s3_board_camera.c` 到 `boards/xtensa/esp32s3/esp32s3-eye/src/`
- [x] 5.2 在该文件 `board_camera_initialize` 函数中实现：esp32s3_cam_initialize → ov2640_start_xclk → imgdata_register → imgsensor_register → capture_register("/dev/video0", ...)
- [x] 5.3 在 `boards/xtensa/esp32s3/esp32s3-eye/src/Make.defs` 增加 `ifeq ($(CONFIG_ESP32S3_CAM),y) CSRCS += esp32s3_board_camera.c endif`
- [x] 5.4 在 `boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_bringup.c` 增加 `#ifdef CONFIG_ESP32S3_CAM ... board_camera_initialize() ... #endif` 块（位于 SDMMC 初始化后）
- [x] 5.5 验证 libboard.a 包含 esp32s3_board_camera.o + `board_camera_initialize` 符号

## 6. LCD 240x240 配置修复

- [x] 6.1 修改 .config 中 `CONFIG_LCD_ST7789_YRES=320` → `CONFIG_LCD_ST7789_YRES=240`
- [x] 6.2 保留 `CONFIG_LCD_ST7789_YOFFSET=80`（ST7789 GRAM 内部偏移）
- [x] 6.3 保留 `CONFIG_LCD_ST7789_INVCOLOR=y` 和 `CONFIG_LCD_RPORTRAIT=y`
- [x] 6.4 验证 `fb` 命令打印 `xres: 240, yres: 240, fblen: 115200`

## 7. Stack 与 Scheduler 配置

- [x] 7.1 设置 `CONFIG_EXAMPLES_CAMERA_STACKSIZE=8192`
- [x] 7.2 启用 `CONFIG_SCHED_HPWORK=y`、`HPWORKPRIORITY=224`、`HPWORKSTACKSIZE=2048`（CAM driver work_queue 路径需要）
- [x] 7.3 设置 `CONFIG_VIDEO_REQBUFS_COUNT_MAX=5`（与 vendor defconfig 对齐）
- [x] 7.4 验证 boot 不再因 stack overflow 挂死

## 8. XTS 测试相关 CONFIG

- [x] 8.1 启用 `CONFIG_TESTING_OSTEST=y` + `CONFIG_TESTING_GETPRIME=y` + `CONFIG_TESTING_MM=y`
- [x] 8.2 启用 `CONFIG_EXAMPLES_HELLO=y` + `CONFIG_EXAMPLES_FB=y` + `CONFIG_EXAMPLES_CAMERA=y`
- [x] 8.3 启用 `CONFIG_SYSTEM_I2CTOOL=y`
- [x] 8.4 启用 `CONFIG_VIDEO_STREAM=y`
- [x] 8.5 启用 `CONFIG_ESP32S3_LEDC=y` + `LEDC_TIM0=y` + `CAM_XCLK_LEDC=y`

## 9. 编译与烧录验证

- [x] 9.1 执行 `make distclean && tools/configure.sh esp32s3-eye:openvela`
- [x] 9.2 重新打 patch（spinlock + Kconfig + Make.defs + bringup）
- [x] 9.3 执行 `make -j$(nproc)`，nuttx.bin 大小约 1.4MB
- [x] 9.4 烧录流程：`fuser -k $PORT && esptool.py -b 460800 erase-flash && esptool.py -b 460800 write-flash 0x0000 nuttx.bin`
- [x] 9.5 验证 NSH 启动 + ls /dev 含 video0/fb0/i2c0
- [x] 9.6 验证 fb 命令显示干净 240x240 嵌套矩形（无右侧花屏）
- [x] 9.7 验证 camera 1 命令拍照画面显示在 LCD（用户视觉确认）

## 10. XTS 自测脚本

- [x] 10.1 解析飞书 XTS 表格《Vela xTS全集-V2.xlsx》提取 vTS 通用自测用例 42 项
- [x] 10.2 标注每个 case 在 EYE 板的可行性：自动跑 36 / 人工 3 / 跳过 2 / 替代验证 1
- [x] 10.3 编写 Python serial 自动化 runner（`/tmp/xts_runner.py`）：reset → 等 nsh > → 顺序发命令 → regex 匹配 PASS/FAIL pattern
- [x] 10.4 选 20 个核心 case 实际跑测：6 内核 + 3 应用 + 5 BSP + 1 性能 + 5 EYE 专属
- [x] 10.5 生成 `/tmp/xts_results.json` 测试结果数据
- [x] 10.6 生成 `/tmp/xts_report.md` 结构化报告

## 11. 测试报告发布

- [x] 11.1 报告内容：测试基本信息 / 总览 / 分类统计 / 详细 case 表 / 关键证据 / 硬件验证 / 测试方法 / 结论
- [x] 11.2 通过 `feishu-mcp create-doc` 创建飞书文档发布
- [x] 11.3 文档 URL: https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh

## 12. 代码同步与文档归档

- [x] 12.1 创建 openspec change `esp32s3-eye-openvela-port` 记录全过程
- [ ] 12.2 编写 proposal.md / design.md / specs/ / tasks.md
- [ ] 12.3 提交本次所有改动到 yanxingyu17/nuttx fork 的对应 branch
- [ ] 12.4 推送 commit 到 GitHub fork
- [ ] 12.5 后续：向 openvela 上游或 apache/nuttx 提交 PR

## 13. 后续工作（不在本次 change 范围）

- [ ] 13.1 补 SD card / 麦克风 / 加速度计驱动适配
- [ ] 13.2 启用 WiFi / BLE
- [ ] 13.3 接入 cmocka 完整测试套件以补全 vTS 1.1.x 系列
- [ ] 13.4 跑完 cTS 内核自测 1123 项 + 应用自测 796 项
- [ ] 13.5 提交 openvela 兼容性认证申请
