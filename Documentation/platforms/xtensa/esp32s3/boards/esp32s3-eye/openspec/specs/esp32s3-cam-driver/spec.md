## ADDED Requirements

### Requirement: ESP32-S3 LCD_CAM DVP 驱动
系统 SHALL 提供 ESP32-S3 LCD_CAM 控制器 DVP 模式驱动，支持 8-bit 并行图像传感器接入。

#### Scenario: 驱动初始化返回有效 imgdata 实例
- **WHEN** 调用 `esp32s3_cam_initialize()`
- **THEN** 返回非 NULL 的 `struct imgdata_s *` 实例

#### Scenario: 驱动注册到 V4L2 框架
- **WHEN** 内核执行 board_camera_initialize 流程
- **THEN** `imgdata_register()` + `imgsensor_register()` + `capture_register("/dev/video0", ...)` 全部成功返回 0

### Requirement: 帧数据完整性
驱动 SHALL 保证捕获的视频帧数据 100% 完整（无 78% 半帧问题）。

#### Scenario: ISR 不重启 CAM/DMA
- **WHEN** VSYNC ISR 触发处理一帧完成
- **THEN** ISR 仅设置 frame_done flag 并 work_queue 派遣 dcache invalidate + memcpy；不在 ISR 上下文重启 CAM/DMA

#### Scenario: 优雅停止 DMA
- **WHEN** stop_capture 调用
- **THEN** 设置 INLINK_STOP 寄存器并轮询等待 DMA 真正空闲（bounded wait），避免在传输中途切换 buffer

### Requirement: GPIO matrix 配置
驱动 SHALL 通过 GPIO matrix 把摄像头数据线和控制线映射到 LCD_CAM peripheral。

#### Scenario: 数据线输入方向配置
- **WHEN** 驱动初始化 GPIO
- **THEN** D0-D7 配置为 INPUT 模式并通过 `esp32s3_gpio_matrix_in()` 接到 CAM_DATA_IN_n 信号

#### Scenario: 时钟线输出方向配置
- **WHEN** 驱动初始化 XCLK 输出
- **THEN** XCLK 配置为 OUTPUT 模式并通过 `esp32s3_gpio_matrix_out()` 接到 LEDC 时钟源

### Requirement: V4L2 capture 接口
驱动 SHALL 实现 V4L2 标准的 capture 接口，使应用程序可通过 `/dev/video0` 操作。

#### Scenario: 应用程序 open /dev/video0
- **WHEN** 用户应用执行 `open("/dev/video0", O_RDWR)`
- **THEN** 返回有效 fd（不为 -1）；首次 open 时驱动调用 imgsensor->ops->init 完成 OV2640 sensor 初始化

#### Scenario: 应用 ioctl VIDIOC_S_FMT
- **WHEN** 应用调用 `ioctl(fd, VIDIOC_S_FMT, ...)` 设置 QVGA RGB565 格式
- **THEN** 驱动配置 CAM 寄存器和 OV2640 寄存器，返回成功 0

### Requirement: Kconfig 引脚定义
系统 SHALL 提供 11 个 ESP32S3_CAM_*_PIN Kconfig 子选项，让 board defconfig 可以独立指定每个引脚号。

#### Scenario: defconfig 设置 PIN
- **WHEN** board defconfig 包含 `CONFIG_ESP32S3_CAM_XCLK_PIN=15`
- **THEN** make olddefconfig 后该配置保留在最终 .config 中（不被裁剪）

#### Scenario: PIN 子选项依赖父开关
- **WHEN** `CONFIG_ESP32S3_CAM=n`
- **THEN** 所有 PIN 子选项自动隐藏（在 menuconfig 中不显示）

### Requirement: 驱动编译条件
ESP32-S3 cam driver 文件 SHALL 仅在 `CONFIG_ESP32S3_CAM=y` 时编译。

#### Scenario: Make.defs 启用编译
- **WHEN** `CONFIG_ESP32S3_CAM=y`
- **THEN** `arch/xtensa/src/esp32s3/Make.defs` 中 `CHIP_CSRCS += esp32s3_cam.c` 生效，最终 libarch.a 包含 esp32s3_cam.o

### Requirement: 板级摄像头集成
板级文件 `esp32s3_board_camera.c` SHALL 实现完整的 OV2640 初始化序列 + LEDC XCLK 启动 + V4L2 注册。

#### Scenario: board_camera_initialize 调用流程
- **WHEN** 调用 `board_camera_initialize()`
- **THEN** 顺序执行：(1) `esp32s3_cam_initialize()` 获取 imgdata; (2) `ov2640_start_xclk()` 启动 LEDC 20MHz on GPIO15; (3) `imgdata_register(imgdata)`; (4) `imgsensor_register(&g_ov2640_sensor)`; (5) `capture_register("/dev/video0", imgdata, sensors, 1)`
