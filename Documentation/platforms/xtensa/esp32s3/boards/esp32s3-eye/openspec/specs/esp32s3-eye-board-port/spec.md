## ADDED Requirements

### Requirement: 板级编译支持
ESP32-S3-EYE 板 SHALL 通过 `tools/configure.sh esp32s3-eye:openvela` 命令配置后能成功编译生成 `nuttx.bin`。

#### Scenario: 全新 distclean 后编译
- **WHEN** 执行 `make distclean && ./tools/configure.sh esp32s3-eye:openvela && make -j$(nproc)`
- **THEN** 编译过程无错误，生成有效 nuttx 和 nuttx.bin 文件

#### Scenario: 编译产物大小合理
- **WHEN** 编译成功完成
- **THEN** nuttx.bin 大小在 1.3MB - 1.5MB 范围内（含 LVGL + Video stack + LCD + Camera + LittleFS）

### Requirement: NSH 启动稳定性
板子 SHALL 在烧录后能稳定启动到 NuttShell 提示符。

#### Scenario: 标准烧录流程
- **WHEN** 执行 `esptool.py erase-flash` + `esptool.py -b 460800 write-flash 0x0000 nuttx.bin`
- **THEN** 设备 reset 后 15 秒内出现 `NuttShell (NSH)` 提示符并响应键盘输入

#### Scenario: NSH 可执行命令
- **WHEN** 在 nsh> 提示符输入 `ls /dev`
- **THEN** 输出包含 `console`、`fb0`、`i2c0`、`timer0`、`ttyACM0`、`video0`、`null`、`zero`、`random` 等设备节点

### Requirement: LCD 显示正确
板子 LCD SHALL 配置为 240x240 分辨率以匹配实际硬件物理面板。

#### Scenario: framebuffer 信息查询
- **WHEN** 在 nsh 中执行 `fb` 命令
- **THEN** 输出 `xres: 240`, `yres: 240`, `fblen: 115200` (= 240 × 240 × 2 bytes)

#### Scenario: fb 测试图案显示无花屏
- **WHEN** 在 nsh 中执行 `fb` 命令
- **THEN** LCD 屏幕显示嵌套同心矩形 pattern，整个 240x240 区域内容完整无花屏，无右侧噪声

### Requirement: 摄像头拍照与 LCD 显示
板子 SHALL 支持 OV2640 摄像头拍照并实时显示到 LCD。

#### Scenario: camera 命令执行
- **WHEN** 在 nsh 中执行 `camera 1`
- **THEN** 输出包含 `OV2640 sensor configured for QVGA RGB565`、`Start capturing...`、`LCD: 240x240 crop (stride=480)`、`Finished capturing...`

#### Scenario: 拍照结果显示在 LCD
- **WHEN** camera 1 命令成功执行
- **THEN** LCD 屏幕显示拍摄到的真实场景画面（240x240 居中显示），并保存 RGB 文件到 `/mnt/spif`

### Requirement: 文件系统挂载
板子 SHALL 在 SPI flash 上挂载 LittleFS 到 `/mnt/spif`。

#### Scenario: 挂载点存在
- **WHEN** 在 nsh 中执行 `ls /mnt/spif`
- **THEN** 命令成功返回挂载点的内容（即使为空也不报 "No such file"）

### Requirement: 系统资源验证
板子 SHALL 正确启用 8MB OCT PSRAM 并加入到内核 heap pool。

#### Scenario: free 命令显示总内存
- **WHEN** 在 nsh 中执行 `free`
- **THEN** 输出 `Umem` 行 `total` 字段 ≥ 8MB（≥ 8000000）

### Requirement: 板级 GPIO 配置
板子 SHALL 配置正确的 GPIO 引脚号匹配 ESP32-S3-EYE 硬件原理图。

#### Scenario: Camera GPIO 配置
- **WHEN** 检查 .config 中 CAM 相关配置
- **THEN** `CONFIG_ESP32S3_CAM_XCLK_PIN=15`, `PCLK_PIN=13`, `VSYNC_PIN=6`, `HREF_PIN=7`, `D0_PIN=11`, `D1_PIN=9`, `D2_PIN=8`, `D3_PIN=10`, `D4_PIN=12`, `D5_PIN=18`, `D6_PIN=17`, `D7_PIN=16`

#### Scenario: I2C SCCB GPIO 配置
- **WHEN** 检查 OV2640 SCCB I2C 总线
- **THEN** I2C0 配置 SDA=GPIO4, SCL=GPIO5
