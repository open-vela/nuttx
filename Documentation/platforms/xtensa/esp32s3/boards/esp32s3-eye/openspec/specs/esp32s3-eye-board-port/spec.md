## ADDED Requirements

### Requirement: 板级编译支持
ESP32-S3-EYE 板 SHALL 通过 `tools/configure.sh esp32s3-eye:openvela` 命令配置后能成功编译生成 `nuttx.bin`。

#### Scenario: 全新 distclean 后编译
- **WHEN** 执行 `make distclean && ./tools/configure.sh esp32s3-eye:openvela && make -j$(nproc)`
- **THEN** 编译过程无错误，生成有效 nuttx 和 nuttx.bin 文件

#### Scenario: 编译产物大小合理
- **WHEN** 编译成功完成（含全部外设：CAM/LCD/WiFi/BLE/SDMMC/I2S/LVGL）
- **THEN** nuttx.bin 大小在 1.4MB - 1.8MB 范围内

### Requirement: NSH 启动稳定性
板子 SHALL 在烧录后能稳定启动到 NuttShell 提示符。

#### Scenario: 标准烧录流程
- **WHEN** 执行 `esptool.py -b 460800 erase-flash` + `esptool.py -b 460800 write-flash 0x0000 nuttx.bin`
- **THEN** 设备 reset 后 25 秒内出现 `NuttShell (NSH)` 提示符并响应键盘输入

#### Scenario: NSH 设备节点完整
- **WHEN** 在 nsh 中执行 `ls /dev`
- **THEN** 输出包含 `console`、`fb0`、`i2c0`、`timer0`、`ttyACM0`、`video0`、`audio/`、`mmcsd1`、`null`、`zero`、`random`

### Requirement: LCD 240x240 显示正确
板子 LCD SHALL 配置为 240x240 分辨率以匹配实际硬件物理面板。

#### Scenario: framebuffer 信息查询
- **WHEN** 在 nsh 中执行 `fb` 命令
- **THEN** 输出 `xres: 240`, `yres: 240`, `fblen: 115200`

#### Scenario: fb 测试图案显示无花屏
- **WHEN** 在 nsh 中执行 `fb` 命令
- **THEN** LCD 屏幕显示嵌套同心矩形 pattern，整个 240x240 区域内容完整无花屏

### Requirement: 摄像头拍照与 LCD 显示
板子 SHALL 支持 OV2640 摄像头拍照并实时显示到 LCD。

#### Scenario: camera 命令执行
- **WHEN** 在 nsh 中执行 `camera 1`
- **THEN** 输出 `OV2640 sensor configured for QVGA RGB565`、`Start capturing...`、`LCD: 240x240 crop`、`Finished capturing...`

#### Scenario: 拍照结果显示在 LCD
- **WHEN** camera 1 命令成功执行
- **THEN** LCD 屏幕显示拍摄到的真实场景画面（240x240 居中），并保存 RGB 文件到 `/mnt/spif`

### Requirement: LittleFS 文件系统
板子 SHALL 在 SPI flash 上挂载 LittleFS 到 `/mnt/spif`。

#### Scenario: 挂载点存在
- **WHEN** 在 nsh 中执行 `ls /mnt/spif`
- **THEN** 命令成功返回挂载点的内容（即使为空也不报 "No such file"）

### Requirement: SDMMC + SD 卡支持
板子 SHALL 通过 SDMMC 1-bit 模式识别和挂载 MicroSD 卡。

#### Scenario: SD 卡设备节点出现
- **WHEN** 在 nsh 中执行 `ls /dev/mmcsd1`
- **THEN** 命令成功返回 `/dev/mmcsd1`，不报 "No such file"

#### Scenario: VFAT 挂载成功
- **WHEN** 执行 `mount -t vfat /dev/mmcsd1 /mnt/sd`
- **THEN** 命令成功返回，`mount` 输出包含 `/mnt/sd type vfat`

#### Scenario: SD 卡容量正确
- **WHEN** 挂载 SD 卡后执行 `df`
- **THEN** 输出包含 `/mnt/sd` 行，Block Size 为 65536，Number of Blocks 反映实际卡容量

#### Scenario: 1-bit 模式不冲突
- **WHEN** 启用 `CONFIG_SDIO_WIDTH_D1_ONLY=y`
- **THEN** SDMMC 仅使用 D0=GPIO40，不触碰 D1=GPIO16（避免与 CAM_D7 GPIO16 冲突）

### Requirement: I2S MEMS 麦克风音频输入
板子 SHALL 通过 I2S0 RX 模式注册音频捕获设备 `/dev/audio/pcm_in0`。

#### Scenario: 音频设备节点
- **WHEN** 执行 `ls /dev/audio`
- **THEN** 输出包含 `pcm_in0`

#### Scenario: I2S0 引脚配置正确
- **WHEN** 检查 .config
- **THEN** `CONFIG_ESP32S3_I2S0_BCLKPIN=41`, `WSPIN=42`, `DINPIN=2`, `ESP32S3_I2S0_RX=y`, `ESP32S3_I2S0_TX is not set`

### Requirement: WiFi 网络支持
板子 SHALL 启用 WiFi 802.11 b/g/n station 模式。

#### Scenario: wlan0 接口注册
- **WHEN** 执行 `ifconfig`
- **THEN** 输出包含 `wlan0 ... HWaddr xx:xx:xx:xx:xx:xx`

#### Scenario: WiFi 扫描可用
- **WHEN** 执行 `wapi scan wlan0`
- **THEN** 输出 `bssid / frequency / signal level / encode / ssid` 表头 + 至少 1 个 AP 条目

### Requirement: 系统资源验证
板子 SHALL 正确启用 8MB OCT PSRAM 并加入到内核 heap pool。

#### Scenario: free 命令显示总内存
- **WHEN** 执行 `free`
- **THEN** 输出 `Umem` 行 `total` 字段 ≥ 8MB（≥ 8000000）

### Requirement: 板级 GPIO 配置
板子 SHALL 配置正确的 GPIO 引脚号匹配 ESP32-S3-EYE 硬件原理图。

#### Scenario: Camera GPIO 配置
- **WHEN** 检查 .config 中 CAM 相关配置
- **THEN** PIN 配置匹配：XCLK=15, PCLK=13, VSYNC=6, HREF=7, D0=11, D1=9, D2=8, D3=10, D4=12, D5=18, D6=17, D7=16

#### Scenario: I2C SCCB GPIO 配置
- **WHEN** 检查 OV2640 SCCB I2C 总线
- **THEN** I2C0 配置 SDA=GPIO4, SCL=GPIO5

#### Scenario: SDMMC GPIO 配置
- **WHEN** 检查 SDMMC 配置
- **THEN** CLK=GPIO39, CMD=GPIO38, D0=GPIO40

#### Scenario: I2S 麦克风 GPIO 配置
- **WHEN** 检查 I2S0 配置
- **THEN** BCLK=GPIO41, WS=GPIO42, DIN=GPIO2
