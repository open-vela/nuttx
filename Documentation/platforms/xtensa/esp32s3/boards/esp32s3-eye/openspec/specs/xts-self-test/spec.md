## ADDED Requirements

### Requirement: XTS 自测套件
系统 SHALL 提供按 Vela xTS 全集 V2.0 通用自测用例 + EYE 全外设扩展的自动化测试脚本。

#### Scenario: 测试用例数量
- **WHEN** 执行自测脚本（V3 版本）
- **THEN** 至少跑 26 个核心 test case，覆盖 5 个大类：系统内核 / 系统应用 / 驱动 BSP / 性能 / EYE 全外设（含 audio/SDMMC/WiFi/BLE）

#### Scenario: 通过率达标
- **WHEN** 自测套件执行完成
- **THEN** 输出通过率 100%（26/26 PASS）

### Requirement: 系统内核测试
自测套件 SHALL 验证 NuttX 内核核心能力：内存管理、调度、系统调用、OS 测试套件。

#### Scenario: mm 内存测试
- **WHEN** 执行 `mm` 命令
- **THEN** 输出 `TEST COMPLETE`，无 ERROR

#### Scenario: ostest 综合测试
- **WHEN** 执行 `ostest` 命令
- **THEN** 输出 `ostest_main: Exiting with status 0`

#### Scenario: getprime 算力测试
- **WHEN** 执行 `getprime` 命令
- **THEN** 输出 `getprime took <N> msec`

#### Scenario: hello 用户态测试
- **WHEN** 执行 `hello` 命令
- **THEN** 输出 `Hello, World!!`

### Requirement: 系统应用测试
自测套件 SHALL 验证系统应用基本功能：reboot、free、df。

#### Scenario: 启动正常
- **WHEN** 执行 `uname -a`
- **THEN** 输出包含 `NuttX` 字符串

#### Scenario: free 显示 RAM
- **WHEN** 执行 `free`
- **THEN** 输出包含 total/used/free 三列并显示 8MB 量级数字

#### Scenario: df 显示文件系统
- **WHEN** 执行 `df`
- **THEN** 输出包含 `/proc` 或 `/mnt` 表项

### Requirement: 驱动 BSP 测试
自测套件 SHALL 验证关键驱动设备节点存在：Flash/GPIO/I2C/SPI/UART/RNG/audio/mmcsd1。

#### Scenario: 设备节点完整
- **WHEN** 执行 `ls /dev`
- **THEN** 输出包含 `console`, `fb0`, `i2c0`, `audio`, `mmcsd1`, `null`, `random`, `timer0`, `ttyACM0`, `video0`, `zero`

#### Scenario: I2C/SPI/Video 验证
- **WHEN** 执行 `ls /dev/i2c0 /dev/fb0 /dev/video0`
- **THEN** 三个设备节点都存在不报 No such file

#### Scenario: 随机数生成
- **WHEN** 执行 `rand 5`
- **THEN** 输出包含数字（≥ 3 位）

### Requirement: EYE 基础硬件测试
自测套件 SHALL 验证 ESP32-S3-EYE 板基础硬件：OV2640 + ST7789 + I2C bus + fb 命令。

#### Scenario: i2c bus 扫描
- **WHEN** 执行 `i2c bus`
- **THEN** 输出 `Bus 0: YES`（OV2640 SCCB 接此总线）

#### Scenario: fb framebuffer 测试
- **WHEN** 执行 `fb` 命令
- **THEN** 命令完成不挂起，LCD 显示嵌套同心矩形 pattern

#### Scenario: camera 拍照测试
- **WHEN** 执行 `camera 1` 命令
- **THEN** OV2640 配置 QVGA RGB565，CAM driver 完成 1 张图像捕获，LCD 显示拍照画面

### Requirement: EYE 扩展外设测试
自测套件 SHALL 验证新增外设：audio / SDMMC / WiFi / BLE。

#### Scenario: 音频设备节点
- **WHEN** 执行 `ls /dev/audio`
- **THEN** 输出包含 `pcm_in0`

#### Scenario: SD 卡识别
- **WHEN** 执行 `ls /dev/mmcsd1`
- **THEN** 命令成功返回 `/dev/mmcsd1`

#### Scenario: SD 卡 VFAT 挂载
- **WHEN** 执行 `mount -t vfat /dev/mmcsd1 /mnt/sd`
- **THEN** 命令成功，`df` 显示 `/mnt/sd` 容量

#### Scenario: WiFi 扫描真实 AP
- **WHEN** 执行 `wapi scan wlan0`
- **THEN** 输出 `bssid / frequency / signal level / encode / ssid` 表头 + 至少 1 个真实 AP（验证扫到 15 个含 MIPublic/MILAB 等小米办公室 WiFi）

#### Scenario: WiFi 接口配置
- **WHEN** 执行 `ifconfig`
- **THEN** 输出包含 `wlan0 ... HWaddr xx:xx:xx:xx:xx:xx`

#### Scenario: BLE 接口信息
- **WHEN** 执行 `bt bnep0 info`
- **THEN** 输出 `Device: bnep0`, `BDAddr: xx:xx:xx:xx:xx:xx`

#### Scenario: BLE 扫描真实设备
- **WHEN** 执行 `bt bnep0 scan start; sleep 5; bt bnep0 scan get`
- **THEN** 输出 `Scan result:` + 至少 1 个真实 BLE 设备（验证扫到 5 个含 Apple `0x004c` 和 小米 `0x0006` vendor ID）

### Requirement: 测试报告生成
自测套件 SHALL 生成结构化的 Markdown 报告，包含通过率、分类统计、详细 case 结果、关键证据。

#### Scenario: 报告输出文件
- **WHEN** 测试套件执行完成
- **THEN** 生成 `/tmp/xts_results_v3.json` 和 `/tmp/xts_report_v3.md`

#### Scenario: 报告内容完整性
- **WHEN** 检查 `xts_report_v3.md`
- **THEN** 包含测试基本信息、测试总览、外设适配成果表、详细 case 表格、关键证据代码块、Binary 信息、GitHub 同步状态、结论 8 个章节

### Requirement: 飞书云文档发布
测试报告 SHALL 通过 feishu MCP 工具发布为飞书云文档供团队查看。

#### Scenario: 飞书文档创建/更新
- **WHEN** 调用 `feishu-mcp create-doc` 或 `update-doc` 上传 markdown
- **THEN** 返回有效的 doc_url（验证 https://www.feishu.cn/docx/DVvBdlNoFoLgx2x4HC6cYRYanOh 可访问）

### Requirement: 真实信号验证
自测套件 SHALL 通过真实环境信号（不是 loopback 或 mock）验证 WiFi/BLE/SD 卡能力。

#### Scenario: WiFi 真实环境验证
- **WHEN** 在小米办公室环境跑 wapi scan
- **THEN** 扫描结果包含至少 10 个真实 AP，含 SSID 字符串 `MIPublic` 或 `MILAB` 等

#### Scenario: BLE 真实环境验证
- **WHEN** 跑 bt scan
- **THEN** 扫描结果包含至少 3 个真实 BLE 设备（验证 advertiser data 含 vendor ID `0x004c`=Apple 或 `0x0006`=小米）

#### Scenario: SD 卡真实数据读取
- **WHEN** 挂载 SD 卡后 `ls /mnt/sd`
- **THEN** 输出包含真实文件（验证 230+ 个 video/picture 文件 + log 子目录），`df` 显示 ~31GB 容量
