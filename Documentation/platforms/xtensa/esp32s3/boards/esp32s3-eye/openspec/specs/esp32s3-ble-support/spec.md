## ADDED Requirements

### Requirement: ESP32-S3 BLE Kconfig 链路完整
NuttX `arch/xtensa/src/esp32s3/Kconfig` SHALL 通过 `select ESPRESSIF_BLE` 触发 esp-hal-3rdparty sdkconfig.h 中的 BT controller 配置段。

#### Scenario: ESP32S3_BLE 启用时自动 select ESPRESSIF_BLE
- **WHEN** 用户在 menuconfig 启用 `CONFIG_ESP32S3_BLE=y`
- **THEN** `CONFIG_ESPRESSIF_BLE=y` 自动加入到 .config

#### Scenario: ESPRESSIF_BLE 触发 sdkconfig.h BT 段
- **WHEN** 编译时检查 `arch/xtensa/src/chip/esp-hal-3rdparty/components/esp_common/include/esp_assert.h`
- **THEN** `#ifdef CONFIG_ESPRESSIF_BLE` 包裹的 BT_NIMBLE_ENABLED / BT_CTRL_PINNED_TO_CORE / BT_CONTROLLER_ENABLED 等 macros 被定义

#### Scenario: ESPRESSIF_BLE 是 hidden symbol
- **WHEN** 用户运行 `make menuconfig`
- **THEN** `ESPRESSIF_BLE` 不显示在菜单中（hidden Kconfig symbol，仅供 select 使用）

### Requirement: BLE adapter 兼容 NuttX SMP API
`esp32s3_ble_adapter.c` SHALL 不滥用 `sched_lock()` 和 `sched_unlock()` 的返回值（NuttX 中两者返回 void）。

#### Scenario: SMP 启用时编译通过
- **WHEN** `CONFIG_SMP=y` + `CONFIG_ESP32S3_BLE=y`
- **THEN** `esp32s3_ble_adapter.c` 编译无 `void value not ignored as it ought to be` 错误

#### Scenario: sched_lock 不带返回值
- **WHEN** 检查 `esp_task_create_pinned_to_core` 函数
- **THEN** `sched_lock();` 直接调用，不赋值给 `ret` 变量

#### Scenario: sched_unlock 不带返回值
- **WHEN** 检查 `esp_task_create_pinned_to_core` 函数
- **THEN** `sched_unlock();` 直接调用，不赋值给 `ret` 变量

### Requirement: BLE controller 初始化
ESP32-S3-EYE 启动时 SHALL 自动初始化 BLE controller 并注册 `/dev/bnep0` 网络接口。

#### Scenario: BLE 接口注册
- **WHEN** 启动后执行 `ifconfig`
- **THEN** 输出包含 `bnep0 ... at DOWN` 接口

#### Scenario: BLE MAC 地址正确
- **WHEN** 执行 `bt bnep0 info`
- **THEN** 输出 `Device: bnep0`, `BDAddr: <WiFi MAC + 2>`（与 WiFi MAC 同段，BLE +2）

### Requirement: BLE 扫描功能
板子 SHALL 支持通过 `bt` 命令对环境中的 BLE 设备进行扫描。

#### Scenario: bt scan start 不报错
- **WHEN** 执行 `bt bnep0 scan start`
- **THEN** 命令成功返回，无 "command not found" 或 "ioctl failed"

#### Scenario: bt scan get 返回扫描结果
- **WHEN** 执行 `bt bnep0 scan start; sleep 5; bt bnep0 scan get`
- **THEN** 输出 `Scan result:` 表头 + 至少 1 个设备条目（addr/rssi/type/advertiser data 字段）

#### Scenario: bt scan stop 关闭扫描
- **WHEN** 执行 `bt bnep0 scan stop`
- **THEN** 命令成功返回

### Requirement: BLE 完整 host stack
板子 SHALL 启用 NuttX BLE host stack 完整支持（advertising / scanning / connection / GATT）。

#### Scenario: BLE host stack 配置完整
- **WHEN** 检查 .config
- **THEN** `CONFIG_DRIVERS_BLUETOOTH=y` + `CONFIG_NET_BLUETOOTH=y` + `CONFIG_WIRELESS_BLUETOOTH=y` + `CONFIG_WIRELESS_BLUETOOTH_HOST=y` + `CONFIG_BTSAK=y` + `CONFIG_ALLOW_BSD_COMPONENTS=y` 全部启用

#### Scenario: bt 命令支持完整子命令
- **WHEN** 执行 `bt`（不带参数）
- **THEN** 输出 usage 包含 `info / features / scan / advertise / security / gatt`

### Requirement: BLE 与 WiFi 共存
板子 SHALL 支持 WiFi station 和 BLE 同时工作（SW coexistence）。

#### Scenario: WiFi 和 BLE 同时初始化
- **WHEN** 启动后执行 `ifconfig`
- **THEN** 同时显示 `wlan0 ... at UP` 和 `bnep0 ... at DOWN` 两个接口（wlan0 在初次 wapi 命令后变 UP）

#### Scenario: ESPRESSIF_BLE + ESPRESSIF_WIFI 同时启用
- **WHEN** 检查 sdkconfig.h
- **THEN** `#if defined(CONFIG_ESPRESSIF_BLE) && defined(CONFIG_ESPRESSIF_WIFI)` 触发 `CONFIG_ESP_COEX_SW_COEXIST_ENABLE 1`
