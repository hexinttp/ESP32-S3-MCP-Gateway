# ESP32-S3 TCM/AMM/UIF Industrial Gateway

本项目是面向 ESP32-S3 的 MODBUS-MQTT 工业协议网关固件，采用 ESP-IDF 5.3.1 开发。网关将 MODBUS RTU/TCP 数据转换为固定 TCM 上下文，通过 MQTT 发布，并提供动态 AMM 映射、UIF 离线恢复、Web 配置、LCD 状态菜单、TF 历史记录、离线自动化决策和 MCP 工具接口。

## 研究目标对应关系

| 研究目标 | 固件实现 |
| --- | --- |
| RO1 TCM | `tcm_context` 作为协议与业务之间的固定中间层；保留设备、点位、协议、通道、地址、类型、字节序、原始值、量程换算、质量、网络状态、映射版本和控制约束。 |
| RO2 AMM | 映射可通过 Web 和设备发现动态增删改并持久化到 NVS；模型版本变化后调度器自动重建轮询计划；支持 RTU/TCP、多通道、数据类型、字节序、比例、偏移、优先级和轮询周期。 |
| RO3 UIF | SPI Flash 为主离线队列，TF 卡为溢出备用；使用单调序列号、QoS 1 PUBACK 确认删除、按序恢复会话；本地规则在 MQTT 离线时仍可执行。 |

## 已实现功能

- MODBUS RTU 主站：FC03、FC04、FC06、FC16，RS485 半双工。
- MODBUS TCP 主站：多个可配置 TCP endpoint，支持 FC03、FC04、FC06、FC16。
- 后台设备发现：从站扫描、寄存器扫描、语义推断和 AMM 映射生成；发现过程严格只读。
- 固定 TCM 1.0 JSON 上下文、字段验证、映射版本和掉电安全序列号。
- 动态 AMM：NVS 持久化、运行时增删改、混合协议/通道寻址和动态轮询计划。
- MQTT 上行和受控下行；所有 MQTT、Web、MCP 和自动化写入共用 AMM 权限/量程边界。
- UIF 离线恢复：14 MB SPI Flash FAT 队列优先，TF 卡溢出，PUBACK 后删除。
- TF 历史：按序列分片保存 JSONL；空间不足时删除最早历史文件。
- 自动化规则：Web 配置条件、保持时间、冷却时间、写点或 MQTT 告警动作；规则保存在 NVS。
- Web 配置：中文/English 一键切换，无登录认证；配置 AP 与已有网络接口均可访问。
- LCD 状态菜单：网络、MQTT、TCM/AMM/UIF、TF 和配置 AP 状态轮播。
- W5500 以太网优先，Wi-Fi STA 备用，同时保留配置 AP。
- MCP JSON-RPC 工具入口：`POST /mcp`。
- 运行指标：轮询、TCM 验证、MQTT、缓存、回放、命令和数据丢失计数。

为给 Wi-Fi、MQTT 和 HTTP 运行时堆保留空间，当前单次发现结果最多保留 8 台设备、每台 32 个寄存器；AMM 最多保留 64 个活动点位，自动化规则最多 16 条。更大总线可分段扫描后逐批应用映射。

## 六层系统架构

1. **L1 MODBUS Access**：RS485 RTU、MODBUS TCP、设备发现和寄存器编解码。
2. **L2 Protocol Adaptation / TCM**：原始协议数据标准化并构造固定 TCM 中间对象。
3. **L3 Context Modeling**：语义字段验证、状态池、序列号和 JSON 序列化。
4. **L4 Scheduling / AMM**：动态映射、轮询优先级、规则执行和任务编排。
5. **L5 Persistence / Recovery / UIF**：Flash/TF 离线队列、MQTT 确认和有序回放。
6. **L6 Evaluation / Management**：Web、LCD、MCP 工具边界、日志和指标。

代码模块比论文层次更细，模块不是新的论文层。TCM、AMM、UIF 分别贯穿对应的研究层次。

## 硬件 GPIO 规划

第一版固定启用 LCD、W5500、TF 卡和 RS485。

| 外设 | GPIO |
| --- | --- |
| W5500 SPI | CS=10, MOSI=11, SCLK=12, MISO=13, INT=14, RST=15 |
| ST7735S LCD | RST=15, DC=16, MOSI=17, SCLK=18, CS=21 |
| TF / SDMMC 4-bit | D2=33, D3=34, CMD=35, CLK=36, D0=37, D1=38 |
| RS485 / UART1 | TX=39, RX=40, DE/RE=41 |
| UART0 | TX=43, RX=44 |
| USB | GPIO19, GPIO20 |

GPIO15 是 W5500 与 LCD 的共享复位信号，由 `board` 模块统一控制。GPIO33-38 已由 TF 卡占用，不能再分配给 RS485。

RS485 需要 3.3 V 兼容收发器。A/B 总线应按现场拓扑配置终端电阻、偏置和隔离保护。

## Flash 分区

目标为开发板实测的 16 MB Flash：

| 分区 | 偏移 | 大小 | 用途 |
| --- | --- | --- | --- |
| `nvs` | `0x9000` | 152 KB | 运行配置、AMM、规则、TCM 序列保留 |
| `phy_init` | `0x2F000` | 4 KB | PHY 数据 |
| `factory` | `0x30000` | `0x1D0000` | 固件 |
| `cache` | `0x200000` | 14 MB | SPI Flash UIF 离线队列 |

当前构建固件约 1.28 MB，应用分区仍有约 28% 空间。

## 构建

本机 ESP-IDF 位于 `D:\Espressif`，工程使用的版本为 5.3.1。

```powershell
$env:IDF_PATH='D:\Espressif\frameworks\esp-idf-v5.3.1'
$env:IDF_TOOLS_PATH='D:\Espressif'
$env:PATH='D:\Espressif\tools\idf-git\2.44.0\cmd;' + $env:PATH
. 'D:\Espressif\frameworks\esp-idf-v5.3.1\export.ps1'
idf.py set-target esp32s3
idf.py build
```

在 Windows 上建议限制 Ninja 并行度，避免同时创建过多编译进程：

```powershell
ninja -C build -j 4
```

生成文件：

- `build/esp32s3_mcp_gateway.bin`
- `build/bootloader/bootloader.bin`
- `build/partition_table/partition-table.bin`

## 烧录与监视

将 `COMx` 替换为开发板端口：

```powershell
idf.py -p COMx flash monitor
```

也可以只烧录：

```powershell
idf.py -p COMx flash
```

尚未连接实物开发板时，编译成功只证明软件构建通过；W5500、LCD、SDMMC、RS485 电气时序和实际显示方向仍需上板验证。

## 首次配置

1. 上电后网关创建开放配置 AP，名称类似 `MCP-Gateway-XXXX`。
2. 连接 AP 后访问 `http://192.168.4.1/`。
3. 配置 Wi-Fi、MQTT、MODBUS RTU/TCP endpoint 和 AMM 映射。
4. 配置保存到 NVS。网络、MQTT 和串口核心参数修改后重启生效。
5. Web 当前按课题第一版要求不启用登录认证，不应直接暴露到不可信网络。

W5500 获得 DHCP 地址后优先作为默认路由；Wi-Fi STA 可作为备用。配置 AP 始终用于本地维护。

## TCM 1.0 固定格式

主要字段包括：

```json
{
  "tcm_version": "1.0",
  "gateway_id": "esp32s3_gateway_01",
  "context_id": 1,
  "sequence_id": 1,
  "mapping_version": 5,
  "device_id": "plc_line1_01",
  "point_id": "motor_temp_01",
  "source_protocol": "MODBUS_RTU",
  "channel_id": 0,
  "slave_id": 1,
  "function_code": 3,
  "register_address": 40001,
  "data_type": "float32",
  "byte_order": "ABCD",
  "measurement_name": "Motor temperature",
  "unit": "degC",
  "raw_value": 72.5,
  "scale_factor": 1.0,
  "offset": 0.0,
  "value": 72.5,
  "timestamp_ms": 10000,
  "quality_state": "good",
  "network_state": "online",
  "operation_type": "read_publish",
  "control_constraint": {
    "writable": false,
    "valid_range_min": 0,
    "valid_range_max": 120
  }
}
```

## MCP 工具接口

固件提供适合局域网/边缘侧使用的 MCP Streamable HTTP JSON-RPC 子集：

- `list_points`
- `read_point`
- `write_point`
- `discover_modbus_devices`
- `get_gateway_config`
- `get_cache_status`

入口：`POST http://<gateway-ip>/mcp`

工具列表请求：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}
```

读取点位：

```json
{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"read_point","arguments":{"device_id":"plc_line1_01","point_id":"motor_temp_01"}}}
```

`write_point` 默认关闭。必须在网关配置中启用 `mcp_write_enabled`，并且目标 AMM 条目为可写、值在安全范围内，才会执行 MODBUS 写入。对于需要完整 MCP 会话、鉴权、审计或多客户端治理的部署，建议由边缘计算机运行正式 MCP Server，再调用本固件受控接口。

## 关键 REST API

| API | 用途 |
| --- | --- |
| `GET/PUT /api/gateway/config` | 网关 ID、语言、硬件开关、MCP 写授权 |
| `GET/PUT /api/wifi/config` | Wi-Fi 配置 |
| `GET/PUT /api/mqtt/config` | MQTT 配置 |
| `GET/PUT /api/modbus/config` | RTU 和 TCP endpoint 配置 |
| `GET/POST/PUT/DELETE /api/mappings` | AMM 动态映射 |
| `POST /api/discover/scan` | 后台设备/寄存器扫描 |
| `POST /api/discover/apply` | 将发现结果应用到 AMM |
| `GET/POST/DELETE /api/automation/rules` | 离线自动化规则 |
| `GET /api/system/status` | 运行状态与研究指标 |

## 目录结构

```text
main/
  board/       W5500、LCD、TF、共享复位与 GPIO 状态
  network/     W5500 + Wi-Fi STA/AP 网络管理
  config/      NVS 运行配置
  modbus/      RTU/TCP 访问与设备发现
  tcm/         固定上下文与最新状态池
  amm/         动态映射模型
  scheduler/   动态轮询、TCM 管线和回放调度
  mqtt_comm/   MQTT 发布、订阅和 PUBACK 跟踪
  storage/     Flash 离线队列与 TF 历史
  uif/         离线缓存和有序会话恢复
  automation/  离线规则引擎与 Web API
  services/    统一受控写入服务
  mcp/         MCP JSON-RPC 工具接口
  web/         HTTP REST 服务
  eval/        研究指标与日志
web/
  web_config.html  中英文 Web 管理页面
```

## 当前验证状态

- ESP-IDF 5.3.1、目标 `esp32s3`：构建通过。
- Web 内嵌 JavaScript：语法检查通过。
- 固件尺寸与 16 MB 自定义分区：检查通过。
- 实物板卡烧录与 RS485/W5500/LCD/TF 联调：待连接开发板后执行。
