# ESP32-S3 TCM/AMM/UIF Industrial Gateway

## 2026-07-29 Development Status

This revision consolidates the gateway, ThingsCloud, MCP management, AMM cloud
routing, rule-engine, health-monitoring, and UIF recovery work completed during
hardware integration on the ESP32-S3 board.

### Verified on hardware

- ESP32-S3 target with 8 MB PSRAM, SPI Flash, ST7735S LCD, W5500/Wi-Fi, TF and
  MAX3485 RS485 hardware allocation.
- MODBUS RTU polling and device discovery using a physical SHT20 sensor and a
  PC RTU simulator.
- Configurable MODBUS TCP endpoints, discovery, mapping and northbound server.
- AMM mapping capacity of 1000 points with runtime tables allocated in PSRAM.
- TCM semantic profiles, safe unresolved mappings and user-confirmed semantics.
- ThingsCloud gateway/sub-device and gateway-attribute reporting modes.
- SPI Flash offline queue with ordered replay after MQTT reconnection; TF is
  used as overflow storage when present.
- Chinese/English Web UI, MCP management, rule engine, MQTT logs and health
  counters.
- Firmware builds with ESP-IDF 5.3.1 and flashes successfully through COM9.

### MQTT integration status

- ThingsCloud plaintext MQTT currently uses `mqtt://<endpoint>:1883`, MQTT
  3.1.1, `AccessToken` as Username and `ProjectKey` as Password.
- Connection diagnostics reuse the persistent MQTT client; a second temporary
  client is no longer allocated, preventing duplicate sessions and internal
  SRAM exhaustion.
- The firmware records TCP and CONNACK failures separately. `CONNACK 5` is
  reported as an authentication rejection rather than a generic network error.
- MQTT.fx and the gateway must not use the same ThingsCloud device credentials
  simultaneously because the platform permits only one connection per device
  identity.
- Runtime MQTT reconfiguration still requires a production connection-manager
  refactor. The present 20 KB MQTT task stack can be allocated during cold boot,
  but internal SRAM fragmentation can prevent client recreation after a live
  configuration change. The planned implementation uses one long-lived client
  owner, a non-blocking command queue, explicit maintenance mode, and
  error-class-specific exponential backoff.

### Resource and reliability policy

- Time-critical MODBUS tasks, UART buffers and control queues use internal SRAM.
- Large AMM/TCM tables, discovery results and suitable worker stacks use PSRAM.
- Flash stores firmware, Web assets, configuration, AMM snapshots and the UIF
  queue; TF remains optional overflow/history storage.
- Watchdog, reset reason, minimum heap, largest internal block, cache usage and
  online/offline device counts are exposed through `/api/system/status`.
- Local `test_logs/` artifacts are intentionally excluded from Git.

本项目是面向 ESP32-S3 的 MODBUS-MQTT 工业协议网关固件，采用 ESP-IDF 5.3.1 和 FreeRTOS 开发。网关将 MODBUS RTU/TCP 数据转换为固定 TCM 上下文，通过 MQTT 发布，并提供动态 AMM 映射、UIF 离线恢复、Web 配置、LCD 状态菜单、TF 历史记录、离线自动化决策和 MCP 工具接口。当前版本新增 ThingsCloud 云平台适配层，支持以"网关 + 子设备"模型将 TCM 数据聚合上报到 ThingsCloud。

当前版本已经在带 16 MB Flash、8 MB PSRAM 的 ESP32-S3 实物开发板上完成烧录，并使用真实 RS485 温湿度传感器、PC 端 RTU 仿真器和 Modbus TCP 仿真器完成设备发现与寄存器读取测试。

本次工业化功能扩展已再次在实板验证：固件可稳定启动并保持运行，WiFi 获取地址 `192.168.100.22`，Web 首页局域网首次响应约 117 ms，配置 API 返回 HTTP 200，北向 MODBUS TCP Server 的 502 端口可连接并返回标准异常响应。未配置有效 MQTT Broker 时 MQTT 保持停用；未插 TF 卡时系统继续使用 SPI Flash 缓存，这两种状态都不会阻塞网关启动。

## 工业化优先级完成情况

| 优先级 | 已实现能力 |
| --- | --- |
| P1 协议、语义、可靠性 | MODBUS FC01/02/03/04/05/06/15/16；BOOL、16/32/64 位整数、FLOAT32/FLOAT64、BCD16、位域、ASCII、字节序和块读取；TCM 1.1 语义来源/证据/置信度；陌生设备稳定指纹与安全原始映射；AMM 模型版本、对象区冲突检测和一版回滚；配置双槽事务、CRC、轮询重试/退避、任务看门狗复位统计。 |
| P2 时间、消息、网络、安全、控制 | NTP/时区/时间质量；MQTT TLS 证书包、QoS、持久会话、Retain、LWT 和 UIF 有序回放；W5500/WiFi 主备切换；可选 Bearer 鉴权；HTTPS OTA、镜像 SHA-256、双 OTA 分区和启动回滚；自动化回差、保持、冷却、联锁和审计。 |
| P3 北向互操作与部署 | 北向 MODBUS TCP Server（FC01-06/15/16）、REST API、MCP 工具接口、运行健康指标和工业硬件部署检查表。Sparkplug B/OPC UA 定位为可选上位机适配器，不在 MCU 固件中伪装为已完成协议栈。 |

## 研究目标对应关系

| 研究目标 | 固件实现 |
| --- | --- |
| RO1 TCM | `tcm_context` 作为协议与业务之间的固定中间层；保留设备、点位、协议、通道、地址、类型、字节序、原始值、量程换算、质量、网络状态、映射版本和控制约束。 |
| RO2 AMM | 映射可通过 Web 和设备发现动态增删改并持久化到 NVS；已知设备匹配语义模板，陌生设备生成可追踪的 `UNRESOLVED` 原始映射，用户确认或导入模板后升级语义状态；模型版本变化后调度器自动重建轮询计划；支持 RTU/TCP、多通道、数据类型、字节序、比例、偏移、优先级和轮询周期。 |
| RO3 UIF | SPI Flash 为主离线队列，TF 卡为溢出备用；使用单调序列号、QoS 1 PUBACK 确认删除、按序恢复会话；本地规则在 MQTT 离线时仍可执行。 |

## 已实现功能

- MODBUS RTU/TCP 主站：FC01、FC02、FC03、FC04、FC05、FC06、FC15、FC16，RS485 半双工；TCP 最多 8 个运行时端点。
- 北向 MODBUS TCP Server：将 AMM/TCM 最新状态暴露为 coils、discrete inputs、holding/input registers，端口和最大客户端数可在 Web 配置。
- RTU/TCP 设备发现：可选择 RS485 总线或指定 TCP 端点，执行从站扫描、原始寄存器扫描和 AMM 原始映射生成；发现过程严格只读。
- RTU 自适应探测：支持 FC03/FC04、常用工业寄存器入口和 Modbus 异常响应在线判定；发现后按设备实际寄存器区域继续扫描。
- 扫描隔离：设备发现期间暂停常规 AMM 轮询，扫描结束后自动恢复，避免两个任务竞争 RS485 总线。
- 固定 TCM 1.1 JSON 上下文、字段验证、语义来源/版本/证据/置信度、映射版本和掉电安全序列号。
- 动态 AMM：最多 1000 点，PSRAM 运行表、独立 NVS 持久化、运行时增删改、混合协议/通道寻址和动态轮询计划。
- AMM 设备模板：Web 批量导入 RTU/TCP 设备语义，配置数据类型、字节序、比例、偏移、单位、量程和点位名称，将原始寄存器转换为工程值。
- 陌生设备安全映射：根据协议、通道、从站、设备元数据、在线探针和寄存器签名生成稳定指纹；建立只读 `DISCOVERY / UNRESOLVED` 原始 TCM 点，保留证据和置信度，不猜测工程量、单位、缩放、符号位或多寄存器布局。
- 语义确认闭环：映射表显示语义状态和置信度；用户编辑保存后标记为 `USER / VERIFIED`，导入验证模板可批量升级；同一设备采用事务替换，重复应用不会累加点位。
- AMM 运维：映射表支持清空、事务导入、一版回滚和对象区冲突检测，并显示正常、等待首次轮询、读取失败和数据过期状态。
- 公平轮询：最久未轮询点优先，优先级用于同等条件排序，避免前部 RTU 点或高优先级点长期阻塞后部 RTU/TCP 点。
- MQTT TLS/QoS/持久会话/Retain/LWT 上行和受控下行；所有 MQTT、Web、MCP、北向 MODBUS TCP 和自动化写入共用 AMM 权限/量程边界。
- UIF 离线恢复：13 MB SPI Flash FAT 队列优先，TF 卡溢出，PUBACK 后删除。
- TF 历史：按序列分片保存 JSONL；空间不足时删除最早历史文件。
- 自动化规则：Web 配置阈值、回差、保持、冷却、点位联锁、写点或 MQTT 告警动作；规则保存在 NVS，并保留最近 64 条执行审计。
- Web 配置：中文/English 一键切换；默认不启用认证，可选 Bearer Token；配置 AP 与已有网络接口均可访问；设备结果和通信日志使用低内存流式传输。
- LCD 状态菜单：网络、MQTT、TCM/AMM/UIF、TF 和配置 AP 状态轮播。
- W5500 以太网优先，链路断开后自动切换 Wi-Fi STA，同时保留配置 AP，并记录出口与切换次数。
- NTP 时间同步、时间质量标记、运行健康指标、HTTPS OTA、镜像哈希校验和失败回滚。
- MCP JSON-RPC 工具入口：`POST /mcp`。
- 运行指标：轮询、TCM 验证、MQTT、缓存、回放、命令和数据丢失计数。
- ThingsCloud 云平台适配：统一云数据对象（`cloud_adapter`）+ ThingsCloud 官方 MQTT 网关协议（`attributes`、`gateway/attributes`、`gateway/connect|disconnect`、`gateway/command/send|reply` 等 Topic）；支持子设备地址编码/解析、子设备在线/离线状态机与重连后批量补报、按周期聚合的属性上报、网关模式属性键 `p{port}_s{slave}_{point}` 及键名合法性校验、下行属性推送与命令回调、凭据脱敏日志。
- ThingsCloud 可靠性：支持 100 台子设备和全部 1000 个 AMM 点位的严格通道下行匹配；离线 TCM 记录在重连后重新经过当前云平台上报模式；聚合数据按实际 JSON 字节拆包，限流或缓冲拥塞时转入 UIF 持久化缓存；Web 页面以中英文显示在线设备、待发送点位、离线缓存、限流次数和最近上报状态。
- AMM 动态云路由：自定义 MQTT 点位支持 `AUTO`/`CUSTOM` Topic 策略；`AUTO` 在发布时根据最新 Topic 前缀、网关、设备和点位动态计算，修改 MQTT 配置后无需批量改写映射；`CUSTOM` 保留用户指定的完整 Topic。ThingsCloud 子设备模式使用 `point_id`，网关模式使用自定义或自动生成的 `gateway_property_key`，Web 映射表会随平台和模式切换显示当前有效路由。
- 发布管线稳定性：`publish_task` 采用 PSRAM 64 KB 静态栈（`xTaskCreateStatic`），规避内部 DRAM 紧张导致的任务创建失败和深层 esp-mqtt 调用链栈溢出；离线缓存写入解耦到独立 `cache_writer_task` 异步执行，SPI Flash 慢写不再阻塞实时发布路径；MQTT 断线重连退避 10 s。

设备发现表优先分配到 PSRAM：检测到外部 PSRAM 时单次最多保留 100 台设备、每台 32 个原始寄存器字；PSRAM 不可用时自动降级为 8 台设备。扫描任务栈也优先放入 PSRAM，并在不可用时回退内部 RAM，避免大量活动映射造成内部 RAM 碎片后无法启动扫描。AMM 和 TCM 最新状态池最多保留 1000 个活动点位；无 PSRAM 时 AMM 自动降级为 64 点。自动化规则最多 16 条。设备发现容量和 AMM 活动映射容量相互独立。

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

RS485 需要 3.3 V 兼容收发器。当前 MAX3485 接线如下：

| MAX3485 | ESP32-S3 |
| --- | --- |
| DI / TX | GPIO39 |
| RO / RX | GPIO40 |
| DE + RE | GPIO41 |
| VCC | 3.3 V |
| GND | GND |

MAX3485 的 A/B/GND 分别连接到传感器或 USB-RS485 模块的 A/B/GND。若无响应，可先交换 A/B 排查不同厂商的标记差异。总线应按现场拓扑配置终端电阻、偏置和隔离保护。

## Flash 分区

目标为开发板实测的 16 MB Flash：

| 分区 | 偏移 | 大小 | 用途 |
| --- | --- | --- | --- |
| `nvs` | `0x9000` | 152 KB | 运行配置、规则、TCM 序列保留 |
| `phy_init` | `0x2F000` | 4 KB | PHY 数据 |
| `otadata` | `0x30000` | 8 KB | OTA 启动选择与回滚状态 |
| `ota_0` | `0x40000` | `0x1D0000` | 当前/候选固件 A |
| `ota_1` | `0x210000` | `0x1D0000` | 当前/候选固件 B |
| `amm_nvs` | `0x3E0000` | 1 MB | 1000 点 AMM 映射与回滚快照 |
| `cache` | `0x4E0000` | `0xB20000` | SPI Flash UIF 离线队列 |

当前构建固件约 1.47 MiB，单个 OTA 应用分区仍有约 19% 空间。

从旧的单应用分区版本首次升级到该双 OTA 分区表时，`amm_nvs` 会按新布局初始化，原映射需要重新导入或通过设备发现重新应用一次。之后的普通 OTA 更新会保留 NVS、AMM 映射和离线缓存。

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

烧录完成后，开发板会从 NVS 恢复网络、MQTT、MODBUS、AMM 和自动化配置。串口监视器默认使用 `115200` 波特率。

## 首次配置

1. 上电后网关创建开放配置 AP，名称类似 `MCP-Gateway-XXXX`。
2. 连接 AP 后访问 `http://192.168.4.1/`。
3. 配置 Wi-Fi、MQTT、MODBUS RTU/TCP endpoint 和 AMM 映射。
4. 配置保存到 NVS。TCP 端点的名称、IP、端口和超时可随时修改；设备发现启动 TCP 扫描前也会自动保存当前端点配置。
5. Web 当前按课题第一版要求不启用登录认证，不应直接暴露到不可信网络。

W5500 获得 DHCP 地址后优先作为默认路由；Wi-Fi STA 可作为备用。配置 AP 始终用于本地维护。

## MODBUS RTU 设备发现

默认 RTU 参数为 `9600 8N1`，响应超时为 `1000 ms`。Web 的“设备发现”页面可以设置从站范围、寄存器范围和 FC03/FC04，然后在后台执行扫描。

扫描分为两个阶段：

1. 使用用户起始地址、`0/1` 和常见工业寄存器入口进行低成本从站在线探测；静默或不可达的从站立即跳过。
2. 仅对第一阶段确认在线、且进入第二阶段前复检仍在线的设备扫描寄存器，并保留原始值供用户建立 AMM 语义映射。

发起扫描：

```bash
curl -X POST http://<gateway-ip>/api/discover/scan \
  -H "Content-Type: application/json" \
  -d '{"slave_start":1,"slave_end":10,"reg_start":1,"reg_end":16,"function_codes":[3,4],"max_empty_gap":8}'
```

查询状态和结果：

```bash
curl http://<gateway-ip>/api/discover/status
curl http://<gateway-ip>/api/discover/devices
```

MODBUS RTU 没有统一的总线枚举和寄存器描述标准。自适应探测可以发现返回数据或合法异常响应的设备，但非标准寄存器布局仍应由用户指定扫描范围，或在发现后通过 AMM 配置数据类型、字节序、比例、偏移、单位和点位语义。

## AMM 设备模板与工程值

Modbus 报文本身不提供点位名称、数据类型、字节序、单位或缩放公式。设备发现只能可靠保留原始寄存器，不能仅凭寄存器值判断其工业语义。Web“映射表”中的“导入设备模板”用于补齐这些信息：

1. 选择设备模板 JSON。导入器根据模板的 `source_protocol` 或设备 `protocols` 自动判断 RTU/TCP；TCP 模板使用已启用的运行时端点。
2. 可以导入仿真器导出的有效 JSON，也可直接使用 `profiles/` 中的独立模板。
3. 网关按协议、通道和 Slave/Unit ID 批量替换对应设备的原始映射，并一次写入 `amm_nvs`。
4. 调度器按照模板读取一个或多个寄存器，完成有符号、浮点、字节序、比例和偏移换算。

工程值计算公式为：

```text
engineering_value = decoded_raw_value * scale_factor + offset
```

模板导入后，映射表同时显示解码原值、工程值和单位。没有模板的自动发现点位使用 `raw_fc<功能码>_<地址>` 标识，保持空单位，工程值显示为 `--`，避免网关把原始字误当成已确认工程量。

设备发现统计的是 16 位原始寄存器字，不等同于设备模板中的工程点数。例如一个 `FLOAT32` 工程点占用两个连续的 16 位字，原始扫描会先显示为两条记录；RTU 与 TCP 扫描结果也按不同通道分别计数。“应用 TCM 语义映射”先根据协议、通道、从站 ID、设备标识和已发现地址匹配已验证设备模板，再将原始字组合为带数据类型、字节序、比例和单位的工程点。未匹配的陌生设备会生成稳定指纹和只读原始 TCM 点，标记为 `DISCOVERY / UNRESOLVED`，保留响应证据和置信度，但不猜测工程量、单位、比例、符号位或多寄存器布局。用户在 Web 中编辑保存后升级为 `USER / VERIFIED`；导入已验证模板也可批量完成语义升级。相同协议、通道和从站的点位采用事务替换，重复应用不会累加重复映射。

当前固件内置本课题实验设备语义库：SHT20 2 点以及 PC 仿真器从站 2-21 的 144 点。用户导入的新设备模板仍由 AMM 持久化；后续可继续扩展为可学习、可版本化的动态语义库。

1000 点是 AMM 的运行容量，不代表任意现场都能在 100 ms 内刷新 1000 点。RTU 总线必须串行收发，实际周期受波特率、设备数量、响应延迟和寄存器分布影响。要缩短周期，应优先合并同一设备的连续寄存器块、提高允许的 RTU 波特率、并行调度不同 TCP 端点，并为快速、常规和慢速点配置不同轮询周期。

## MODBUS TCP 配置与设备发现

TCP 地址不固定在固件中。进入 Web 的“MODBUS 设置”，可以新增、编辑或删除最多 8 个端点：

- 端点名称
- 主机名或 IPv4 地址
- TCP 端口，工业设备通常使用 `502`，仿真器可使用 `5020` 等非特权端口
- 响应超时

保存后配置写入 NVS，重启仍然有效。进入“设备发现”，将扫描类型切换为 `Modbus TCP`，选择一个已启用端点，再设置 Unit ID、寄存器范围和 FC03/FC04。

TCP 扫描请求示例：

```bash
curl -X POST http://<gateway-ip>/api/discover/scan \
  -H "Content-Type: application/json" \
  -d '{"source_protocol":"TCP","channel_id":1,"slave_start":1,"slave_end":20,"reg_start":1,"reg_end":16,"function_codes":[3,4],"max_empty_gap":8}'
```

对于 RTU，目标从站返回合法 Modbus 异常可以证明该地址上存在设备；对于 TCP，服务器或网关可能对不存在的 Unit ID 统一返回异常 `0x0B`。因此 TCP 发现只有成功读取到寄存器数据才判定设备在线，避免把服务器异常响应误报为设备。

## TCM 1.1 固定格式

主要字段包括：

```json
{
  "tcm_version": "1.1",
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
  "semantic_source": "profile",
  "semantic_status": "verified",
  "semantic_profile_id": "sht20_rtu",
  "semantic_profile_version": 1,
  "semantic_confidence": 100,
  "semantic_evidence": "vendor profile match",
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
    "min": 0,
    "max": 120
  }
}
```

TCM JSON 使用可机读的 UCUM 风格单位 `degC`；中文 Web 显示层将其格式化为 `℃`。二者数值含义一致，避免把界面符号混入跨系统交换格式。

## MCP 工具接口

固件提供适合局域网/边缘侧使用的 MCP Streamable HTTP JSON-RPC 子集：

- `list_points`
- `read_point`
- `write_point`
- `discover_modbus_devices`
- `get_gateway_config`
- `get_cache_status`
- `configure_rule_from_natural_language`

`configure_rule_from_natural_language` is a two-stage safety workflow. The MCP
client converts the user's natural-language instruction into the tool's
structured rule schema and first calls it with `confirmed=false`. The gateway
validates the AMM source point, optional interlock, writable target, value
range, hysteresis, and cooldown, then returns a normalized preview without
changing configuration. Only after the user explicitly confirms that preview
may the client repeat the call with `confirmed=true`. Applying the rule also
requires `mcp_write_enabled`.

MCP access is denied by default. Before any MCP client can connect, configure
and enable the gateway Bearer Token under System Security. Every request to
`POST /mcp` must send `Authorization: Bearer <token>`. Tokens are stored only
as SHA-256 digests. Five consecutive authentication failures trigger a
30-second lockout, and the MCP endpoint does not advertise unrestricted CORS.
Keep `mcp_write_enabled` disabled for read-only clients; enable it only for
trusted operators that must apply rules or write industrial points.

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
| `GET/POST /api/mappings`、`PUT/DELETE /api/mappings/:index` | AMM 动态映射 |
| `DELETE /api/mappings` | 一次清空全部 AMM 映射和运行点位状态 |
| `POST /api/mappings/rollback` | 回滚到上一个 AMM 模型快照 |
| `POST /api/mappings/import` | 批量导入设备语义模板并持久化 |
| `POST /api/discover/scan` | 后台设备/寄存器扫描 |
| `GET /api/discover/status` | 扫描进度和汇总 |
| `GET /api/discover/devices` | 流式返回设备、探测入口和原始寄存器 |
| `POST /api/discover/apply` | 将发现结果应用到 AMM |
| `GET/DELETE /api/modbus/logs` | 查询或清空 RTU TX/RX 原始通信日志 |
| `GET/POST/DELETE /api/automation/rules` | 离线自动化规则 |
| `GET /api/automation/audit` | 自动化执行与联锁审计 |
| `GET/PUT /api/time/config` | NTP、时区和同步周期 |
| `GET/PUT /api/security/config` | 可选 Web 鉴权和 OTA 安全策略 |
| `GET /api/ota/status`、`POST /api/ota/start` | HTTPS OTA 状态与启动 |
| `GET /api/system/status` | 运行状态与研究指标 |

## 目录结构

```text
main/
  board/       W5500、LCD、TF、共享复位与 GPIO 状态
  network/     W5500 + Wi-Fi STA/AP 网络管理
  config/      NVS 运行配置
  modbus/      RTU/TCP 访问与设备发现
  semantic/    陌生设备指纹与安全原始语义映射
  tcm/         固定上下文与最新状态池
  amm/         动态映射模型
  scheduler/   动态轮询、TCM 管线和回放调度
  mqtt_comm/   MQTT 发布、订阅和 PUBACK 跟踪
  storage/     Flash 离线队列与 TF 历史
  uif/         离线缓存和有序会话恢复
  automation/  离线规则引擎与 Web API
  services/    统一受控写入服务
  cloud_adapter/  统一云数据对象定义（平台无关）
  thingscloud/    ThingsCloud MQTT 网关协议适配（Topic、子设备、命令）
  mcp/         MCP JSON-RPC 工具接口
  web/         HTTP REST 服务
  eval/        研究指标与日志
web/
  web_config.html  中英文 Web 管理页面
profiles/
  sht20_rtu.json                 SHT20 RTU 温湿度语义模板
  lab_rtu_146_points.json        20 台 PC 仿真设备与 SHT20 的 146 点实验模板
```

## 当前验证状态

- ESP32-S3 QFN56 rev0.2、16 MB Flash、ESP-IDF 5.3.1：构建和实物烧录通过。
- 8 MB PSRAM：设备发现表按 100 台容量分配，AMM 映射表和 TCM 状态池均按 1000 点容量分配；100 台同时在线的完整压力测试仍待执行。
- LCD ST7735S：初始化、状态页面和字体显示通过。
- Wi-Fi STA/配置 AP：Web 页面、REST API 和中英文切换通过。
- W5500：SPI 驱动初始化和 DHCP 网络接入流程通过。
- RS485/MAX3485：GPIO39/40/41 半双工收发通过。
- 真实 SHT20 传感器：Slave 1、FC04、寄存器 1/2 读取通过；最近一次模板换算实测得到约 `37.0 degC` 和 `47.0 %RH`，语义状态均为 `VERIFIED 100%`。
- 两阶段在线门控：Slave 1 在线探测、扫描前复检和寄存器扫描通过；扫描任务栈使用 PSRAM 后，在 146 个活动映射运行期间仍可正常启动设备发现。
- 陌生设备闭环：仅扫描 SHT20 的一个寄存器使其无法满足已知模板双点指纹，网关生成指纹 `bb7e8024`、1 个只读 `UNRESOLVED` 原始点；连续应用两次后总点位始终为 145，无重复累加。恢复完整双寄存器扫描后重新匹配验证模板，总点位恢复为 146。
- PC RTU 仿真器：20 台设备、144 个语义点位读取通过；与真实 SHT20 的 2 点合并后，146 个 RTU 工程点全部轮询正常。
- TCP 端点动态配置：Web 新增、编辑、取消编辑、NVS 保存和重启恢复通过；当前测试端点为 `192.168.100.9:5020`，该地址仅为运行时测试配置。
- PC TCP 仿真器：扫描 Unit ID 1-20，正确发现启用 TCP 的 18、20 两台设备，共 14 个寄存器，最终错误码为 0；未启用 TCP 的 RTU 设备未被误报。
- 设备结果接口：13,776 字节约 0.52 秒返回；64 条通信日志 15,224 字节约 0.47 秒返回。
- 146 个 RTU 工程映射掉电重启恢复通过；运行时容量接口返回 AMM 1000 点、TCM 状态池 1000 点。
- 公平调度修复后，146 个映射全部完成首轮轮询且状态为正常，没有点位长期停留在等待状态。
- 全量清空事务通过：映射清空后接口返回空表，随后从实验模板批量恢复 146 点成功。
- 持续轮询 1045 次成功、0 次失败，空闲堆约 7.36 MB；MQTT 保持默认占位配置时不写离线缓存，缓存记录和数据丢失计数均为 0。
- ThingsCloud 实测：启用 ThingsCloud 平台后 MQTT 约 13 s 完成连接，90 秒窗口内成功上报 146 条聚合属性、发布失败 0 次，全程无栈溢出、无重启；数据丢失仅发生在启动初期尚未联网的离线窗口。
- 发布管线回归：`publish_task` PSRAM 64 KB 栈修复了此前 12 KB 栈溢出崩溃循环与 32 KB 内部 RAM 分配失败两类故障；离线缓存写入改为 `cache_writer_task` 异步处理后，上下文队列不再溢出。
- 已知遗留：MQTT 离线窗口内 SPI Flash 离线缓存写入（`offline_store_put`）偶发失败，在线上报不受影响，待后续排查缓存分区。
- TF 卡未插入时可以正常降级运行；TF 实卡写入、满盘覆盖和长时间耐久测试仍需后续验证。

以上耗时来自当前开发环境的一次局域网测试，不作为所有网络条件下的硬实时保证。
