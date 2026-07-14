"""
ESP32-S3 MCP-Adapted MODBUS-MQTT Gateway — PC Simulation
=========================================================
Faithful Python simulation of the complete gateway firmware.
Implements all 7 architecture layers and 5 test scenarios.

Run:
  python sim_gateway.py                    (pure simulation, no real MQTT)
  python sim_gateway.py --mqtt-broker uri  (connect to real MQTT broker)

MQTT Broker Configuration:
  --mqtt-broker   MQTT broker URI (e.g., mqtt://broker.example.com:1883)
  --mqtt-client   Client ID
  --mqtt-username MQTT username (optional)
  --mqtt-password MQTT password (optional)
"""

import json
import time
import random
import copy
import argparse
from dataclasses import dataclass, field, asdict
from enum import IntEnum
from typing import Optional

try:
    import paho.mqtt.client as mqtt_client
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

# ================================================================
# ENUMERATIONS (matching tcm_context.h)
# ================================================================
class QualityState(IntEnum):
    GOOD = 0; STALE = 1; INVALID = 2

class NetworkState(IntEnum):
    ONLINE = 0; DELAYED = 1; OFFLINE = 2; REPLAYED = 3

class OperationType(IntEnum):
    READ_PUBLISH = 0; SUBSCRIBE = 1; WRITE = 2; REPLAY = 3; READ_ONLY = 4

class SourceProtocol(IntEnum):
    MODBUS_RTU = 0; MODBUS_TCP = 1

class DataType(IntEnum):
    INT16 = 0; UINT16 = 1; FLOAT32 = 2; INT32 = 3; UINT32 = 4

# ================================================================
# DATA STRUCTURES (matching tcm_context.h / amm_mapping.h)
# ================================================================
@dataclass
class ControlConstraint:
    writable: bool = False
    valid_range_min: float = -3.4e38
    valid_range_max: float = 3.4e38

@dataclass
class TcmContext:
    """TCM Context Object — 16 mandatory fields + metadata"""
    context_id: int = 0
    device_id: str = ""
    point_id: str = ""
    source_protocol: SourceProtocol = SourceProtocol.MODBUS_RTU
    slave_id: int = 0
    function_code: int = 0
    register_address: int = 0
    data_type: DataType = DataType.FLOAT32
    measurement_name: str = ""
    unit: str = ""
    value: float = 0.0
    timestamp_ms: int = 0
    quality_state: QualityState = QualityState.GOOD
    network_state: NetworkState = NetworkState.ONLINE
    operation_type: OperationType = OperationType.READ_PUBLISH
    control_constraint: ControlConstraint = field(default_factory=ControlConstraint)
    # Metadata
    sequence_id: int = 0
    validated: bool = False

@dataclass
class AmmMappingEntry:
    slave_id: int = 0
    function_code: int = 3
    register_address: int = 0
    data_type: DataType = DataType.FLOAT32
    scale_factor: float = 1.0
    device_id: str = ""
    point_id: str = ""
    measurement_name: str = ""
    unit: str = ""
    mqtt_topic: str = ""
    constraint: ControlConstraint = field(default_factory=ControlConstraint)
    active: bool = True

@dataclass
class AmmValidationResult:
    accepted: bool = False
    reject_reason: str = ""
    matched_entry: Optional[AmmMappingEntry] = None

@dataclass
class UifCacheEntry:
    json_data: str = ""
    sequence_id: int = 0
    timestamp_ms: int = 0
    pending: bool = True

@dataclass
class EvalMetrics:
    total_polls: int = 0
    successful_polls: int = 0
    failed_polls: int = 0
    contexts_created: int = 0
    contexts_validated: int = 0
    contexts_rejected: int = 0
    mqtt_published: int = 0
    mqtt_failed: int = 0
    cached_records: int = 0
    replayed_records: int = 0
    data_loss_count: int = 0
    commands_received: int = 0
    commands_accepted: int = 0
    commands_rejected: int = 0

# ================================================================
# LAYER 1: MODBUS ACCESS (simulated devices)
# ================================================================
class ModbusAccessLayer:
    """Simulates MODBUS RTU/TCP master with virtual slave devices."""

    SIMULATED_DEVICES = {
        (1, 40001): {"base": 72.5, "variance": 1.5, "type": DataType.FLOAT32, "name": "motor_temp"},
        (1, 40003): {"base": 4.2, "variance": 0.3, "type": DataType.FLOAT32, "name": "pressure"},
        (2, 40001): {"base": 1500, "variance": 50, "type": DataType.UINT16, "name": "speed"},
        (2, 40003): {"base": 25, "variance": 3, "type": DataType.INT16, "name": "current"},
    }

    def __init__(self):
        self._write_log = []

    def read_register(self, slave_id: int, func_code: int, reg_addr: int) -> tuple:
        """Returns (raw_value: float, quality: QualityState)"""
        key = (slave_id, reg_addr)
        if key in self.SIMULATED_DEVICES:
            dev = self.SIMULATED_DEVICES[key]
            value = dev["base"] + random.uniform(-dev["variance"], dev["variance"])
            return round(value, 2), QualityState.GOOD
        return 0.0, QualityState.INVALID

    def write_register(self, slave_id: int, reg_addr: int, value: float) -> bool:
        self._write_log.append({"slave": slave_id, "reg": reg_addr, "value": value, "time": _now_ms()})
        return True

# ================================================================
# LAYER 2: TCM CONTEXT MODELING
# ================================================================
class TcmLayer:
    """Builds, validates, and serializes TCM context objects."""

    _context_id_counter = 0
    _sequence_counter = 0

    @classmethod
    def reset(cls):
        cls._context_id_counter = 0
        cls._sequence_counter = 0

    @classmethod
    def build_context(cls, slave_id, func_code, reg_addr, raw_value, quality, net_state):
        cls._context_id_counter += 1
        cls._sequence_counter += 1
        ctx = TcmContext(
            context_id=cls._context_id_counter,
            slave_id=slave_id,
            function_code=func_code,
            register_address=reg_addr,
            value=raw_value,
            quality_state=quality,
            network_state=net_state,
            operation_type=OperationType.READ_PUBLISH,
            timestamp_ms=_now_ms(),
            sequence_id=cls._sequence_counter,
            data_type=DataType.FLOAT32,
        )
        return ctx

    @staticmethod
    def validate(ctx: TcmContext) -> tuple:
        """Returns (passed: bool, fail_reason: str)"""
        reasons = []
        if ctx.context_id <= 0: reasons.append("context_id invalid")
        if not ctx.device_id: reasons.append("device_id empty")
        if not ctx.point_id: reasons.append("point_id empty")
        if not (1 <= ctx.slave_id <= 247): reasons.append("slave_id out of range")
        if ctx.function_code not in (3, 4, 6, 16): reasons.append("invalid function_code")
        if ctx.register_address == 0: reasons.append("register_address zero")
        if not ctx.measurement_name: reasons.append("measurement_name empty")
        if not ctx.unit: reasons.append("unit empty")
        if ctx.timestamp_ms <= 0: reasons.append("timestamp invalid")
        if ctx.control_constraint.writable:
            if not (ctx.control_constraint.valid_range_min <= ctx.value <= ctx.control_constraint.valid_range_max):
                reasons.append("value out of valid_range")
        passed = len(reasons) == 0
        ctx.validated = passed
        return passed, "; ".join(reasons) if reasons else ""

    @staticmethod
    def serialize_json(ctx: TcmContext) -> str:
        data = {
            "context_id": ctx.context_id,
            "device_id": ctx.device_id,
            "point_id": ctx.point_id,
            "source_protocol": ctx.source_protocol.name,
            "slave_id": ctx.slave_id,
            "function_code": ctx.function_code,
            "register_address": ctx.register_address,
            "data_type": ctx.data_type.name,
            "measurement_name": ctx.measurement_name,
            "unit": ctx.unit,
            "value": ctx.value,
            "timestamp_ms": ctx.timestamp_ms,
            "quality_state": ctx.quality_state.name.lower(),
            "network_state": ctx.network_state.name.lower(),
            "operation_type": ctx.operation_type.name.lower(),
            "control_constraint": {
                "writable": ctx.control_constraint.writable,
                "valid_range": [ctx.control_constraint.valid_range_min, ctx.control_constraint.valid_range_max]
            },
            "sequence_id": ctx.sequence_id,
        }
        return json.dumps(data, indent=2)

# ================================================================
# LAYER 3: AMM ADAPTIVE MAPPING
# ================================================================
class AmmLayer:
    """Manages MODBUS↔MQTT mapping table with validation."""

    DEFAULT_MAPPINGS = [
        AmmMappingEntry(slave_id=1, register_address=40001, data_type=DataType.FLOAT32,
                        device_id="plc_line1_01", point_id="motor_temp_01",
                        measurement_name="Motor temperature", unit="degC",
                        mqtt_topic="factory/line1/plc01/motor/temp",
                        constraint=ControlConstraint(False, 0, 120)),
        AmmMappingEntry(slave_id=1, register_address=40003, data_type=DataType.FLOAT32,
                        device_id="plc_line1_01", point_id="pressure_01",
                        measurement_name="Line pressure", unit="bar",
                        mqtt_topic="factory/line1/plc01/pressure",
                        constraint=ControlConstraint(False, 0, 10)),
        AmmMappingEntry(slave_id=2, register_address=40001, data_type=DataType.UINT16,
                        device_id="plc_line2_01", point_id="speed_01",
                        measurement_name="Conveyor speed", unit="rpm",
                        mqtt_topic="factory/line2/plc01/speed",
                        constraint=ControlConstraint(True, 0, 3000)),
        AmmMappingEntry(slave_id=2, register_address=40003, data_type=DataType.INT16,
                        device_id="plc_line2_01", point_id="current_01",
                        measurement_name="Motor current", unit="A",
                        mqtt_topic="factory/line2/plc01/current",
                        constraint=ControlConstraint(False, -50, 50)),
    ]

    def __init__(self):
        self._table: list[AmmMappingEntry] = [copy.deepcopy(m) for m in self.DEFAULT_MAPPINGS]

    def find(self, slave_id, reg_addr) -> Optional[AmmMappingEntry]:
        for e in self._table:
            if e.active and e.slave_id == slave_id and e.register_address == reg_addr:
                return e
        return None

    def enrich_context(self, ctx: TcmContext) -> bool:
        entry = self.find(ctx.slave_id, ctx.register_address)
        if entry:
            ctx.device_id = entry.device_id
            ctx.point_id = entry.point_id
            ctx.measurement_name = entry.measurement_name
            ctx.unit = entry.unit
            ctx.data_type = entry.data_type
            ctx.control_constraint = copy.deepcopy(entry.constraint)
            return True
        return False

    def validate_command(self, cmd: TcmContext) -> AmmValidationResult:
        result = AmmValidationResult()
        entry = self.find(cmd.slave_id, cmd.register_address)
        if not entry:
            result.reject_reason = f"Target device slave={cmd.slave_id} reg={cmd.register_address} not found"
            return result
        if not entry.constraint.writable:
            result.reject_reason = f"Register {cmd.register_address} on {entry.device_id} is read-only"
            return result
        if not (entry.constraint.valid_range_min <= cmd.value <= entry.constraint.valid_range_max):
            result.reject_reason = f"Value {cmd.value} out of range [{entry.constraint.valid_range_min}, {entry.constraint.valid_range_max}]"
            return result
        if cmd.function_code not in (6, 16):
            result.reject_reason = f"Invalid write function_code: {cmd.function_code}"
            return result
        result.accepted = True
        result.matched_entry = entry
        return result

    def add_mapping(self, entry: AmmMappingEntry):
        existing = self.find(entry.slave_id, entry.register_address)
        if existing:
            self._table.remove(existing)
        self._table.append(copy.deepcopy(entry))

    def get_mapping_count(self):
        return sum(1 for e in self._table if e.active)

    def get_mqtt_topic(self, slave_id, reg_addr) -> str:
        entry = self.find(slave_id, reg_addr)
        return entry.mqtt_topic if entry else ""

    def get_all_active(self) -> list:
        return [e for e in self._table if e.active]

# ================================================================
# LAYER 5: MQTT COMMUNICATION (simulated + real MQTT support)
# ================================================================
class MqttLayer:
    """MQTT publish/subscribe with connection state tracking.
    
    Supports both simulated mode (default) and real MQTT broker connection.
    When real MQTT is enabled, messages are actually published to the broker.
    """

    def __init__(self, broker_uri=None, client_id=None, username=None, password=None):
        self._connected = True
        self._publish_log = []
        self._publish_count = 0
        self._fail_count = 0
        
        self._real_mqtt = False
        self._broker_uri = broker_uri
        self._client_id = client_id or "esp32s3_gateway_sim"
        self._username = username
        self._password = password
        self._mqtt_client = None
        
        if broker_uri and MQTT_AVAILABLE:
            self._real_mqtt = True
            self._init_real_mqtt()
        elif broker_uri and not MQTT_AVAILABLE:
            print(f"  [MQTT] WARNING: paho-mqtt not installed, using simulation mode")

    def _init_real_mqtt(self):
        try:
            parts = self._broker_uri.split("://")
            if len(parts) == 2:
                proto = parts[0]
                addr = parts[1].split(":")
                host = addr[0]
                port = int(addr[1]) if len(addr) > 1 else 1883
                
                self._mqtt_client = mqtt_client.Client(client_id=self._client_id)
                
                if self._username:
                    self._mqtt_client.username_pw_set(self._username, self._password)
                
                self._mqtt_client.on_connect = self._on_connect
                self._mqtt_client.on_disconnect = self._on_disconnect
                self._mqtt_client.on_publish = self._on_publish
                
                print(f"  [MQTT] Connecting to real broker: {host}:{port}")
                print(f"  [MQTT] Client ID: {self._client_id}")
                
                self._mqtt_client.connect(host, port, keepalive=60)
                self._mqtt_client.loop_start()
                
                time.sleep(1)
                
                if self._mqtt_client.is_connected():
                    print(f"  [MQTT] Connected to {host}:{port}")
                else:
                    print(f"  [MQTT] WARNING: Connection may still be pending")
                    
        except Exception as e:
            print(f"  [MQTT] ERROR: Failed to connect: {e}")
            self._real_mqtt = False
            self._mqtt_client = None

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"  [MQTT] Real broker connected (rc={rc})")
            self._connected = True
        else:
            print(f"  [MQTT] Connection failed (rc={rc})")
            self._connected = False

    def _on_disconnect(self, client, userdata, rc):
        print(f"  [MQTT] Disconnected (rc={rc})")
        self._connected = False

    def _on_publish(self, client, userdata, mid):
        print(f"  [MQTT] Publish acknowledged (mid={mid})")

    @property
    def is_connected(self):
        if self._real_mqtt and self._mqtt_client:
            return self._mqtt_client.is_connected()
        return self._connected

    def connect(self):
        self._connected = True
        if self._real_mqtt and self._mqtt_client:
            try:
                self._mqtt_client.reconnect()
            except Exception as e:
                print(f"  [MQTT] Reconnect failed: {e}")

    def disconnect(self):
        self._connected = False
        if self._real_mqtt and self._mqtt_client:
            self._mqtt_client.disconnect()

    def publish(self, topic: str, payload: str, qos: int = 0) -> bool:
        if self._real_mqtt and self._mqtt_client:
            try:
                result = self._mqtt_client.publish(topic, payload, qos=qos)
                if result.rc == 0:
                    self._publish_count += 1
                    self._publish_log.append({"topic": topic, "payload_len": len(payload), "time": _now_ms()})
                    print(f"  [MQTT] Published: {topic} (qos={qos})")
                    return True
                else:
                    self._fail_count += 1
                    print(f"  [MQTT] Publish failed: rc={result.rc}")
                    return False
            except Exception as e:
                self._fail_count += 1
                print(f"  [MQTT] Publish exception: {e}")
                return False
        else:
            if self._connected:
                self._publish_count += 1
                self._publish_log.append({"topic": topic, "payload_len": len(payload), "time": _now_ms()})
                return True
            self._fail_count += 1
            return False

    def get_stats(self):
        return {"published": self._publish_count, "failed": self._fail_count}

    def cleanup(self):
        if self._real_mqtt and self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()

# ================================================================
# LAYER 6: UIF PERSISTENCE (offline cache + ordered replay)
# ================================================================
class UifLayer:
    """SPI Flash cache with ordered replay on reconnection."""

    MAX_RECORDS = 512

    def __init__(self):
        self._cache: list[UifCacheEntry] = []
        self._data_loss = 0

    def cache_record(self, ctx: TcmContext) -> bool:
        if len(self._cache) >= self.MAX_RECORDS:
            # Try to evict replayed entries
            self._cache = [e for e in self._cache if e.pending]
            if len(self._cache) >= self.MAX_RECORDS:
                self._data_loss += 1
                return False
        self._cache.append(UifCacheEntry(
            json_data=TcmLayer.serialize_json(ctx),
            sequence_id=ctx.sequence_id,
            timestamp_ms=ctx.timestamp_ms,
            pending=True
        ))
        return True

    def get_cached_count(self):
        return sum(1 for e in self._cache if e.pending)

    def get_cache_usage_percent(self):
        return int(len(self._cache) / self.MAX_RECORDS * 100)

    @property
    def data_loss_count(self):
        return self._data_loss

    def replay_all(self, mqtt: MqttLayer, amm: AmmLayer) -> dict:
        """Replay cached records in sequence_id order via MQTT."""
        pending = [(i, e) for i, e in enumerate(self._cache) if e.pending]
        pending.sort(key=lambda x: x[1].sequence_id)

        replayed = 0
        order_correct = 0
        last_seq = -1
        replay_start = _now_ms()

        for idx, entry in pending:
            topic = "factory/data/replayed"
            if mqtt.publish(topic, entry.json_data):
                self._cache[idx].pending = False
                replayed += 1
                if entry.sequence_id > last_seq:
                    order_correct += 1
                last_seq = entry.sequence_id

        # Clean up
        self._cache = [e for e in self._cache if e.pending]
        replay_time = _now_ms() - replay_start

        return {
            "replayed": replayed,
            "order_correct": order_correct,
            "order_accuracy": f"{order_correct/max(replayed,1)*100:.1f}%",
            "replay_time_ms": replay_time,
        }

# ================================================================
# LAYER 7: EVALUATION LOGGER
# ================================================================
class EvalLayer:
    def __init__(self):
        self.metrics = EvalMetrics()
        self._events = []

    def log(self, event_type: str, detail: str = ""):
        self._events.append({"type": event_type, "detail": detail, "time": _now_ms()})
        m = self.metrics
        dispatch = {
            "poll_ok": lambda: setattr(m, 'successful_polls', m.successful_polls + 1),
            "poll_fail": lambda: setattr(m, 'failed_polls', m.failed_polls + 1),
            "ctx_created": lambda: setattr(m, 'contexts_created', m.contexts_created + 1),
            "ctx_validated": lambda: setattr(m, 'contexts_validated', m.contexts_validated + 1),
            "ctx_rejected": lambda: setattr(m, 'contexts_rejected', m.contexts_rejected + 1),
            "mqtt_pub": lambda: setattr(m, 'mqtt_published', m.mqtt_published + 1),
            "mqtt_fail": lambda: setattr(m, 'mqtt_failed', m.mqtt_failed + 1),
            "cached": lambda: setattr(m, 'cached_records', m.cached_records + 1),
            "replayed": lambda: setattr(m, 'replayed_records', m.replayed_records + 1),
            "data_loss": lambda: setattr(m, 'data_loss_count', m.data_loss_count + 1),
            "cmd_recv": lambda: setattr(m, 'commands_received', m.commands_received + 1),
            "cmd_accept": lambda: setattr(m, 'commands_accepted', m.commands_accepted + 1),
            "cmd_reject": lambda: setattr(m, 'commands_rejected', m.commands_rejected + 1),
        }
        if event_type in dispatch:
            dispatch[event_type]()

    def inc_polls(self):
        self.metrics.total_polls += 1

    def print_summary(self, title=""):
        m = self.metrics
        print(f"\n{'='*60}")
        print(f"  EVALUATION METRICS SUMMARY {title}")
        print(f"{'='*60}")
        print(f"  MODBUS  | Polls: {m.total_polls}  Success: {m.successful_polls}  Failed: {m.failed_polls}  Rate: {m.successful_polls/max(m.total_polls,1)*100:.0f}%")
        print(f"  TCM     | Created: {m.contexts_created}  Validated: {m.contexts_validated}  Rejected: {m.contexts_rejected}")
        print(f"  MQTT    | Published: {m.mqtt_published}  Failed: {m.mqtt_failed}")
        print(f"  Cache   | Stored: {m.cached_records}  Replayed: {m.replayed_records}  Lost: {m.data_loss_count}")
        print(f"  Command | Received: {m.commands_received}  Accepted: {m.commands_accepted}  Rejected: {m.commands_rejected}")
        print(f"{'='*60}\n")

# ================================================================
# HELPER
# ================================================================
_start_time = time.time()
def _now_ms():
    return int((time.time() - _start_time) * 1000)

def _print_banner(title):
    print(f"\n{'#'*70}")
    print(f"  {title}")
    print(f"{'#'*70}")

# ================================================================
# SIMULATION SCENARIOS
# ================================================================

# Poll configuration: (slave_id, function_code, register_address)
POLL_TARGETS = [
    (1, 3, 40001),
    (1, 3, 40003),
    (2, 3, 40001),
    (2, 3, 40003),
]


def scenario1_normal_operation(modbus, tcm, amm, mqtt, uif, eval_, cycles=10, cloud_platform="standard"):
    """Scenario 1: Normal uplink data flow with full TCM pipeline."""
    _print_banner("SCENARIO 1: Normal Operation (Uplink Data Flow)")

    thingscloud_attrs = {}

    for cycle in range(cycles):
        print(f"\n--- Poll Cycle {cycle+1}/{cycles} ---")
        thingscloud_attrs.clear()

        for slave_id, func_code, reg_addr in POLL_TARGETS:
            eval_.inc_polls()

            # Layer 1: MODBUS read
            raw_value, quality = modbus.read_register(slave_id, func_code, reg_addr)
            if quality == QualityState.GOOD:
                eval_.log("poll_ok")
            else:
                eval_.log("poll_fail")
                continue

            # Layer 2: TCM build
            net_state = NetworkState.ONLINE if mqtt.is_connected else NetworkState.OFFLINE
            ctx = TcmLayer.build_context(slave_id, func_code, reg_addr, raw_value, quality, net_state)
            eval_.log("ctx_created")

            # Layer 3: AMM enrich
            amm.enrich_context(ctx)

            # Layer 2: TCM validate
            passed, reason = TcmLayer.validate(ctx)
            if passed:
                eval_.log("ctx_validated")
            else:
                eval_.log("ctx_rejected", reason)
                continue

            if cloud_platform == "thingscloud":
                thingscloud_attrs[ctx.point_id] = ctx.value
            else:
                topic = amm.get_mqtt_topic(slave_id, reg_addr)
                payload = TcmLayer.serialize_json(ctx)

                if mqtt.is_connected:
                    if mqtt.publish(topic, payload):
                        eval_.log("mqtt_pub")
                        if cycle == 0:
                            print(f"  [{ctx.device_id}/{ctx.point_id}] value={ctx.value} {ctx.unit} | quality={ctx.quality_state.name}")
                            if slave_id == 1 and reg_addr == 40001:
                                print(f"    TCM JSON payload:\n{payload}")
                    else:
                        eval_.log("mqtt_fail")
                else:
                    if uif.cache_record(ctx):
                        eval_.log("cached")
                    else:
                        eval_.log("data_loss")

        if cloud_platform == "thingscloud" and mqtt.is_connected and thingscloud_attrs:
            topic = "attributes"
            payload = json.dumps(thingscloud_attrs)
            if mqtt.publish(topic, payload):
                eval_.log("mqtt_pub")
                print(f"  [ThingsCloud] Published to 'attributes': {thingscloud_attrs}")
            else:
                eval_.log("mqtt_fail")
        elif cloud_platform == "thingscloud" and not mqtt.is_connected and thingscloud_attrs:
            ctx_copy = ctx
            ctx_copy._raw_value = 0
            if uif.cache_record(ctx_copy):
                eval_.log("cached")
            else:
                eval_.log("data_loss")

    eval_.print_summary("(After Normal Operation)")


def scenario2_offline_caching(modbus, tcm, amm, mqtt, uif, eval_, cycles=5):
    """Scenario 2: MQTT disconnection → offline caching."""
    _print_banner("SCENARIO 2: MQTT Disconnection + Offline Caching")

    mqtt.disconnect()
    print(f"  MQTT disconnected. Network state = OFFLINE")
    print(f"  Polling {cycles} cycles with data routed to SPI Flash cache...\n")

    for cycle in range(cycles):
        for slave_id, func_code, reg_addr in POLL_TARGETS:
            eval_.inc_polls()
            raw_value, quality = modbus.read_register(slave_id, func_code, reg_addr)
            if quality != QualityState.GOOD:
                eval_.log("poll_fail")
                continue
            eval_.log("poll_ok")

            ctx = TcmLayer.build_context(slave_id, func_code, reg_addr, raw_value, quality, NetworkState.OFFLINE)
            eval_.log("ctx_created")
            amm.enrich_context(ctx)
            passed, _ = TcmLayer.validate(ctx)
            if passed:
                eval_.log("ctx_validated")
            else:
                eval_.log("ctx_rejected")
                continue

            # Cache to UIF
            if uif.cache_record(ctx):
                eval_.log("cached")
                print(f"  [Cycle {cycle+1}] Cached: {ctx.device_id}/{ctx.point_id} = {ctx.value} {ctx.unit} (seq={ctx.sequence_id})")
            else:
                eval_.log("data_loss")

    print(f"\n  Cache status: {uif.get_cached_count()} pending records | {uif.get_cache_usage_percent()}% usage | {uif.data_loss_count} lost")


def scenario3_reconnect_replay(modbus, tcm, amm, mqtt, uif, eval_):
    """Scenario 3: MQTT reconnection → ordered replay."""
    _print_banner("SCENARIO 3: Reconnection + Ordered Replay")

    mqtt.connect()
    print(f"  MQTT reconnected. Triggering ordered replay...\n")

    replay_result = uif.replay_all(mqtt, amm)

    print(f"  Replay results:")
    print(f"    Records replayed:  {replay_result['replayed']}")
    print(f"    Order correct:     {replay_result['order_correct']}")
    print(f"    Order accuracy:    {replay_result['order_accuracy']}")
    print(f"    Replay duration:   {replay_result['replay_time_ms']} ms")

    for _ in range(replay_result['replayed']):
        eval_.log("replayed")
        eval_.log("mqtt_pub")

    eval_.print_summary("(After Replay)")


def scenario4_downlink_validation(modbus, amm, eval_):
    """Scenario 4: Downlink command validation (4 test cases)."""
    _print_banner("SCENARIO 4: Downlink Command Validation")

    test_commands = [
        {"desc": "Valid write (speed=1800 rpm)", "cmd": TcmContext(
            slave_id=2, register_address=40001, function_code=6, value=1800,
            device_id="plc_line2_01", point_id="speed_01",
            measurement_name="Conveyor speed", unit="rpm",
            data_type=DataType.UINT16, timestamp_ms=_now_ms(), context_id=999)},
        {"desc": "Write to read-only device (temp)", "cmd": TcmContext(
            slave_id=1, register_address=40001, function_code=6, value=80,
            device_id="plc_line1_01", point_id="motor_temp_01",
            measurement_name="Motor temperature", unit="degC",
            data_type=DataType.FLOAT32, timestamp_ms=_now_ms(), context_id=998)},
        {"desc": "Out-of-range value (speed=5000, max=3000)", "cmd": TcmContext(
            slave_id=2, register_address=40001, function_code=6, value=5000,
            device_id="plc_line2_01", point_id="speed_01",
            measurement_name="Conveyor speed", unit="rpm",
            data_type=DataType.UINT16, timestamp_ms=_now_ms(), context_id=997)},
        {"desc": "Non-existent device (slave=99)", "cmd": TcmContext(
            slave_id=99, register_address=40001, function_code=6, value=100,
            device_id="unknown", point_id="unknown",
            measurement_name="Unknown", unit="?",
            data_type=DataType.FLOAT32, timestamp_ms=_now_ms(), context_id=996)},
    ]

    for i, test in enumerate(test_commands):
        eval_.metrics.commands_received = eval_.metrics.commands_received + 1
        eval_.log("cmd_recv")
        result = amm.validate_command(test["cmd"])

        if result.accepted:
            eval_.log("cmd_accept")
            status = "ACCEPTED"
            modbus.write_register(test["cmd"].slave_id, test["cmd"].register_address, test["cmd"].value)
        else:
            eval_.log("cmd_reject")
            status = f"REJECTED ({result.reject_reason})"

        print(f"  Test {i+1}: {test['desc']}")
        print(f"    Result: {status}\n")

    print(f"  Commands: {eval_.metrics.commands_received} received | {eval_.metrics.commands_accepted} accepted | {eval_.metrics.commands_rejected} rejected")


def scenario5_adaptive_mapping(modbus, tcm, amm, mqtt, eval_):
    """Scenario 5: Runtime mapping adaptation."""
    _print_banner("SCENARIO 5: Adaptive Mapping (Runtime Device Addition)")

    old_count = amm.get_mapping_count()
    print(f"  Current mapping entries: {old_count}")

    # Add new device
    adapt_start = _now_ms()
    new_entry = AmmMappingEntry(
        slave_id=3, function_code=3, register_address=40010,
        data_type=DataType.FLOAT32,
        device_id="flow_meter_01", point_id="flow_rate_01",
        measurement_name="Volumetric flow rate", unit="L/min",
        mqtt_topic="factory/line3/flowmeter/flow",
        constraint=ControlConstraint(False, 0, 500),
    )
    amm.add_mapping(new_entry)

    # Also add to MODBUS simulator
    modbus.SIMULATED_DEVICES[(3, 40010)] = {"base": 120.0, "variance": 5.0, "type": DataType.FLOAT32, "name": "flow"}

    adapt_time = _now_ms() - adapt_start
    new_count = amm.get_mapping_count()

    print(f"  Added: slave=3, reg=40010, device=flow_meter_01, point=flow_rate_01")
    print(f"  New mapping entries: {new_count} (+{new_count - old_count})")
    print(f"  Adaptation latency: {adapt_time} ms")

    # Poll new device
    print(f"\n  Polling new device...")
    raw_value, quality = modbus.read_register(3, 3, 40010)
    ctx = TcmLayer.build_context(3, 3, 40010, raw_value, quality, NetworkState.ONLINE)
    amm.enrich_context(ctx)
    passed, _ = TcmLayer.validate(ctx)

    print(f"  New device context:")
    print(f"    device_id:       {ctx.device_id}")
    print(f"    point_id:        {ctx.point_id}")
    print(f"    measurement:     {ctx.measurement_name}")
    print(f"    value:           {ctx.value} {ctx.unit}")
    print(f"    validated:       {passed}")
    print(f"    mqtt_topic:      {amm.get_mqtt_topic(3, 40010)}")


# ================================================================
# MAIN SIMULATION
# ================================================================
def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 MCP-Adapted MODBUS-MQTT Gateway Simulation")
    parser.add_argument("--mqtt-broker", help="MQTT broker URI (e.g., mqtt://broker.example.com:1883)")
    parser.add_argument("--mqtt-client", help="MQTT client ID")
    parser.add_argument("--mqtt-username", help="MQTT username (optional)")
    parser.add_argument("--mqtt-password", help="MQTT password (optional)")
    parser.add_argument("--cloud-platform", choices=["standard", "thingscloud"], default="standard",
                        help="Cloud platform type: standard (default) or thingscloud")
    parser.add_argument("--cycles", type=int, default=10, help="Number of polling cycles")
    args = parser.parse_args()

    print("=" * 70)
    print("  ESP32-S3 MCP-Adapted MODBUS-MQTT Gateway — PC Simulation")
    print("  Implementing: TCM + AMM + UIF | 5 Test Scenarios")
    print("=" * 70)

    if args.mqtt_broker:
        print(f"\n  [CONFIG] Real MQTT enabled: {args.mqtt_broker}")
        if args.mqtt_client:
            print(f"  [CONFIG] Client ID: {args.mqtt_client}")
        if args.mqtt_username:
            print(f"  [CONFIG] Username: {args.mqtt_username}")
        print(f"  [CONFIG] Cloud Platform: {args.cloud_platform}")
    else:
        print(f"\n  [CONFIG] Using simulated MQTT mode")

    # Initialize all layers
    TcmLayer.reset()
    modbus = ModbusAccessLayer()
    amm = AmmLayer()
    mqtt = MqttLayer(
        broker_uri=args.mqtt_broker,
        client_id=args.mqtt_client,
        username=args.mqtt_username,
        password=args.mqtt_password
    )
    uif = UifLayer()
    eval_ = EvalLayer()

    print(f"\n  AMM default mappings: {amm.get_mapping_count()} entries")
    for entry in amm.get_all_active():
        print(f"    slave={entry.slave_id} reg={entry.register_address} → {entry.device_id}/{entry.point_id} ({entry.unit})")

    # Run all 5 scenarios
    scenario1_normal_operation(modbus, TcmLayer, amm, mqtt, uif, eval_, cycles=args.cycles, cloud_platform=args.cloud_platform)
    scenario2_offline_caching(modbus, TcmLayer, amm, mqtt, uif, eval_, cycles=5)
    scenario3_reconnect_replay(modbus, TcmLayer, amm, mqtt, uif, eval_)
    scenario4_downlink_validation(modbus, amm, eval_)
    scenario5_adaptive_mapping(modbus, TcmLayer, amm, mqtt, eval_)

    # Cleanup MQTT connection
    mqtt.cleanup()

    # Final summary
    eval_.print_summary("FINAL (All Scenarios)")

    print("  Simulation complete.")
    print(f"  Total TCM contexts created: {TcmLayer._context_id_counter}")
    print(f"  Total sequence IDs issued:  {TcmLayer._sequence_counter}")


if __name__ == "__main__":
    main()
