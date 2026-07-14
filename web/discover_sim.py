#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Virtual MODBUS Bus & Device Discovery Simulator.

Simulates a MODBUS RTU bus with multiple slave devices, each exposing
a set of holding registers with realistic industrial values.  Provides
broadcast scan, register scan, semantic inference, and auto-mapping
creation — mirroring the C firmware modbus_discover module.

Usage:
    import discover_sim
    discover_sim.init()
    discover_sim.full_scan()
    print(discover_sim.get_result())
    discover_sim.apply_mappings()
"""

import math
import struct
import time
import threading

# ---- MODBUS TCP Bridge (optional, for connecting to real MODBUS servers) ----
try:
    import modbus_bridge
    _bridge_available = True
except ImportError:
    _bridge_available = False

# ---- Simulator metadata query (optional, for enriched scanning) ----
_sim_metadata_url = None  # e.g., 'http://localhost:8081'


def set_simulator_url(url):
    """Set the simulator REST API URL for metadata queries."""
    global _sim_metadata_url
    _sim_metadata_url = url


def _query_simulator_metadata(slave_id):
    """Query the simulator for a device's register definitions.

    Returns a dict mapping address -> {data_type, function_code, name, unit, writable}
    or None if the simulator is not available or the device doesn't exist.
    Includes retry logic (up to 3 attempts) with a 10-second timeout per attempt.
    """
    if not _sim_metadata_url:
        return None
    import urllib.request
    import json as _json

    max_retries = 3
    for attempt in range(max_retries):
        try:
            url = f"{_sim_metadata_url}/api/devices/{slave_id}"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = _json.loads(resp.read())
            if 'error' in data:
                return None
            meta = {}
            for reg in data.get('registers', []):
                addr = reg['address']
                meta[addr] = {
                    'data_type': reg.get('data_type', 'UINT16'),
                    'function_code': reg.get('function_code', 3),
                    'name': reg.get('name', ''),
                    'unit': reg.get('unit', ''),
                    'writable': reg.get('writable', False),
                    'sim_mode': reg.get('sim_mode', ''),
                }
                # For multi-register types, also mark the second register
                dtype = reg.get('data_type', 'UINT16')
                if dtype in ('FLOAT32', 'UINT32'):
                    meta[addr + 1] = {
                        'data_type': dtype,
                        'function_code': reg.get('function_code', 3),
                        'name': reg.get('name', ''),
                        'unit': reg.get('unit', ''),
                        'writable': reg.get('writable', False),
                        '_continuation': True,  # Marks this as the 2nd register of a pair
                    }
            return meta
        except Exception as e:
            if attempt < max_retries - 1:
                time.sleep(0.5)  # Brief pause before retry
                continue
            _add_log("warn", f"[DISCOVER] Metadata query failed for slave {slave_id} "
                     f"after {max_retries} attempts: {e}")
            return None

# ======================== Semantic Profile Database ========================

SEMANTIC_PROFILES = [
    # (addr_lo, addr_hi, name, unit, range_min, range_max, writable)
    (40001, 40099, "Temperature",    "degC",  -40.0,  500.0, False),
    (40101, 40199, "Pressure",       "bar",     0.0,  100.0, False),
    (40201, 40299, "Flow rate",      "L/min",   0.0, 5000.0, False),
    (40301, 40399, "Speed",          "rpm",     0.0, 6000.0, True),
    (40401, 40499, "Electrical",     "V",       0.0,  600.0, False),
    (40501, 40599, "Humidity",       "%RH",     0.0,  100.0, False),
    (40601, 40699, "Level",          "mm",      0.0, 5000.0, False),
    (40701, 40799, "Counter",        "pcs",     0.0, 99999.0, False),
    (40801, 40899, "Setpoint",       "",     -999.0,  999.0, True),
    (40901, 40999, "Status",         "",        0.0,  255.0, True),
]

# Value-based refinement hints
VALUE_HINTS = {
    "Temperature": [
        (-40.0,   0.0, "Temperature (cold)",  "degC"),
        (  0.0,  50.0, "Ambient temperature", "degC"),
        ( 50.0, 150.0, "Process temperature", "degC"),
        (150.0, 500.0, "High temperature",    "degC"),
    ],
    "Pressure": [
        (0.0,  1.0, "Vacuum pressure", "bar"),
        (1.0, 10.0, "Line pressure",   "bar"),
        (10.0, 100.0, "High pressure", "bar"),
    ],
    "Speed": [
        (0.0, 100.0, "Slow speed", "rpm"),
        (100.0, 1500.0, "Nominal speed", "rpm"),
        (1500.0, 6000.0, "High speed", "rpm"),
    ],
}


def _float_to_regs(value):
    """Convert a Python float to two 16-bit MODBUS registers (big-endian word)."""
    raw = struct.pack('>f', value)
    hi = struct.unpack('>H', raw[0:2])[0]
    lo = struct.unpack('>H', raw[2:4])[0]
    return [hi, lo]


def _regs_to_float(regs):
    """Convert two 16-bit registers back to float."""
    raw = struct.pack('>HH', regs[0], regs[1])
    return struct.unpack('>f', raw)[0]


def _is_valid_float(value):
    """Check if a float is a plausible sensor reading."""
    if math.isnan(value) or math.isinf(value):
        return False
    return 0.001 < abs(value) < 1e8


# ======================== Virtual MODBUS Bus ========================

class VirtualModbusDevice:
    """A simulated MODBUS slave with a set of holding registers."""

    def __init__(self, slave_id, name, registers):
        """
        Args:
            slave_id: MODBUS slave address (1-247).
            name: Human-readable device name.
            registers: dict mapping register_address (int) to
                       (value_float, data_type_str, writable_bool).
                       data_type_str: 'FLOAT32', 'INT16', 'UINT16'.
        """
        self.slave_id = slave_id
        self.name = name
        self.registers = {}  # addr -> {'value': float, 'type': str, 'writable': bool, 'raw': [regs]}
        for addr, (value, dtype, writable) in registers.items():
            if dtype == 'FLOAT32':
                raw = _float_to_regs(value)
            elif dtype == 'INT16':
                raw = [int(value) & 0xFFFF]
            else:  # UINT16
                raw = [int(value) & 0xFFFF]
            self.registers[addr] = {
                'value': value, 'type': dtype, 'writable': writable, 'raw': raw
            }


# Pre-configured virtual devices (simulating a small factory line)
VIRTUAL_BUS = [
    VirtualModbusDevice(1, "PLC_Line1_Controller", {
        40001: (72.5,  'FLOAT32', False),   # Motor temperature
        40003: (3.82,  'FLOAT32', False),   # Line pressure
        40005: (1500,  'UINT16',  False),   # Motor RPM
        40006: (12.4,  'FLOAT32', False),   # Motor current (A)
        40008: (45.2,  'FLOAT32', False),   # Ambient temperature
        40801: (80.0,  'FLOAT32', True),    # Temperature setpoint
    }),
    VirtualModbusDevice(2, "Conveyor_Drive_Unit", {
        40001: (58.3,  'FLOAT32', False),   # Drive temperature
        40003: (2.15,  'FLOAT32', False),   # Belt tension (bar)
        40301: (1200,  'UINT16',  True),    # Speed setpoint (rpm)
        40302: (1198,  'UINT16',  False),   # Actual speed (rpm)
        40401: (380.5, 'FLOAT32', False),   # Bus voltage (V)
        40403: (8.7,   'FLOAT32', False),   # Phase current (A)
    }),
    VirtualModbusDevice(3, "FlowMeter_Station", {
        40201: (125.6, 'FLOAT32', False),   # Volumetric flow rate (L/min)
        40203: (22.4,  'FLOAT32', False),   # Fluid temperature (degC)
        40205: (1.08,  'FLOAT32', False),   # Fluid density (g/cm3)
        40207: (45230, 'UINT16',  False),   # Totalizer (L)
        40601: (850,   'UINT16',  False),   # Tank level (mm)
    }),
    VirtualModbusDevice(5, "HVAC_Unit_01", {
        40501: (62.3,  'FLOAT32', False),   # Room humidity (%RH)
        40001: (23.8,  'FLOAT32', False),   # Room temperature
        40003: (18.2,  'FLOAT32', False),   # Supply air temperature
        40801: (24.0,  'FLOAT32', True),    # Temperature setpoint
        40803: (55.0,  'FLOAT32', True),    # Humidity setpoint (%RH)
        40901: (1,     'UINT16',  True),    # Unit enable/disable
    }),
]


# ======================== MODBUS Read/Write Simulation ========================

def _find_device(slave_id):
    for dev in VIRTUAL_BUS:
        if dev.slave_id == slave_id:
            return dev
    return None


def modbus_read_holding_register(slave_id, reg_addr, reg_count=1):
    """Simulate MODBUS FC03 read. Returns list of raw register values or None."""
    dev = _find_device(slave_id)
    if not dev:
        return None

    # reg_addr is 0-based in the C API, but our virtual bus uses absolute addresses
    abs_addr = reg_addr + 40001 if reg_addr < 40000 else reg_addr

    results = []
    for i in range(reg_count):
        addr = abs_addr + i
        if addr in dev.registers:
            results.extend(dev.registers[addr]['raw'])
        else:
            results.append(0)
    return results


def modbus_write_single_register(slave_id, reg_addr, value):
    """Simulate MODBUS FC06 write. Returns True if successful."""
    dev = _find_device(slave_id)
    if not dev:
        return False

    abs_addr = reg_addr + 40001 if reg_addr < 40000 else reg_addr
    if abs_addr in dev.registers and dev.registers[abs_addr]['writable']:
        return True
    return False


# ======================== Semantic Inference ========================

def infer_semantics(reg_addr, value):
    """
    Infer measurement name, unit, and valid range from register address and value.

    Returns:
        dict with keys: name, unit, range_min, range_max, writable
    """
    result = {
        'name': f'Register_{reg_addr}',
        'unit': '',
        'range_min': -32768.0,
        'range_max': 65535.0,
        'writable': False,
    }

    for addr_lo, addr_hi, name, unit, rmin, rmax, writable in SEMANTIC_PROFILES:
        if addr_lo <= reg_addr <= addr_hi:
            result['name'] = name
            result['unit'] = unit
            result['range_min'] = rmin
            result['range_max'] = rmax
            result['writable'] = writable

            # Value-based refinement
            hints = VALUE_HINTS.get(name, [])
            for vlo, vhi, rname, runit in hints:
                if vlo <= value < vhi:
                    result['name'] = rname
                    result['unit'] = runit
                    break
            break

    return result


def infer_data_type(raw_regs, reg_addr):
    """Determine data type from raw register values."""
    if len(raw_regs) >= 2:
        fval = _regs_to_float(raw_regs[:2])
        if _is_valid_float(fval):
            return 'FLOAT32'

    val = raw_regs[0] if raw_regs else 0
    if val > 32767:
        return 'UINT16'
    return 'INT16'


# ======================== Discovery Engine ========================

class DiscoverResult:
    def __init__(self):
        self.devices = []       # list of discovered device dicts
        self.total_scanned = 0
        self.devices_found = 0
        self.registers_found = 0
        self.mappings_created = 0
        self.scan_complete = False
        self.scan_in_progress = False
        self.scan_start_time = None
        self.scan_end_time = None

    def to_dict(self):
        return {
            'total_scanned': self.total_scanned,
            'devices_found': self.devices_found,
            'registers_found': self.registers_found,
            'mappings_created': self.mappings_created,
            'scan_complete': self.scan_complete,
            'scan_in_progress': self.scan_in_progress,
            'scan_duration_ms': int((self.scan_end_time - self.scan_start_time) * 1000)
                if self.scan_start_time and self.scan_end_time else 0,
        }

    def devices_to_list(self):
        return self.devices


# Module-level state
_result = DiscoverResult()
_mappings = []  # Will be set by the web server
_add_log = None  # Log callback
_lock = threading.Lock()


def init(mappings_list=None, log_callback=None):
    """Initialize the discovery module.

    Args:
        mappings_list: Reference to the shared MAPPINGS list from web_server_sim.
        log_callback: Function(level, text) for adding log entries.
    """
    global _result, _mappings, _add_log
    _result = DiscoverResult()
    _mappings = mappings_list or []
    _add_log = log_callback or (lambda *a: None)


def reset():
    """Clear all discovery results."""
    global _result
    _result = DiscoverResult()
    _add_log("info", "[DISCOVER] Discovery state reset")


def _bridge_read_float(slave_id, addr, count=2):
    """Read registers via bridge and interpret as FLOAT32 (2 regs, big-endian)."""
    if not _bridge_available:
        return None
    regs = modbus_bridge.read_holding_registers(slave_id, addr, count)
    if regs is None:
        # Try input registers
        regs = modbus_bridge.read_input_registers(slave_id, addr, count)
    if regs is None or len(regs) < 2:
        return None
    try:
        return _regs_to_float(regs[:2])
    except Exception:
        return None


def _bridge_read_raw(slave_id, addr, count=1):
    """Read raw register values via bridge (FC03 first, then FC04)."""
    if not _bridge_available:
        return None
    regs = modbus_bridge.read_holding_registers(slave_id, addr, count)
    if regs is None:
        regs = modbus_bridge.read_input_registers(slave_id, addr, count)
    return regs


def broadcast_scan(slave_start=1, slave_end=247):
    """Quick broadcast scan to find active slave devices.

    If the MODBUS bridge is connected, performs real MODBUS TCP probing.
    Otherwise uses the virtual bus simulation.
    """
    with _lock:
        if _result.scan_in_progress:
            return {'error': 'Scan already in progress'}

        _result.scan_in_progress = True
        _result.scan_start_time = time.time()
        _result.devices = []

    use_bridge = (_bridge_available and modbus_bridge.is_connected())
    mode_str = "MODBUS TCP bridge" if use_bridge else "virtual bus"
    _add_log("info", f"[DISCOVER] Broadcast scan ({mode_str}): slave IDs {slave_start}..{slave_end}")

    found = []

    if use_bridge:
        # ---- Real MODBUS TCP scan via bridge ----
        found_ids = []
        for sid in range(slave_start, min(slave_end + 1, 250)):
            # Probe with absolute address 40001 (standard first holding register)
            # Try FC03 first, then FC04
            r = modbus_bridge.read_holding_registers(sid, 40001, 1)
            if r is None:
                r = modbus_bridge.read_input_registers(sid, 40001, 1)
            if r is not None:
                found_ids.append(sid)

        for sid in found_ids:
            # Estimate register count by probing common ranges with absolute addresses
            reg_count = 0
            for probe_addr in range(40001, 40021):
                r3 = modbus_bridge.read_holding_registers(sid, probe_addr, 1)
                r4 = modbus_bridge.read_input_registers(sid, probe_addr, 1)
                has_data = False
                if r3 is not None and any(v != 0 for v in r3):
                    has_data = True
                elif r4 is not None and any(v != 0 for v in r4):
                    has_data = True
                if has_data:
                    reg_count += 1

            device_info = {
                'slave_id': sid,
                'device_id': f'device_slave_{sid:02d}',
                'name': f'MODBUS_TCP_Slave_{sid}',
                'register_count': reg_count,
                'registers': [],
                'active': True,
                'source': 'bridge',
            }
            found.append(device_info)
            # Add to result incrementally so frontend can see it immediately
            with _lock:
                _result.devices.append(device_info)
                _result.devices_found = len(_result.devices)
            _add_log("ok", f"[DISCOVER] Found slave {sid} via MODBUS TCP "
                     f"({reg_count} non-zero registers)")
            time.sleep(0.05)
    else:
        # ---- Virtual bus scan ----
        for dev in VIRTUAL_BUS:
            if slave_start <= dev.slave_id <= slave_end:
                device_info = {
                    'slave_id': dev.slave_id,
                    'device_id': f'device_slave_{dev.slave_id:02d}',
                    'name': dev.name,
                    'register_count': len(dev.registers),
                    'registers': [],
                    'active': True,
                    'source': 'virtual',
                }
                found.append(device_info)
                # Add to result incrementally
                with _lock:
                    _result.devices.append(device_info)
                    _result.devices_found = len(_result.devices)
                _add_log("ok", f"[DISCOVER] Found slave {dev.slave_id}: {dev.name}")
                time.sleep(0.05)

    with _lock:
        _result.total_scanned = slave_end - slave_start + 1
        _result.scan_in_progress = False
        _result.scan_complete = True
        _result.scan_end_time = time.time()

    _add_log("ok", f"[DISCOVER] Broadcast complete: {len(found)} devices found ({mode_str})")
    return _result.to_dict()


def scan_device(slave_id, reg_start=40001, reg_end=40100):
    """Fine register scan for a specific slave device.

    If the MODBUS bridge is connected, reads real register values via MODBUS TCP.
    Otherwise uses the virtual bus simulation.
    """
    use_bridge = (_bridge_available and modbus_bridge.is_connected())

    if not use_bridge:
        dev = _find_device(slave_id)
        if not dev:
            return {'error': f'Slave {slave_id} not found on bus'}

    mode_str = "MODBUS TCP" if use_bridge else "virtual"
    _add_log("info", f"[DISCOVER] Register scan ({mode_str}): slave {slave_id}, "
             f"addr {reg_start}..{reg_end}")

    registers = []
    addr = reg_start

    if use_bridge:
        # ---- Real MODBUS TCP register read with metadata enhancement ----
        metadata = _query_simulator_metadata(slave_id)

        if metadata:
            _add_log("info", f"[DISCOVER] Using metadata for slave {slave_id}: {len(metadata)} entries")
            # Use metadata to know exact register layout and types
            sorted_addrs = sorted(metadata.keys())
            for addr in sorted_addrs:
                meta = metadata[addr]
                if meta.get('_continuation'):
                    continue
                if addr < reg_start or addr > reg_end:
                    continue

                dtype = meta['data_type']
                fc = meta['function_code']
                count = 2 if dtype in ('FLOAT32', 'UINT32') else 1

                if fc == 4:
                    raw_regs = modbus_bridge.read_input_registers(slave_id, addr, count)
                else:
                    raw_regs = modbus_bridge.read_holding_registers(slave_id, addr, count)

                if raw_regs is None:
                    continue

                if dtype == 'FLOAT32' and len(raw_regs) >= 2:
                    value = _regs_to_float(raw_regs[:2])
                elif dtype == 'UINT32' and len(raw_regs) >= 2:
                    value = (raw_regs[0] << 16) | raw_regs[1]
                else:
                    value = raw_regs[0]
                    if dtype == 'INT16' and value > 32767:
                        value = value - 65536

                sem = infer_semantics(addr, value)
                reg_name = meta.get('name', '') or sem['name']
                reg_unit = meta.get('unit', '') or sem['unit']

                reg_entry = {
                    'register_address': addr,
                    'function_code': fc,
                    'data_type': dtype,
                    'sample_value': round(value, 4),
                    'inferred_name': reg_name,
                    'inferred_unit': reg_unit,
                    'writable': meta.get('writable', False),
                    'range_min': sem['range_min'],
                    'range_max': sem['range_max'],
                    'valid': True,
                }
                registers.append(reg_entry)
                _add_log("info",
                         f"[DISCOVER]   [{addr}] {reg_name} = {round(value, 4)} "
                         f"{reg_unit} ({dtype}, {'R/W' if meta.get('writable') else 'R/O'})")
                time.sleep(0.01)
        else:
            # No metadata — heuristic scan address by address
            _add_log("info", f"[DISCOVER] No metadata for slave {slave_id}, using heuristic scan")
            addr = reg_start
            while addr <= reg_end:
                raw_regs_fc3 = modbus_bridge.read_holding_registers(slave_id, addr, 2)
                raw_regs_fc4 = modbus_bridge.read_input_registers(slave_id, addr, 2)

                fc = 3
                raw_regs = raw_regs_fc3
                if raw_regs_fc3 is None:
                    fc = 4
                    raw_regs = raw_regs_fc4
                elif all(v == 0 for v in raw_regs_fc3) and raw_regs_fc4 is not None:
                    if any(v != 0 for v in raw_regs_fc4):
                        fc = 4
                        raw_regs = raw_regs_fc4

                if raw_regs is None or len(raw_regs) == 0:
                    addr += 1
                    continue

                if all(v == 0 for v in raw_regs):
                    addr += 1
                    continue

                value = float(raw_regs[0])
                dtype = 'UINT16'
                skip = 1

                if len(raw_regs) >= 2 and raw_regs[0] != 0:
                    fval = _regs_to_float(raw_regs[:2])
                    if _is_valid_float(fval):
                        value = fval
                        dtype = 'FLOAT32'
                        skip = 2
                    elif raw_regs[0] > 32767:
                        dtype = 'UINT16'
                    else:
                        dtype = 'INT16'
                        value = raw_regs[0] if raw_regs[0] < 32768 else raw_regs[0] - 65536
                elif raw_regs[0] > 32767:
                    dtype = 'UINT16'
                else:
                    dtype = 'INT16'
                    value = raw_regs[0]

                sem = infer_semantics(addr, value)
                reg_entry = {
                    'register_address': addr,
                    'function_code': fc,
                    'data_type': dtype,
                    'sample_value': round(value, 4),
                    'inferred_name': sem['name'],
                    'inferred_unit': sem['unit'],
                    'writable': sem['writable'],
                    'range_min': sem['range_min'],
                    'range_max': sem['range_max'],
                    'valid': True,
                }
                registers.append(reg_entry)
                _add_log("info",
                         f"[DISCOVER]   [{addr}] {sem['name']} = {round(value, 4)} "
                         f"{sem['unit']} ({dtype}, {'R/W' if sem['writable'] else 'R/O'})")
                time.sleep(0.01)
                addr += skip

    else:
        # ---- Virtual bus register read ----
        while addr <= reg_end:
            if addr in dev.registers:
                reg_info = dev.registers[addr]
                raw = reg_info['raw']
                value = reg_info['value']
                dtype = reg_info['type']

                sem = infer_semantics(addr, value)

                if dtype == 'FLOAT32' and addr + 2 <= reg_end:
                    addr += 2
                else:
                    addr += 1

                reg_entry = {
                    'register_address': addr - (2 if dtype == 'FLOAT32' else 1),
                    'function_code': 3,
                    'data_type': dtype,
                    'sample_value': value,
                    'inferred_name': sem['name'],
                    'inferred_unit': sem['unit'],
                    'writable': reg_info['writable'] or sem['writable'],
                    'range_min': sem['range_min'],
                    'range_max': sem['range_max'],
                    'valid': True,
                }

                actual_addr = reg_entry['register_address']
                sem2 = infer_semantics(actual_addr, value)
                reg_entry['inferred_name'] = sem2['name']
                reg_entry['inferred_unit'] = sem2['unit']

                registers.append(reg_entry)
                _add_log("info",
                         f"[DISCOVER]   [{actual_addr}] {sem2['name']} = {value} "
                         f"{sem2['unit']} ({dtype}, {'R/W' if reg_entry['writable'] else 'R/O'})")
                time.sleep(0.02)
            else:
                addr += 1

    # Update device in result
    with _lock:
        for d in _result.devices:
            if d['slave_id'] == slave_id:
                d['registers'] = registers
                d['register_count'] = len(registers)
                break
        _result.registers_found += len(registers)

    _add_log("ok", f"[DISCOVER] Slave {slave_id}: {len(registers)} registers discovered "
             f"({mode_str})")
    return {'slave_id': slave_id, 'registers': len(registers)}


def full_scan(slave_start=1, slave_end=247, reg_start=40001, reg_end=40100):
    """Full scan: broadcast + register scan for all found devices."""
    _add_log("info", "[DISCOVER] Starting full scan...")

    # Phase 1: Broadcast
    broadcast_result = broadcast_scan(slave_start, slave_end)

    # Phase 2: Register scan for each device
    with _lock:
        devices_copy = list(_result.devices)

    _result.scan_in_progress = True
    for dev in devices_copy:
        scan_device(dev['slave_id'], reg_start, reg_end)

    _result.scan_complete = True
    _result.scan_in_progress = False
    _result.scan_end_time = time.time()

    _add_log("ok",
             f"[DISCOVER] Full scan complete: {_result.devices_found} devices, "
             f"{_result.registers_found} registers")
    return _result.to_dict()


def _full_scan_worker(slave_start, slave_end, reg_start, reg_end):
    """Background worker for async full scan."""
    try:
        _add_log("info", "[DISCOVER] Starting async full scan...")

        # Phase 1: Broadcast (devices are added incrementally)
        # broadcast_scan handles scan_in_progress internally
        broadcast_scan(slave_start, slave_end)

        # Phase 2: Register scan for each discovered device
        # Re-set scan_in_progress since broadcast_scan cleared it
        with _lock:
            _result.scan_in_progress = True
            _result.scan_complete = False

        while True:
            with _lock:
                # Find devices that haven't been register-scanned yet
                pending = [d for d in _result.devices if not d.get('registers')]
            if not pending:
                break
            dev = pending[0]
            scan_device(dev['slave_id'], reg_start, reg_end)

        with _lock:
            _result.scan_complete = True
            _result.scan_in_progress = False
            _result.scan_end_time = time.time()
        _add_log("ok",
                 f"[DISCOVER] Async full scan complete: {_result.devices_found} devices, "
                 f"{_result.registers_found} registers")
    except Exception as e:
        _add_log("error", f"[DISCOVER] Scan error: {e}")
        with _lock:
            _result.scan_in_progress = False
            _result.scan_complete = True
            _result.scan_end_time = time.time()


def start_full_scan_async(slave_start=1, slave_end=247, reg_start=40001, reg_end=40100):
    """Start a full scan in a background thread. Returns immediately."""
    with _lock:
        if _result.scan_in_progress:
            return {'status': 'already_running', 'scan_in_progress': True}

        # Reset state — broadcast_scan will set scan_in_progress
        _result.scan_start_time = time.time()
        _result.scan_end_time = None
        _result.scan_complete = False
        _result.devices = []
        _result.devices_found = 0
        _result.registers_found = 0

    t = threading.Thread(
        target=_full_scan_worker,
        args=(slave_start, slave_end, reg_start, reg_end),
        daemon=True,
    )
    t.start()

    return {
        'status': 'started',
        'scan_in_progress': True,
        'slave_start': slave_start,
        'slave_end': slave_end,
    }


def get_result():
    """Get current scan result summary."""
    return _result.to_dict()


def get_devices():
    """Get list of discovered devices with their registers."""
    return _result.devices_to_list()


def apply_mappings():
    """Apply all discovered registers as AMM mapping entries."""
    created = 0

    with _lock:
        devices = list(_result.devices)

    for dev in devices:
        if not dev.get('active'):
            continue

        for reg in dev.get('registers', []):
            if not reg.get('valid'):
                continue

            # Check for duplicates
            slave_id = dev['slave_id']
            reg_addr = reg['register_address']
            duplicate = any(
                m['slave_id'] == slave_id and m['register_address'] == reg_addr
                for m in _mappings
            )
            if duplicate:
                _add_log("info",
                         f"[AMM] Mapping already exists: slave {slave_id} / addr {reg_addr}")
                continue

            new_entry = {
                'slave_id': slave_id,
                'register_address': reg_addr,
                'data_type': reg['data_type'],
                'scale_factor': 1.0,
                'device_id': dev['device_id'],
                'point_id': reg['inferred_name'].replace(' ', '_').lower(),
                'measurement_name': reg['inferred_name'],
                'unit': reg['inferred_unit'],
                'mqtt_topic': f"factory/data/{dev['device_id']}/{reg['inferred_name'].replace(' ', '_').lower()}",
                'writable': reg.get('writable', False),
                'range_min': reg.get('range_min', -32768),
                'range_max': reg.get('range_max', 65535),
            }

            _mappings.append(new_entry)
            created += 1
            _add_log("ok",
                     f"[AMM] Auto-created mapping: {new_entry['device_id']}/"
                     f"{new_entry['point_id']} @ slave {slave_id} addr {reg_addr}")

    _result.mappings_created += created
    _add_log("ok", f"[DISCOVER] Applied {created} new AMM mapping entries")
    return {'mappings_created': created, 'total_mappings': len(_mappings)}


def reset():
    """Clear all discovery results."""
    global _result
    with _lock:
        _result = DiscoverResult()
    _add_log("info", "[DISCOVER] Discovery state reset")
    return {'status': 'ok'}


# ======================== Device & Register Editing ========================

def _find_discovered_device(slave_id):
    """Find a discovered device by slave_id."""
    for dev in _result.devices:
        if dev.get('slave_id') == slave_id and dev.get('active'):
            return dev
    return None


def _find_discovered_register(dev, reg_addr):
    """Find a register within a discovered device."""
    for reg in dev.get('registers', []):
        if reg.get('register_address') == reg_addr and reg.get('valid'):
            return reg
    return None


def update_device(slave_id, updates):
    """
    Update device-level properties.

    Args:
        slave_id: Target slave ID.
        updates: dict with optional keys:
            device_id (str), name (str), description (str),
            mqtt_topic_prefix (str).

    Returns:
        Updated device dict or error dict.
    """
    with _lock:
        dev = _find_discovered_device(slave_id)
        if not dev:
            return {'error': f'Device slave {slave_id} not found'}

        old_id = dev.get('device_id', '')
        if 'device_id' in updates and updates['device_id']:
            dev['device_id'] = updates['device_id']
        if 'name' in updates:
            dev['name'] = updates['name']
        if 'description' in updates:
            dev['description'] = updates['description']
        if 'mqtt_topic_prefix' in updates:
            dev['mqtt_topic_prefix'] = updates['mqtt_topic_prefix']
            # Update all register MQTT topics
            prefix = updates['mqtt_topic_prefix']
            for reg in dev.get('registers', []):
                point = reg.get('point_id') or reg.get('inferred_name', '').replace(' ', '_').lower()
                reg['mqtt_topic'] = f"{prefix}/{point}"

    new_id = dev.get('device_id', '')
    _add_log("info", f"[DISCOVER] Device updated: slave {slave_id}, "
             f"'{old_id}' -> '{new_id}'")
    return {'status': 'ok', 'device': dev}


def update_register(slave_id, reg_addr, updates):
    """
    Update register-level properties.

    Args:
        slave_id: Target slave ID.
        reg_addr: Register address.
        updates: dict with optional keys:
            inferred_name, inferred_unit, data_type, writable,
            range_min, range_max, scale_factor, point_id.

    Returns:
        Updated register dict or error dict.
    """
    with _lock:
        dev = _find_discovered_device(slave_id)
        if not dev:
            return {'error': f'Device slave {slave_id} not found'}

        reg = _find_discovered_register(dev, reg_addr)
        if not reg:
            return {'error': f'Register {reg_addr} not found on slave {slave_id}'}

        old_name = reg.get('inferred_name', '')
        # Accept both short names and full names for convenience
        name_map = {
            'name': 'inferred_name',
            'unit': 'inferred_unit',
        }
        for key in ('inferred_name', 'inferred_unit', 'data_type', 'writable',
                     'range_min', 'range_max', 'scale_factor', 'point_id'):
            if key in updates:
                reg[key] = updates[key]
        # Also accept short aliases
        for alias, full_key in name_map.items():
            if alias in updates and full_key not in updates:
                reg[full_key] = updates[alias]

        # Auto-generate point_id from name if not provided
        if 'inferred_name' in updates and 'point_id' not in updates:
            reg['point_id'] = reg['inferred_name'].replace(' ', '_').lower()

    _add_log("info", f"[DISCOVER] Register updated: slave {slave_id} addr {reg_addr}, "
             f"'{old_name}' -> '{reg.get('inferred_name', '')}'")
    return {'status': 'ok', 'register': reg}


def delete_register(slave_id, reg_addr):
    """
    Remove a single register from the discovered device.

    Returns:
        Success or error dict.
    """
    with _lock:
        dev = _find_discovered_device(slave_id)
        if not dev:
            return {'error': f'Device slave {slave_id} not found'}

        regs = dev.get('registers', [])
        for i, reg in enumerate(regs):
            if reg.get('register_address') == reg_addr:
                removed = regs.pop(i)
                dev['register_count'] = len(regs)
                _result.registers_found = max(0, _result.registers_found - 1)
                _add_log("warn", f"[DISCOVER] Register deleted: slave {slave_id} "
                         f"addr {reg_addr} ({removed.get('inferred_name', '?')})")
                return {'status': 'ok', 'deleted': removed}

    return {'error': f'Register {reg_addr} not found on slave {slave_id}'}


def toggle_register(slave_id, reg_addr):
    """
    Toggle a register's active/valid state (enable/disable).

    Returns:
        New state or error dict.
    """
    with _lock:
        dev = _find_discovered_device(slave_id)
        if not dev:
            return {'error': f'Device slave {slave_id} not found'}

        # Search all registers regardless of valid state
        reg = None
        for r in dev.get('registers', []):
            if r.get('register_address') == reg_addr:
                reg = r
                break
        if not reg:
            return {'error': f'Register {reg_addr} not found'}

        reg['valid'] = not reg.get('valid', True)
        state = 'enabled' if reg['valid'] else 'disabled'
        _add_log("info", f"[DISCOVER] Register {state}: slave {slave_id} "
                 f"addr {reg_addr} ({reg.get('inferred_name', '?')})")
        return {'status': 'ok', 'valid': reg['valid']}


def export_devices():
    """
    Export all discovered devices as a JSON-serializable list.
    Includes all device and register properties for backup/sharing.

    Returns:
        List of device dicts.
    """
    with _lock:
        export = []
        for dev in _result.devices:
            if not dev.get('active'):
                continue
            export_dev = {
                'slave_id': dev['slave_id'],
                'device_id': dev.get('device_id', ''),
                'name': dev.get('name', ''),
                'description': dev.get('description', ''),
                'mqtt_topic_prefix': dev.get('mqtt_topic_prefix',
                    f"factory/data/{dev.get('device_id', '')}"),
                'registers': [],
            }
            for reg in dev.get('registers', []):
                export_dev['registers'].append({
                    'register_address': reg['register_address'],
                    'function_code': reg.get('function_code', 3),
                    'data_type': reg.get('data_type', 'UINT16'),
                    'inferred_name': reg.get('inferred_name', ''),
                    'inferred_unit': reg.get('inferred_unit', ''),
                    'writable': reg.get('writable', False),
                    'valid': reg.get('valid', True),
                    'range_min': reg.get('range_min', -32768),
                    'range_max': reg.get('range_max', 65535),
                    'scale_factor': reg.get('scale_factor', 1.0),
                    'point_id': reg.get('point_id', ''),
                })
            export.append(export_dev)

    _add_log("info", f"[DISCOVER] Exported {len(export)} devices")
    return export


def import_devices(device_list):
    """
    Import device configurations from a JSON list.
    Replaces current discovery results with the imported data.

    Args:
        device_list: List of device dicts (same format as export_devices output).

    Returns:
        Import result summary dict.
    """
    if not isinstance(device_list, list):
        return {'error': 'Expected a JSON array of devices'}

    imported_devices = 0
    imported_registers = 0

    with _lock:
        _result.devices = []

        for dev_data in device_list:
            if 'slave_id' not in dev_data:
                continue

            device = {
                'slave_id': dev_data['slave_id'],
                'device_id': dev_data.get('device_id', f"device_slave_{dev_data['slave_id']:02d}"),
                'name': dev_data.get('name', dev_data.get('device_id', '')),
                'description': dev_data.get('description', ''),
                'mqtt_topic_prefix': dev_data.get('mqtt_topic_prefix', ''),
                'active': True,
                'registers': [],
                'register_count': 0,
            }

            for reg_data in dev_data.get('registers', []):
                if 'register_address' not in reg_data:
                    continue
                reg = {
                    'register_address': reg_data['register_address'],
                    'function_code': reg_data.get('function_code', 3),
                    'data_type': reg_data.get('data_type', 'UINT16'),
                    'sample_value': reg_data.get('sample_value', 0.0),
                    'inferred_name': reg_data.get('inferred_name', f"Register_{reg_data['register_address']}"),
                    'inferred_unit': reg_data.get('inferred_unit', ''),
                    'writable': reg_data.get('writable', False),
                    'valid': reg_data.get('valid', True),
                    'range_min': reg_data.get('range_min', -32768),
                    'range_max': reg_data.get('range_max', 65535),
                    'scale_factor': reg_data.get('scale_factor', 1.0),
                    'point_id': reg_data.get('point_id', ''),
                }
                if not reg['point_id']:
                    reg['point_id'] = reg['inferred_name'].replace(' ', '_').lower()
                device['registers'].append(reg)
                imported_registers += 1

            device['register_count'] = len(device['registers'])
            _result.devices.append(device)
            imported_devices += 1

        _result.devices_found = imported_devices
        _result.registers_found = imported_registers
        _result.scan_complete = True
        _result.scan_in_progress = False

    _add_log("ok", f"[DISCOVER] Imported {imported_devices} devices, "
             f"{imported_registers} registers")
    return {
        'status': 'ok',
        'devices_imported': imported_devices,
        'registers_imported': imported_registers,
    }


# ======================== Bridge Management ========================

def bridge_status():
    """Get MODBUS TCP bridge connection status."""
    if not _bridge_available:
        return {'available': False, 'connected': False,
                'error': 'modbus_bridge module not found'}
    status = modbus_bridge.get_status()
    status['available'] = True
    return status


def bridge_connect(host='localhost', port=5020):
    """Connect the MODBUS TCP bridge to a real server."""
    if not _bridge_available:
        return {'error': 'modbus_bridge module not available'}
    result = modbus_bridge.connect(host, port)
    if 'error' not in result:
        _add_log("ok", f"[BRIDGE] Connected to MODBUS TCP server at {host}:{port}")
    else:
        _add_log("warn", f"[BRIDGE] Connection failed to {host}:{port}: {result['error']}")
    return result


def bridge_disconnect():
    """Disconnect the MODBUS TCP bridge."""
    if not _bridge_available:
        return {'error': 'modbus_bridge module not available'}
    result = modbus_bridge.disconnect()
    _add_log("info", "[BRIDGE] Disconnected from MODBUS TCP server")
    return result
