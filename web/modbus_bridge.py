"""
MODBUS TCP Client Bridge
Allows the gateway simulation to connect to a real MODBUS TCP server
(e.g., the standalone device simulator) and perform actual reads/writes.
"""

import socket
import struct
import threading

_lock = threading.Lock()
_connected = False
_host = 'localhost'
_port = 5020
_trans_id = 0


def connect(host='localhost', port=5020):
    """Connect to a MODBUS TCP server."""
    global _connected, _host, _port
    _host = host
    _port = port
    # Quick connectivity test
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((host, port))
        sock.close()
        _connected = True
        return {'status': 'ok', 'host': host, 'port': port}
    except Exception as e:
        _connected = False
        return {'error': str(e)}


def disconnect():
    """Disconnect from the MODBUS TCP server."""
    global _connected
    _connected = False
    return {'status': 'ok'}


def is_connected():
    """Check if bridge is connected."""
    return _connected


def get_status():
    """Get bridge status."""
    return {
        'connected': _connected,
        'host': _host,
        'port': _port,
    }


def _next_trans_id():
    global _trans_id
    _trans_id = (_trans_id + 1) & 0xFFFF
    return _trans_id


def _modbus_tcp_request(slave_id, fc, data_bytes):
    """Send a MODBUS TCP request and return the response PDU (without MBAP)."""
    if not _connected:
        return None

    with _lock:
        trans_id = _next_trans_id()

    pdu = struct.pack('B', fc) + data_bytes
    mbap = struct.pack('>HHHB', trans_id, 0, len(pdu) + 1, slave_id)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    try:
        sock.connect((_host, _port))
        sock.sendall(mbap + pdu)

        # Read MBAP header (7 bytes)
        header = b''
        while len(header) < 7:
            chunk = sock.recv(7 - len(header))
            if not chunk:
                return None
            header += chunk

        _, resp_proto, resp_len, resp_unit = struct.unpack('>HHHB', header)

        # Read remaining PDU
        pdu_len = resp_len - 1  # subtract unit_id already read
        resp_pdu = b''
        while len(resp_pdu) < pdu_len:
            chunk = sock.recv(pdu_len - len(resp_pdu))
            if not chunk:
                return None
            resp_pdu += chunk

        resp_fc = resp_pdu[0]
        if resp_fc & 0x80:  # Exception
            exc_code = resp_pdu[1] if len(resp_pdu) > 1 else 0
            return {'exception': exc_code, 'fc': fc}

        return resp_pdu

    except (socket.timeout, ConnectionRefusedError, OSError):
        return None
    finally:
        sock.close()


def read_holding_registers(slave_id, start_addr, count):
    """
    FC03: Read holding registers.
    Returns list of 16-bit values, or None on error.
    """
    data = struct.pack('>HH', start_addr, count)
    resp = _modbus_tcp_request(slave_id, 0x03, data)

    if resp is None or isinstance(resp, dict):
        return None

    fc = resp[0]
    byte_count = resp[1]
    values = []
    for i in range(0, byte_count, 2):
        val = struct.unpack('>H', resp[2 + i:4 + i])[0]
        values.append(val)
    return values


def read_input_registers(slave_id, start_addr, count):
    """
    FC04: Read input registers.
    Returns list of 16-bit values, or None on error.
    """
    data = struct.pack('>HH', start_addr, count)
    resp = _modbus_tcp_request(slave_id, 0x04, data)

    if resp is None or isinstance(resp, dict):
        return None

    fc = resp[0]
    byte_count = resp[1]
    values = []
    for i in range(0, byte_count, 2):
        val = struct.unpack('>H', resp[2 + i:4 + i])[0]
        values.append(val)
    return values


def write_single_register(slave_id, address, value):
    """FC06: Write a single register. Returns True on success."""
    data = struct.pack('>HH', address, value)
    resp = _modbus_tcp_request(slave_id, 0x06, data)
    if resp is None or isinstance(resp, dict):
        return False
    return True


def write_multiple_registers(slave_id, start_addr, values):
    """FC16: Write multiple registers. Returns True on success."""
    count = len(values)
    byte_count = count * 2
    data = struct.pack('>HHB', start_addr, count, byte_count)
    for v in values:
        data += struct.pack('>H', v & 0xFFFF)
    resp = _modbus_tcp_request(slave_id, 0x10, data)
    if resp is None or isinstance(resp, dict):
        return False
    return True


def scan_bus(slave_start=1, slave_end=10, timeout=2):
    """
    Quick scan: probe slave IDs with FC03 addr=0 count=1.
    Returns list of responding slave IDs.
    """
    found = []
    for sid in range(slave_start, slave_end + 1):
        result = read_holding_registers(sid, 0, 1)
        if result is not None:
            found.append(sid)
    return found


def scan_device_registers(slave_id, reg_start=40001, reg_end=40100, timeout=2):
    """
    Fine scan: probe register range for a specific slave.
    Returns list of dicts with register info.
    """
    registers = []
    addr = reg_start
    while addr <= reg_end:
        # Try reading 1 register with FC03 (holding)
        result = read_holding_registers(slave_id, addr, 1)
        if result is not None:
            registers.append({
                'address': addr,
                'function_code': 3,
                'raw_value': result[0],
            })
        else:
            # Try FC04 (input)
            result = read_input_registers(slave_id, addr, 1)
            if result is not None:
                registers.append({
                    'address': addr,
                    'function_code': 4,
                    'raw_value': result[0],
                })
        addr += 1
    return registers
