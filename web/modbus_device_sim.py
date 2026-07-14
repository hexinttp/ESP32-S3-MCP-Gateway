#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone MODBUS TCP device simulator for the gateway web demo.

Ports:
  5020 - MODBUS TCP server
  8081 - metadata HTTP API used by discover_sim.py
"""

import argparse
import json
import socketserver
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import discover_sim


STATE_LOCK = threading.RLock()


def _device_by_slave(slave_id):
    for dev in discover_sim.VIRTUAL_BUS:
        if dev.slave_id == slave_id:
            return dev
    return None


def _flat_registers(dev):
    flat = {}
    for base_addr, reg in dev.registers.items():
        for offset, raw in enumerate(reg["raw"]):
            flat[base_addr + offset] = raw & 0xFFFF
    return flat


def _register_metadata(dev):
    regs = []
    for addr, reg in sorted(dev.registers.items()):
        regs.append({
            "address": addr,
            "function_code": 3,
            "data_type": reg["type"],
            "name": f"{dev.name} {addr}",
            "unit": _unit_for_address(addr),
            "writable": bool(reg["writable"]),
            "value": reg["value"],
            "raw": reg["raw"],
        })
    return regs


def _raw_from_value(value, dtype):
    dtype = (dtype or "UINT16").upper()
    if dtype == "FLOAT32":
        return discover_sim._float_to_regs(float(value))
    if dtype == "INT16":
        return [int(value) & 0xFFFF]
    return [int(value) & 0xFFFF]


def _set_register(dev, addr, value, dtype, writable):
    dtype = (dtype or "UINT16").upper()
    dev.registers[int(addr)] = {
        "value": float(value) if dtype == "FLOAT32" else int(value),
        "type": dtype,
        "writable": bool(writable),
        "raw": _raw_from_value(value, dtype),
    }


def _unit_for_address(addr):
    if 40001 <= addr <= 40099:
        return "degC"
    if 40101 <= addr <= 40199:
        return "bar"
    if 40201 <= addr <= 40299:
        return "L/min"
    if 40301 <= addr <= 40399:
        return "rpm"
    if 40401 <= addr <= 40499:
        return "V"
    if 40501 <= addr <= 40599:
        return "%RH"
    if 40601 <= addr <= 40699:
        return "mm"
    return ""


def _write_register(dev, addr, value):
    for base_addr, reg in dev.registers.items():
        raw = reg["raw"]
        if base_addr <= addr < base_addr + len(raw):
            if not reg["writable"]:
                return False
            raw[addr - base_addr] = value & 0xFFFF
            if reg["type"] == "FLOAT32" and len(raw) >= 2:
                reg["value"] = struct.unpack(">f", struct.pack(">HH", raw[0], raw[1]))[0]
            elif reg["type"] == "INT16":
                reg["value"] = struct.unpack(">h", struct.pack(">H", raw[0]))[0]
            else:
                reg["value"] = raw[0]
            return True
    return False


class ModbusTcpHandler(socketserver.BaseRequestHandler):
    def handle(self):
        try:
            header = self._recv_exact(7)
            if not header:
                return
            trans_id, proto_id, length, unit_id = struct.unpack(">HHHB", header)
            pdu = self._recv_exact(length - 1)
            if not pdu:
                return

            fc = pdu[0]
            response_pdu = self._handle_pdu(unit_id, fc, pdu[1:])
            response = struct.pack(">HHHB", trans_id, proto_id, len(response_pdu) + 1, unit_id)
            self.request.sendall(response + response_pdu)
        except Exception:
            return

    def _recv_exact(self, size):
        buf = b""
        while len(buf) < size:
            chunk = self.request.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _exception(self, fc, code):
        return struct.pack("BB", fc | 0x80, code)

    def _handle_pdu(self, slave_id, fc, payload):
        with STATE_LOCK:
            dev = _device_by_slave(slave_id)
            if dev is None:
                return self._exception(fc, 0x0B)

            if fc in (0x03, 0x04):
                if len(payload) < 4:
                    return self._exception(fc, 0x03)
                start, count = struct.unpack(">HH", payload[:4])
                flat = _flat_registers(dev)
                values = []
                for addr in range(start, start + count):
                    if addr not in flat:
                        return self._exception(fc, 0x02)
                    values.append(flat[addr])
                data = b"".join(struct.pack(">H", v) for v in values)
                return struct.pack("BB", fc, len(data)) + data

            if fc == 0x06:
                if len(payload) < 4:
                    return self._exception(fc, 0x03)
                addr, value = struct.unpack(">HH", payload[:4])
                if not _write_register(dev, addr, value):
                    return self._exception(fc, 0x03)
                return struct.pack(">BHH", fc, addr, value)

            if fc == 0x10:
                if len(payload) < 5:
                    return self._exception(fc, 0x03)
                start, count, byte_count = struct.unpack(">HHB", payload[:5])
                data = payload[5:5 + byte_count]
                if len(data) != byte_count or byte_count != count * 2:
                    return self._exception(fc, 0x03)
                values = list(struct.unpack(">" + "H" * count, data))
                for offset, value in enumerate(values):
                    if not _write_register(dev, start + offset, value):
                        return self._exception(fc, 0x03)
                return struct.pack(">BHH", fc, start, count)

            return self._exception(fc, 0x01)


class MetadataHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _send_json(self, obj, status=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_html(self, html, status=200):
        data = html.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def do_GET(self):
        path = urlparse(self.path).path.rstrip("/")
        with STATE_LOCK:
            if path in ("", "/"):
                cards = []
                for dev in discover_sim.VIRTUAL_BUS:
                    rows = []
                    for reg in _register_metadata(dev):
                        rows.append(
                            "<tr>"
                            f"<td>{reg['address']}</td>"
                            f"<td>{reg['data_type']}</td>"
                            f"<td><input id=\"val-{dev.slave_id}-{reg['address']}\" type=\"number\" step=\"0.01\" value=\"{reg['value']}\"></td>"
                            f"<td>{reg['unit']}</td>"
                            f"<td>{'Yes' if reg['writable'] else 'No'}</td>"
                            "<td>"
                            f"<button onclick=\"updateRegister({dev.slave_id},{reg['address']})\">Save</button> "
                            f"<button class=\"danger\" onclick=\"deleteRegister({dev.slave_id},{reg['address']})\">Delete</button>"
                            "</td>"
                            "</tr>"
                        )
                    cards.append(
                        "<section class=\"card\">"
                        f"<h2>Slave {dev.slave_id}: {dev.name} "
                        f"<button class=\"danger right\" onclick=\"deleteDevice({dev.slave_id})\">Delete Device</button></h2>"
                        "<table><thead><tr><th>Register</th><th>Type</th><th>Value</th>"
                        "<th>Unit</th><th>Writable</th><th>Actions</th></tr></thead>"
                        f"<tbody>{''.join(rows)}</tbody></table>"
                        f"<div class=\"inline-form\">"
                        f"<input id=\"addr-{dev.slave_id}\" type=\"number\" placeholder=\"Register\" value=\"40001\">"
                        f"<select id=\"type-{dev.slave_id}\"><option>FLOAT32</option><option>UINT16</option><option>INT16</option></select>"
                        f"<input id=\"newval-{dev.slave_id}\" type=\"number\" step=\"0.01\" placeholder=\"Value\" value=\"0\">"
                        f"<label><input id=\"wr-{dev.slave_id}\" type=\"checkbox\"> Writable</label>"
                        f"<button onclick=\"addRegister({dev.slave_id})\">Add Register</button>"
                        f"</div>"
                        "</section>"
                    )
                self._send_html(f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MODBUS Device Simulator</title>
<style>
body {{ margin:0; font-family:Segoe UI, Arial, sans-serif; background:#f3f5f7; color:#1f2937; }}
header {{ background:#111827; color:white; padding:18px 24px; }}
h1 {{ margin:0; font-size:20px; }}
.sub {{ margin-top:6px; color:#cbd5e1; font-size:13px; }}
main {{ padding:20px; max-width:1100px; margin:auto; }}
.status {{ display:flex; gap:12px; flex-wrap:wrap; margin-bottom:16px; }}
.pill {{ background:white; border:1px solid #d9dee7; border-radius:6px; padding:8px 12px; font-size:13px; }}
.card {{ background:white; border:1px solid #d9dee7; border-radius:8px; margin:14px 0; overflow:hidden; }}
h2 {{ margin:0; padding:12px 14px; font-size:15px; background:#f8fafc; border-bottom:1px solid #e5e7eb; }}
table {{ width:100%; border-collapse:collapse; font-size:13px; }}
th, td {{ padding:9px 12px; border-bottom:1px solid #edf0f4; text-align:left; }}
th {{ color:#64748b; font-size:12px; text-transform:uppercase; }}
a {{ color:#2563eb; }}
input, select {{ border:1px solid #cbd5e1; border-radius:5px; padding:6px 8px; font:inherit; }}
td input {{ width:110px; }}
button {{ border:1px solid #2563eb; background:#2563eb; color:white; border-radius:5px; padding:6px 9px; cursor:pointer; }}
button.danger {{ border-color:#dc2626; background:#dc2626; }}
.right {{ float:right; margin-top:-3px; }}
.inline-form, .add-device {{ display:flex; gap:8px; align-items:center; flex-wrap:wrap; padding:12px 14px; background:#fbfdff; }}
.add-device {{ margin-bottom:14px; border:1px solid #d9dee7; border-radius:8px; background:white; }}
</style>
</head>
<body>
<header>
  <h1>MODBUS TCP Device Simulator</h1>
  <div class="sub">Virtual MODBUS TCP server for ESP32-S3 gateway testing</div>
</header>
<main>
  <div class="status">
    <div class="pill">MODBUS TCP: <strong>localhost:5020</strong></div>
    <div class="pill">Metadata API: <strong>localhost:8081</strong></div>
    <div class="pill"><a href="/api/devices">/api/devices</a></div>
  </div>
  <section class="add-device">
    <strong>Add Device</strong>
    <input id="newSlave" type="number" placeholder="Slave ID" min="1" max="247">
    <input id="newName" type="text" placeholder="Device name">
    <button onclick="addDevice()">Add Device</button>
  </section>
  {''.join(cards)}
</main>
<script>
async function api(method, path, body) {{
  const res = await fetch(path, {{
    method,
    headers: {{ 'Content-Type': 'application/json' }},
    body: body ? JSON.stringify(body) : undefined
  }});
  const data = await res.json().catch(() => ({{}}));
  if (!res.ok || data.error) throw new Error(data.error || res.statusText);
  return data;
}}
async function addDevice() {{
  const slave_id = Number(document.getElementById('newSlave').value);
  const name = document.getElementById('newName').value.trim() || `Device_${{slave_id}}`;
  await api('POST', '/api/devices', {{ slave_id, name }});
  location.reload();
}}
async function deleteDevice(slave_id) {{
  if (!confirm(`Delete slave ${{slave_id}}?`)) return;
  await api('DELETE', `/api/devices/${{slave_id}}`);
  location.reload();
}}
async function addRegister(slave_id) {{
  await api('POST', `/api/devices/${{slave_id}}/registers`, {{
    address: Number(document.getElementById(`addr-${{slave_id}}`).value),
    data_type: document.getElementById(`type-${{slave_id}}`).value,
    value: Number(document.getElementById(`newval-${{slave_id}}`).value),
    writable: document.getElementById(`wr-${{slave_id}}`).checked
  }});
  location.reload();
}}
async function updateRegister(slave_id, address) {{
  await api('PUT', `/api/devices/${{slave_id}}/registers/${{address}}`, {{
    value: Number(document.getElementById(`val-${{slave_id}}-${{address}}`).value)
  }});
  location.reload();
}}
async function deleteRegister(slave_id, address) {{
  if (!confirm(`Delete register ${{address}} from slave ${{slave_id}}?`)) return;
  await api('DELETE', `/api/devices/${{slave_id}}/registers/${{address}}`);
  location.reload();
}}
</script>
</body>
</html>""")
                return

            if path == "/api/devices":
                self._send_json([
                    {
                        "slave_id": dev.slave_id,
                        "name": dev.name,
                        "register_count": len(dev.registers),
                        "registers": _register_metadata(dev),
                    }
                    for dev in discover_sim.VIRTUAL_BUS
                ])
                return

            if path.startswith("/api/devices/"):
                try:
                    slave_id = int(path.rsplit("/", 1)[-1])
                except ValueError:
                    self._send_json({"error": "invalid slave id"}, 400)
                    return
                dev = _device_by_slave(slave_id)
                if dev is None:
                    self._send_json({"error": "device not found"}, 404)
                    return
                self._send_json({
                    "slave_id": dev.slave_id,
                    "name": dev.name,
                    "register_count": len(dev.registers),
                    "registers": _register_metadata(dev),
                })
                return

            self._send_json({"error": "not found"}, 404)

    def do_POST(self):
        path = urlparse(self.path).path.rstrip("/")
        try:
            body = self._read_json()
            with STATE_LOCK:
                if path == "/api/devices":
                    slave_id = int(body.get("slave_id", 0))
                    name = str(body.get("name") or f"Device_{slave_id}")
                    if slave_id < 1 or slave_id > 247:
                        self._send_json({"error": "slave_id must be 1..247"}, 400)
                        return
                    if _device_by_slave(slave_id):
                        self._send_json({"error": "slave_id already exists"}, 409)
                        return
                    discover_sim.VIRTUAL_BUS.append(discover_sim.VirtualModbusDevice(slave_id, name, {}))
                    discover_sim.VIRTUAL_BUS.sort(key=lambda d: d.slave_id)
                    self._send_json({"status": "ok"})
                    return

                if path.startswith("/api/devices/") and path.endswith("/registers"):
                    parts = path.split("/")
                    slave_id = int(parts[3])
                    dev = _device_by_slave(slave_id)
                    if not dev:
                        self._send_json({"error": "device not found"}, 404)
                        return
                    _set_register(dev, int(body["address"]), body.get("value", 0),
                                  body.get("data_type", "UINT16"), body.get("writable", False))
                    self._send_json({"status": "ok"})
                    return

            self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, 400)

    def do_PUT(self):
        path = urlparse(self.path).path.rstrip("/")
        try:
            body = self._read_json()
            parts = path.split("/")
            with STATE_LOCK:
                if len(parts) == 6 and parts[1] == "api" and parts[2] == "devices" and parts[4] == "registers":
                    slave_id = int(parts[3])
                    addr = int(parts[5])
                    dev = _device_by_slave(slave_id)
                    if not dev or addr not in dev.registers:
                        self._send_json({"error": "register not found"}, 404)
                        return
                    reg = dev.registers[addr]
                    _set_register(dev, addr, body.get("value", reg["value"]),
                                  body.get("data_type", reg["type"]),
                                  body.get("writable", reg["writable"]))
                    self._send_json({"status": "ok"})
                    return

            self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, 400)

    def do_DELETE(self):
        path = urlparse(self.path).path.rstrip("/")
        try:
            parts = path.split("/")
            with STATE_LOCK:
                if len(parts) == 4 and parts[1] == "api" and parts[2] == "devices":
                    slave_id = int(parts[3])
                    before = len(discover_sim.VIRTUAL_BUS)
                    discover_sim.VIRTUAL_BUS[:] = [d for d in discover_sim.VIRTUAL_BUS if d.slave_id != slave_id]
                    self._send_json({"status": "ok"} if len(discover_sim.VIRTUAL_BUS) != before else {"error": "device not found"},
                                    200 if len(discover_sim.VIRTUAL_BUS) != before else 404)
                    return

                if len(parts) == 6 and parts[1] == "api" and parts[2] == "devices" and parts[4] == "registers":
                    slave_id = int(parts[3])
                    addr = int(parts[5])
                    dev = _device_by_slave(slave_id)
                    if not dev or addr not in dev.registers:
                        self._send_json({"error": "register not found"}, 404)
                        return
                    del dev.registers[addr]
                    self._send_json({"status": "ok"})
                    return

            self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, 400)


def main():
    parser = argparse.ArgumentParser(description="Virtual MODBUS TCP device simulator")
    parser.add_argument("--modbus-port", type=int, default=5020)
    parser.add_argument("--http-port", type=int, default=8081)
    args = parser.parse_args()

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    modbus_server = socketserver.ThreadingTCPServer(("0.0.0.0", args.modbus_port), ModbusTcpHandler)
    http_server = ThreadingHTTPServer(("0.0.0.0", args.http_port), MetadataHandler)

    threading.Thread(target=modbus_server.serve_forever, daemon=True).start()
    print(f"[MODBUS-SIM] MODBUS TCP listening on 0.0.0.0:{args.modbus_port}", flush=True)
    print(f"[MODBUS-SIM] Metadata API listening on http://localhost:{args.http_port}", flush=True)
    print("[MODBUS-SIM] Virtual slaves: " + ", ".join(str(d.slave_id) for d in discover_sim.VIRTUAL_BUS), flush=True)

    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        modbus_server.shutdown()
        http_server.shutdown()


if __name__ == "__main__":
    main()
