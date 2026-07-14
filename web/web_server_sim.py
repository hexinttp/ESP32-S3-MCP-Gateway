#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC Simulation Web Server for ESP32-S3 Gateway Configuration.

Serves the web_config.html page and provides mock REST API endpoints
that mirror the ESP-IDF esp_http_server implementation.

Usage:
    python web_server_sim.py [--port 8080]

Then open http://localhost:8080 in your browser.
"""

import json
import os
import sys
import time
import argparse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import threading

# ---- Import discovery module ----
import discover_sim

# ---- Configuration Defaults (matching gateway_config.h) ----
CONFIG = {
    "wifi": {
        "ssid": "MyHomeNetwork",
        "password": "••••••••",
        "auth_mode": 3,
        "ip": "192.168.1.105"
    },
    "mqtt": {
        "uri": "mqtt://192.168.1.100:1883",
        "client_id": "esp32s3_gateway_01",
        "username": "",
        "password": "",
        "keepalive": 120,
        "qos": 1,
        "topic_prefix": "factory/data",
        "cloud_platform": "standard"
    },
    "onenet": {
        "enabled": False,
        "product_id": "",
        "device_id": "",
        "device_key": "",
        "mqtt_uri": "mqtts://mqtt.heclouds.com:8883"
    },
    "modbus": {
        "uart_port": 1,
        "baudrate": 9600,
        "parity": "none",
        "data_bits": 8,
        "stop_bits": 1,
        "timeout": 1000,
        "tx_pin": 17,
        "rx_pin": 18,
        "rts_pin": 19,
        "poll_interval": 1000
    }
}

# ---- AMM Mapping Table (mirrors the C module's default entries) ----
MAPPINGS = [
    {
        "slave_id": 1, "register_address": 40001,
        "data_type": "FLOAT32", "scale_factor": 1.0,
        "device_id": "plc_line1_01", "point_id": "motor_temp_01",
        "measurement_name": "Motor temperature", "unit": "degC",
        "mqtt_topic": "factory/line1/plc01/motor/temp",
        "writable": False, "range_min": 0, "range_max": 120
    },
    {
        "slave_id": 1, "register_address": 40003,
        "data_type": "FLOAT32", "scale_factor": 1.0,
        "device_id": "plc_line1_01", "point_id": "pressure_01",
        "measurement_name": "Line pressure", "unit": "bar",
        "mqtt_topic": "factory/line1/plc01/pressure",
        "writable": False, "range_min": 0, "range_max": 10
    },
    {
        "slave_id": 2, "register_address": 40001,
        "data_type": "UINT16", "scale_factor": 1.0,
        "device_id": "plc_line2_01", "point_id": "speed_01",
        "measurement_name": "Conveyor speed", "unit": "rpm",
        "mqtt_topic": "factory/line2/plc01/speed",
        "writable": True, "range_min": 0, "range_max": 3000
    },
    {
        "slave_id": 2, "register_address": 40003,
        "data_type": "INT16", "scale_factor": 1.0,
        "device_id": "plc_line2_01", "point_id": "current_01",
        "measurement_name": "Motor current", "unit": "A",
        "mqtt_topic": "factory/line2/plc01/current",
        "writable": False, "range_min": -50, "range_max": 50
    },
]

# ---- Simulated System Metrics (updated over time) ----
START_TIME = time.time()
METRICS = {
    "total_polls": 61, "successful_polls": 61, "failed_polls": 0,
    "contexts_created": 61, "contexts_validated": 61, "contexts_rejected": 0,
    "mqtt_published": 41, "mqtt_failed": 0,
    "cached_records": 20, "replayed_records": 20, "data_loss": 0,
    "commands_received": 4, "commands_accepted": 1, "commands_rejected": 3,
    "sequence_counter": 61,
}

# ---- Log Buffer ----
LOGS = []
LOG_LOCK = threading.Lock()

def add_log(level, text):
    ts = time.strftime("%H:%M:%S")
    with LOG_LOCK:
        LOGS.append({"level": level, "text": f"[{ts}] {text}"})
        if len(LOGS) > 500:
            LOGS.pop(0)

add_log("info", "[WEB] Simulation web server starting...")
add_log("ok", "[INIT] All modules initialized successfully.")
add_log("ok", "[INIT] Active AMM mappings: 4")
add_log("ok", "[INIT] MQTT connected: YES")
add_log("info", "[TCM] Built context: id=1 seq=1 dev=temp_sensor_01 pt=PT100_inlet val=72.50")
add_log("ok", "[TCM] Validation PASSED for context_id=1")
add_log("info", "[MQTT] Published to factory/line1/plc01/motor/temp (488 bytes)")
add_log("warn", "[TCM] Lookup miss: slave=2 addr=40003, using defaults")
add_log("info", "[UIF] Replaying 20 pending records")
add_log("ok", "[UIF] Replay complete: 20 sent, 0 failed, 0 remaining pending")
add_log("warn", "[AMM] Command rejected: Point 'plc_line1_01/motor_temp_01' is read-only")

# ---- Simulated poll counter (increments every second in background) ----
def background_poll_simulator():
    """Simulates ongoing MODBUS polling that updates metrics."""
    global METRICS
    poll_points = [
        ("plc_line1_01", "motor_temp_01", "factory/line1/plc01/motor/temp", 72.5),
        ("plc_line1_01", "pressure_01", "factory/line1/plc01/pressure", 3.82),
        ("plc_line2_01", "speed_01", "factory/line2/plc01/speed", 1500),
        ("plc_line2_01", "current_01", "factory/line2/plc01/current", 12.4),
    ]
    seq = METRICS["sequence_counter"]
    while True:
        time.sleep(3)
        METRICS["total_polls"] += 4
        METRICS["successful_polls"] += 4
        METRICS["contexts_created"] += 4
        METRICS["contexts_validated"] += 4
        METRICS["mqtt_published"] += 4
        METRICS["sequence_counter"] += 4
        seq = METRICS["sequence_counter"]
        # Add periodic log entries
        dev, pt, topic, val = poll_points[seq % len(poll_points)]
        add_log("info", f"[TCM] Built context: seq={seq} dev={dev} pt={pt} val={val}")
        add_log("ok", f"[TCM] Validation PASSED seq={seq}")
        add_log("info", f"[MQTT] Published to {topic} ({480 + seq % 40} bytes)")

# ---- HTTP Request Handler ----
class GatewayHandler(SimpleHTTPRequestHandler):
    """Handles both static file serving and REST API endpoints."""
    protocol_version = "HTTP/1.0"  # Avoid keep-alive issues with single-threaded server

    def log_message(self, format, *args):
        """Override to add to our log buffer instead of just stderr."""
        msg = format % args
        add_log("info", f"[HTTP] {msg}")

    def _send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, PUT, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length', 0))
        if length == 0:
            return None
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            return None

    # ---- GET routes ----
    def do_GET(self):
        path = urlparse(self.path).path

        if path == '/' or path == '/index.html':
            html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      'web_config.html')
            try:
                with open(html_path, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(content)
            except FileNotFoundError:
                self.send_error(404, "web_config.html not found")
            return

        if path == '/api/system/status':
            data = {
                "mqtt_connected": True,
                "modbus_active": True,
                "free_heap": 200000,
                "flash_size": 2097152,
                "uptime_seconds": int(time.time() - START_TIME),
            }
            data.update(METRICS)
            self._send_json(data)
            return

        if path == '/api/wifi/config':
            self._send_json(CONFIG["wifi"])
            return

        if path == '/api/mqtt/config':
            self._send_json(CONFIG["mqtt"])
            return

        if path == '/api/modbus/config':
            self._send_json(CONFIG["modbus"])
            return

        if path == '/api/onenet/config':
            self._send_json(CONFIG["onenet"])
            return

        if path == '/api/mappings':
            self._send_json(MAPPINGS)
            return

        # ---- Discovery endpoints ----
        if path == '/api/discover/status':
            self._send_json(discover_sim.get_result())
            return

        if path == '/api/discover/devices':
            self._send_json(discover_sim.get_devices())
            return

        if path == '/api/system/logs' or path == '/api/logs':
            # Copy logs under lock, then release before sending response
            # (sending response triggers log_message which also needs LOG_LOCK)
            with LOG_LOCK:
                logs_copy = list(LOGS)
            self._send_json(logs_copy)
            return

        # ---- Bridge status (GET) ----
        if path == '/api/bridge/status':
            result = discover_sim.bridge_status()
            self._send_json(result)
            return

        # Static files
        super().do_GET()

    # ---- PUT routes ----
    def do_PUT(self):
        path = urlparse(self.path).path
        body = self._read_json_body()

        if path == '/api/wifi/config' and body:
            CONFIG["wifi"].update(body)
            add_log("info", f"[WiFi] Config updated via web: SSID={body.get('ssid', '?')}")
            self._send_json({"status": "ok"})
            return

        if path == '/api/mqtt/config' and body:
            CONFIG["mqtt"].update(body)
            add_log("info", f"[MQTT] Config updated via web: URI={body.get('uri', '?')}")
            self._send_json({"status": "ok"})
            return

        if path == '/api/modbus/config' and body:
            CONFIG["modbus"].update(body)
            add_log("info", f"[MODBUS] Config updated via web: baud={body.get('baudrate', '?')}")
            self._send_json({"status": "ok"})
            return

        if path == '/api/onenet/config' and body:
            CONFIG["onenet"].update(body)
            enabled = body.get('enabled', False)
            if enabled:
                CONFIG["mqtt"]["cloud_platform"] = "onenet"
                add_log("ok", f"[OneNET] Enabled with product={body.get('product_id', '?')}")
            else:
                CONFIG["mqtt"]["cloud_platform"] = "standard"
                add_log("info", "[OneNET] Disabled, switching to standard MQTT")
            self._send_json({"status": "ok"})
            return

        # PUT /api/mappings/<idx> — update mapping
        if path.startswith('/api/mappings/') and body:
            try:
                idx = int(path.split('/')[-1])
                if 0 <= idx < len(MAPPINGS):
                    MAPPINGS[idx] = body
                    add_log("info", f"[AMM] Mapping updated via web: idx={idx}")
                    self._send_json({"status": "ok"})
                    return
            except (ValueError, IndexError):
                pass

        # PUT /api/discover/devices/<slave_id> — update device info
        if path.startswith('/api/discover/devices/') and body and '/registers/' not in path:
            try:
                slave_id = int(path.split('/')[-1])
                result = discover_sim.update_device(slave_id, body)
                self._send_json(result)
                return
            except Exception as e:
                sys.stderr.write(f"[PUT device] ERROR: {e}\n")
                sys.stderr.flush()
                self._send_json({'error': str(e)})
                return

        # PUT /api/discover/devices/<slave_id>/registers/<reg_addr>
        if '/api/discover/devices/' in path and '/registers/' in path and body:
            parts = path.split('/')
            try:
                slave_id = int(parts[4])
                reg_addr = int(parts[6])
                result = discover_sim.update_register(slave_id, reg_addr, body)
                self._send_json(result)
                return
            except Exception as e:
                sys.stderr.write(f"[PUT register] ERROR: {e}\n")
                sys.stderr.flush()
                self._send_json({'error': str(e)})
                return

        self.send_error(400)

    # ---- POST routes ----
    def do_POST(self):
        path = urlparse(self.path).path
        body = self._read_json_body()

        if path == '/api/mappings' and body:
            MAPPINGS.append(body)
            dev = body.get('device_id', '?')
            pt = body.get('point_id', '?')
            add_log("ok", f"[AMM] New mapping added via web: {dev}/{pt}")
            METRICS["sequence_counter"] += 1
            self._send_json({"status": "ok", "index": len(MAPPINGS) - 1})
            return

        # ---- Discovery POST endpoints ----
        if path == '/api/discover/scan':
            params = body or {}
            s_start = params.get('slave_start', 1)
            s_end   = params.get('slave_end', 247)
            r_start = params.get('reg_start', 40001)
            r_end   = params.get('reg_end', 40100)
            result = discover_sim.start_full_scan_async(s_start, s_end, r_start, r_end)
            self._send_json(result)
            return

        if path == '/api/discover/scan_device':
            params = body or {}
            slave_id = params.get('slave_id', 1)
            r_start = params.get('reg_start', 40001)
            r_end   = params.get('reg_end', 40100)
            result = discover_sim.scan_device(slave_id, r_start, r_end)
            self._send_json(result)
            return

        if path == '/api/discover/apply':
            result = discover_sim.apply_mappings()
            self._send_json(result)
            return

        if path == '/api/discover/reset':
            result = discover_sim.reset()
            self._send_json(result)
            return

        if path == '/api/discover/export':
            result = discover_sim.export_devices()
            self._send_json(result)
            return

        if path == '/api/discover/import' and body:
            result = discover_sim.import_devices(body)
            self._send_json(result)
            return

        # ---- Bridge management endpoints ----
        if path == '/api/bridge/connect' and body:
            host = body.get('host', 'localhost')
            port = body.get('port', 5020)
            result = discover_sim.bridge_connect(host, int(port))
            self._send_json(result)
            return

        if path == '/api/bridge/disconnect':
            result = discover_sim.bridge_disconnect()
            self._send_json(result)
            return

        # POST /api/discover/devices/<slave_id>/registers/<reg_addr>/toggle
        if '/api/discover/devices/' in path and path.endswith('/toggle'):
            parts = path.split('/')
            try:
                slave_id = int(parts[4])
                reg_addr = int(parts[6])
                result = discover_sim.toggle_register(slave_id, reg_addr)
                self._send_json(result)
                return
            except Exception as e:
                sys.stderr.write(f"[POST toggle] ERROR: {e}\n")
                sys.stderr.flush()
                self._send_json({'error': str(e)})
                return

        self.send_error(400)

    # ---- DELETE routes ----
    def do_DELETE(self):
        path = urlparse(self.path).path
        body = self._read_json_body()

        if path.startswith('/api/mappings/'):
            try:
                idx = int(path.split('/')[-1])
                if 0 <= idx < len(MAPPINGS):
                    removed = MAPPINGS.pop(idx)
                    add_log("warn", f"[AMM] Mapping deleted via web: "
                                    f"{removed.get('device_id', '?')}/{removed.get('point_id', '?')}")
                    self._send_json({"status": "ok"})
                    return
            except (ValueError, IndexError):
                pass

        # DELETE /api/discover/devices/<slave_id>/registers/<reg_addr>
        if '/api/discover/devices/' in path and '/registers/' in path:
            parts = path.split('/')
            try:
                slave_id = int(parts[4])
                reg_addr = int(parts[6])
                result = discover_sim.delete_register(slave_id, reg_addr)
                self._send_json(result)
                return
            except Exception as e:
                sys.stderr.write(f"[DELETE register] ERROR: {e}\n")
                sys.stderr.flush()
                self._send_json({'error': str(e)})
                return

        self.send_error(400)

    # ---- OPTIONS (CORS preflight) ----
    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, PUT, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


def main():
    parser = argparse.ArgumentParser(description='ESP32-S3 Gateway Web Config Simulator')
    parser.add_argument('--port', type=int, default=8080,
                        help='HTTP server port (default: 8080)')
    args = parser.parse_args()

    # Initialize discovery module with shared state
    discover_sim.init(mappings_list=MAPPINGS, log_callback=add_log)
    # Point to the device simulator for metadata-enriched scanning
    discover_sim.set_simulator_url('http://localhost:8081')

    # Start background poll simulator
    poll_thread = threading.Thread(target=background_poll_simulator, daemon=True)
    poll_thread.start()

    server = ThreadingHTTPServer(('0.0.0.0', args.port), GatewayHandler)
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║  ESP32-S3 Gateway Web Configuration — PC Simulation        ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Server running at:  http://localhost:{args.port}              ║
║                                                              ║
║  Open this URL in your browser to access the configuration   ║
║  interface. The page supports Chinese/English toggle.        ║
║                                                              ║
║  API Endpoints:                                              ║
║    GET  /api/system/status   → System metrics                ║
║    GET  /api/wifi/config     → WiFi settings                 ║
║    PUT  /api/wifi/config     → Update WiFi                   ║
║    GET  /api/mqtt/config     → MQTT settings                 ║
║    PUT  /api/mqtt/config     → Update MQTT                   ║
║    GET  /api/modbus/config   → MODBUS settings               ║
║    PUT  /api/modbus/config   → Update MODBUS                 ║
║    GET  /api/mappings        → List AMM mappings             ║
║    POST /api/mappings        → Add new mapping               ║
║    PUT  /api/mappings/<idx>  → Update mapping                ║
║    DEL  /api/mappings/<idx>  → Delete mapping                ║
║                                                              ║
║    GET  /api/discover/status  → Discovery status              ║
║    GET  /api/discover/devices → Discovered devices            ║
║    POST /api/discover/scan    → Full bus scan                 ║
║    POST /api/discover/apply   → Apply as AMM mappings         ║
║    POST /api/discover/reset   → Reset discovery               ║
║                                                              ║
║  Press Ctrl+C to stop the server.                            ║
╚══════════════════════════════════════════════════════════════╝
""")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[WEB] Server stopped.")
        server.server_close()


if __name__ == '__main__':
    main()
