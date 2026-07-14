"""Test all device management API endpoints."""
import json
import urllib.request
import sys

BASE = 'http://localhost:8080'
passed = 0
failed = 0

def req(method, path, body=None):
    url = BASE + path
    data = json.dumps(body).encode() if body else None
    r = urllib.request.Request(url, data=data, method=method)
    r.add_header('Content-Type', 'application/json')
    try:
        resp = urllib.request.urlopen(r, timeout=5)
        return json.loads(resp.read().decode())
    except Exception as e:
        return {'_error': str(e)}

def test(name, result, check_fn):
    global passed, failed
    ok = check_fn(result)
    status = 'PASS' if ok else 'FAIL'
    if ok:
        passed += 1
    else:
        failed += 1
    print(f"[{status}] {name}")
    if not ok:
        print(f"       Got: {json.dumps(result, ensure_ascii=False)[:200]}")
    return ok

print("=" * 60)
print("Device Management API Tests")
print("=" * 60)

# 0. Scan first
r = req('POST', '/api/discover/scan', {'slave_start': 1, 'slave_end': 10, 'reg_start': 40001, 'reg_end': 40999})
test("POST /api/discover/scan", r, lambda x: x.get('devices_found', 0) >= 4)
print(f"  Found {r.get('devices_found')} devices, {r.get('registers_found')} registers")

# 1. Get discovered devices
r = req('GET', '/api/discover/devices')
test("GET /api/discover/devices returns list", r, lambda x: isinstance(x, list) and len(x) >= 4)

dev0 = r[0] if r else {}
slave_id = dev0.get('slave_id', 1)
regs = dev0.get('registers', [])
print(f"  First device: slave_id={slave_id}, regs={len(regs)}")

# 2. Update device info
r = req('PUT', f'/api/discover/devices/{slave_id}', {
    'device_id': 'plc_main_v2',
    'name': 'PLC Main Controller (Updated)',
    'description': 'Updated via API test',
    'mqtt_topic_prefix': 'factory/line1/plc_main_v2'
})
test("PUT /api/discover/devices/<id> - update device", r,
     lambda x: x.get('status') == 'ok' and 'error' not in x)

# 3. Verify device update
r = req('GET', '/api/discover/devices')
dev0 = [d for d in r if d['slave_id'] == slave_id][0] if r else {}
test("Device name updated", dev0, lambda x: x.get('name') == 'PLC Main Controller (Updated)')
test("Device MQTT prefix updated", dev0,
     lambda x: x.get('mqtt_topic_prefix') == 'factory/line1/plc_main_v2')

# 4. Update register
if regs:
    reg_addr = regs[0]['register_address']
    r = req('PUT', f'/api/discover/devices/{slave_id}/registers/{reg_addr}', {
        'name': 'Temperature Sensor A',
        'unit': 'Celsius',
        'data_type': 'FLOAT32',
        'writable': False,
        'range_min': -40.0,
        'range_max': 120.0
    })
    test("PUT register - update register", r,
         lambda x: x.get('status') == 'ok' and 'error' not in x)

    # 5. Verify register update
    r = req('GET', '/api/discover/devices')
    dev0 = [d for d in r if d['slave_id'] == slave_id][0] if r else {}
    reg0 = [rg for rg in dev0.get('registers', []) if rg['register_address'] == reg_addr]
    if reg0:
        test("Register name updated",
             reg0[0], lambda x: x.get('inferred_name') == 'Temperature Sensor A')
        test("Register unit updated",
             reg0[0], lambda x: x.get('inferred_unit') == 'Celsius')
        test("Register range updated",
             reg0[0], lambda x: x.get('range_max') == 120.0)

# 6. Toggle register
if regs:
    r = req('POST', f'/api/discover/devices/{slave_id}/registers/{reg_addr}/toggle')
    test("POST toggle register - disable", r,
         lambda x: x.get('status') == 'ok' and x.get('valid') == False)

    r = req('POST', f'/api/discover/devices/{slave_id}/registers/{reg_addr}/toggle')
    test("POST toggle register - re-enable", r,
         lambda x: x.get('status') == 'ok' and x.get('valid') == True)

# 7. Export devices
r = req('POST', '/api/discover/export', {})
test("POST /api/discover/export - returns array", r,
     lambda x: isinstance(x, list) and len(x) >= 4)
export_data = r if isinstance(r, list) else []

# 8. Delete register
all_regs = regs[:]
if len(all_regs) > 1:
    del_addr = all_regs[-1]['register_address']
    r = req('DELETE', f'/api/discover/devices/{slave_id}/registers/{del_addr}')
    test("DELETE register", r,
         lambda x: x.get('status') == 'ok' and 'error' not in x)

    # Verify deletion
    r = req('GET', '/api/discover/devices')
    dev0 = [d for d in r if d['slave_id'] == slave_id][0] if r else {}
    remaining = [rg for rg in dev0.get('registers', []) if rg['register_address'] == del_addr]
    test("Register actually deleted", remaining, lambda x: len(x) == 0)

# 9. Import devices
if export_data:
    export_data[0]['name'] = 'Imported Device Test'
    r = req('POST', '/api/discover/import', export_data)
    test("POST /api/discover/import", r,
         lambda x: isinstance(x, dict) and 'error' not in x)

    r = req('GET', '/api/discover/devices')
    test("Import replaced devices", r,
         lambda x: isinstance(x, list) and len(x) >= 4)
    imported_dev = r[0] if r else {}
    test("Imported device name correct", imported_dev,
         lambda x: x.get('name') == 'Imported Device Test')

# 10. Apply mappings
r = req('POST', '/api/discover/apply', {})
test("POST /api/discover/apply", r,
     lambda x: 'mappings_created' in x or 'total_mappings' in x)

# 11. Reset
r = req('POST', '/api/discover/reset', {})
test("POST /api/discover/reset", r,
     lambda x: 'error' not in x)

r = req('GET', '/api/discover/devices')
test("Reset cleared devices", r,
     lambda x: isinstance(x, list) and len(x) == 0)

# Summary
print("=" * 60)
print(f"Results: {passed} passed, {failed} failed, {passed + failed} total")
print("=" * 60)
sys.exit(0 if failed == 0 else 1)
