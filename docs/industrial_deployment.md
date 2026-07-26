# Industrial Deployment Checklist

## RS485

- Use an isolated 3.3 V RS485 transceiver for field deployment. The development MAX3485 connection is suitable for bench validation but is not isolated.
- Add 600 W or higher TVS protection on A/B, a resettable fuse or current limiter, common-mode choke where EMC testing requires it, and a selectable 120 ohm termination at the two physical ends of the bus.
- Use one bias network per segment, shielded twisted pair, shield grounding according to the site EMC plan, and a shared reference or isolated ground strategy.
- Keep the gateway in master mode unless a tested multi-master arbitration scheme is explicitly deployed.

## Power And Ethernet

- Use reverse-polarity, surge, EFT and ESD protection on the field power input. Add a supervisor or brownout margin appropriate to W5500, TF and RS485 peak current.
- Use an Ethernet connector with magnetics and verify shield/chassis bonding. Production hardware should add Ethernet ESD protection close to the connector.
- W5500 is the preferred uplink. WiFi STA is the automatic fallback and the configuration AP remains available for local maintenance.

## Storage

- SPI Flash is the primary UIF queue. TF is overflow and history storage only.
- Validate TF wear, unexpected removal, power-loss recovery and full-media oldest-first deletion with the exact card model used in the deployment.
- Monitor cache usage, data-loss count, minimum heap and watchdog reset count from `/api/system/status`.

## Security And OTA

- Web authentication is disabled by default for the thesis bench network. Enable Bearer authentication before connecting the gateway to an untrusted network.
- Use `mqtts://` and `https://` endpoints with valid public CA chains or add a controlled private CA bundle.
- OTA requires a 64-character SHA-256 value and uses dual OTA partitions with rollback.
- Secure Boot and Flash Encryption are reported but never enabled automatically. Burn eFuses only in a controlled production process after recovery, key custody and manufacturing tests are complete.
- Place the gateway behind a firewall/VLAN. Allow only required MQTT, NTP, HTTPS, MODBUS TCP and management sources.

## Acceptance Tests

- Run 24-hour RTU/TCP polling, broker interruption/replay, Ethernet-to-WiFi failover, power-loss recovery and TF removal tests.
- Record semantic consistency, AMM adaptation latency, UIF recovery time, throughput, CPU/heap/PSRAM, cache growth and watchdog resets.
- Verify every writable AMM point has a bounded range, an operator authorization policy and an automation interlock where the process risk requires it.
