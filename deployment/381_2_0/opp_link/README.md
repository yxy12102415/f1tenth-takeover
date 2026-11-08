# Historical Wi-Fi link experiment — not the active control mode

Current trials use **two independent PS4 controllers**; see
[the active lead firmware](../../../oppVehicle/ps4_control/README.md) and
[the current vehicle checkpoint](../../../docs/VEHICLE_PROGRESS_20261108.md).
Wi-Fi bench tests showed stale-command/acknowledgement failures and UDP errors;
common-L1 control was not commissioned. The files and original notes below
are retained for reference, not instructions to replace the current firmware.

# F1 → ESP32 link: inhibited bench release v1

This release tests the single-controller communication path. It does **not**
commission two moving vehicles or enable physical overtaking.

**Current device state:** restored the original full-flash PS4-direct firmware
and verified the write, following selection of the two-controller fallback.
The inhibited Wi-Fi firmware below is retained as a development candidate and
is **not currently installed**. F1 and the lead vehicle have independent PS4
controls; releasing F1's L1 does not stop the lead vehicle. Restored manual
control has not yet been physically re-tested. No vehicle drivers were started.

Latest hardware result (2026-11-07): original 4 MiB flash backup verified against
the device digest; inhibited firmware uploaded and verified. ESP32 obtains
192.168.1.199 and authenticated discovery sometimes succeeds, but continuous
acknowledgement/fault testing has **not passed**. One five-packet ICMP sample had
105.7–403.6 ms round-trip delay (mean 254.5 ms), despite RSSI around -46 dBm and
power saving disabled at both ends. This does not identify the cause. The actual
L1 demonstration and reciprocal F1 stop integration are still pending. Keep
ESC power off. See `STATUS.json`; do not treat compilation as commissioning.

PS4 stays paired with F1. ESP32 joins F1's existing Wi-Fi as a station. F1 sends
authenticated UDP requests at 50 Hz; ESP32 returns authenticated status. ESP32's
GPIO18 is fixed at the nominal 1000 µs ESC stop signal; GPIO19 is fixed at nominal
1600 µs steering center. Requested throttle is never applied. These are configured
PWM values, not oscilloscope measurements; 1000 µs means coast, not active braking.

## Current components

- `firmware/`: ESP32 Arduino 2.0.14, PlatformIO espressif32 6.6.0. It replaces
  the PS4-direct firmware during bench testing; Bluetooth control is not running.
- `protocol.py`: UDP/HMAC-SHA256, random boot/session identifiers, increasing
  sequence, receiver-time age check, fresh acknowledgement tracking.
- `bench_test.py`: real board/network tests with ESC power off: release, silence,
  repeated packet, invalid authentication, stale new-sequence commands.
- `bench_l1.py`: actual Linux joystick L1 press/release demonstration, using the
  physically inhibited receiver. It never starts the F1 driver. This joystick
  reader is a bench tool, not the production input freshness monitor.
- `shadow_bridge.py`: reads `/baseline/joy`, `/stand_pf/permit`,
  `/stand_pf/health_permit`, `/stand_pf/window_status`, and
  `/stand_pf/actuator_status`. Requires fresh downstream F1 activity and fresh OPP
  acknowledgement to request simulated drive. It has no ROS publishers and
  accepts only inhibited receiver firmware.
- `shadow_gate.py`: candidate gating logic, tested separately. It does not
  integrate the reverse OPP→F1 stop path into the real supervisor.

Receiver requires a release handshake after its 5 s boot interval. A 250 ms command
gap, release after starting, or invalid authenticated command range latches stop.
Old packets cannot renew the command lease. Each bench run is limited to 5 s.
An ESP32 reset creates a new boot identifier and is required for another run;
reconnecting or pressing L1 again does not silently resume a stopped run.

## Local setup and commands

ESC power must be off for USB backup/flash/testing. Disconnect BEC→VIN before
USB powers the ESP32. `prepare_network.py` uses the locally saved Wi-Fi key without
printing it or changing F1 network settings. Run it with sudo on the vehicle.
Its generated header, keys, backups, logs and firmware build output are ignored
by Git; binaries and the full flash backup may contain private credentials.

```sh
cd /home/f1tenth/f1tenth-takeover-381-overtaking/deployment/381_2_0/opp_link
sudo python3 prepare_network.py
~/.local/share/opp-link-tools/bin/pio run -d firmware
python3 -m unittest -v test_host
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_core.cpp -o /tmp/opp_link_core_test
/tmp/opp_link_core_test
```

After the original flash has been backed up and checked, upload the inhibited
firmware to **ESP32 `/dev/ttyUSB0`**, never VESC `/dev/ttyACM0`:

```sh
~/.local/share/opp-link-tools/bin/pio run -d firmware -t upload --upload-port /dev/ttyUSB0
python3 bench_test.py
python3 bench_l1.py
```

Backup path: `private/esp32_before_opp_link_20261107.bin` (4 MiB). Restoration,
if required with ESC power off, uses esptool `write_flash 0x0` with that exact
backup and an explicit ESP32 port. Verify the backup hash in `STATUS.json` first.
Do not erase the flash as a troubleshooting shortcut.

## Required before actual two-car motion

1. Finish the reciprocal OPP-ready/acknowledgement stop interlock in a new F1
   supervisor release. The successful v6 vehicle release is unchanged.
2. A separate reviewed actuator-enabled receiver release, with nonzero PWM
   tested with wheels raised and the operator present. Do not enable actuation
   by removing the bench assertion and assuming the system is commissioned.
3. Measure the lead vehicle's sustainable speed, dimensions and coast distance;
   fixed PWM is not a known m/s command.
4. Integrate predictive overtaking and open-route terminal stopping with the
   physical stack; check a complete pass on the 5.6 m straight. The existing
   dynamic-MPC baseline success is not a dynamic-overtaking validation.

Configured transport/boot timeouts are software budgets, not a guarantee that
both vehicles come to physical rest at the same instant.
