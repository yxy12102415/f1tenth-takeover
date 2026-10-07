#!/usr/bin/env bash
set -euo pipefail

controller_mac="88:03:4C:1E:3A:B4"
joy_probe="/opt/ros/foxy/lib/joy/joy_enumerate_devices"

controller_ready() {
  [[ -e /dev/input/js0 ]] &&
    timeout 3 "$joy_probe" 2>/dev/null |
      grep -Eq '^[[:space:]]*0[[:space:]]*:[[:space:]]*Wireless Controller[[:space:]]*$'
}

if controller_ready; then
  exit 0
fi

printf 'PS4 controller is asleep or disconnected. Press the PS button now; retrying for 18 seconds...\n'
bluetoothctl power on >/dev/null 2>&1 || true
bluetoothctl trust "$controller_mac" >/dev/null 2>&1 || true

for _ in 1 2 3; do
  timeout 5 bluetoothctl connect "$controller_mac" >/dev/null 2>&1 || true
  for _ in 1 2; do
    if controller_ready; then
      printf 'PS4 controller connected and readable as /dev/input/js0.\n'
      exit 0
    fi
    sleep 1
  done
done

printf 'REFUSED: PS4 controller did not wake. Press PS once, then rerun the command.\n' >&2
printf 'If the light keeps flashing, hold SHARE+PS to pair it again.\n' >&2
exit 1
