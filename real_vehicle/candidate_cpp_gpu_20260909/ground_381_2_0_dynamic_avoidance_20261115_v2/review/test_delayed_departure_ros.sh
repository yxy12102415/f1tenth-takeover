#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
for tracker in pid dynamic_mpc; do
  bash ../offline_env.sh env ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 python3 test_generic_ros.py --tracker "$tracker" --lead-width .30 --lead-speed 0 --startup-command-deadzone .70 --lead-start-distance 2.85 --lead-lateral-center -.08 --ego-start-lateral -.12 --ego-start-heading-offset -.05 --report-name "compact_fix/${tracker}_delayed_static"
done
