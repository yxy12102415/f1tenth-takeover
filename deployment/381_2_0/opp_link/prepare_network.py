#!/usr/bin/env python3
"""Run locally with sudo. Never prints Wi-Fi credentials or link keys."""
import json
import os
from pathlib import Path
import secrets
import subprocess

ROOT = Path(__file__).resolve().parent

def main():
    if os.geteuid() != 0:
        raise SystemExit('Run with sudo to read the saved NetworkManager Wi-Fi key.')
    uid = int(os.environ.get('SUDO_UID', ROOT.stat().st_uid))
    gid = int(os.environ.get('SUDO_GID', ROOT.stat().st_gid))
    def nm(*args):
        return subprocess.check_output(['nmcli', *args], text=True, stderr=subprocess.DEVNULL).strip()
    connection = nm('-g', 'GENERAL.CONNECTION', 'device', 'show', 'wlan0')
    ssid = nm('-g', '802-11-wireless.ssid', 'connection', 'show', connection)
    psk = nm('--show-secrets', '-g', '802-11-wireless-security.psk', 'connection', 'show', connection)
    if not ssid or not psk or psk == '--':
        raise SystemExit('No saved Wi-Fi PSK available. No files written.')
    private = ROOT / 'private'
    private.mkdir(mode=0o700, exist_ok=True)
    os.chown(private, uid, gid)
    os.chmod(private, 0o700)
    existing = private / 'host.json'
    key = json.loads(existing.read_text())['key_hex'] if existing.exists() else secrets.token_hex(32)
    header = ROOT / 'firmware/include/link_private.h'
    header.parent.mkdir(parents=True, exist_ok=True)
    content = ('#pragma once\nstatic const char WIFI_SSID[] = ' + json.dumps(ssid) + ';\n'
               'static const char WIFI_PSK[] = ' + json.dumps(psk) + ';\n'
               'static const unsigned char LINK_KEY[32] = {' +
               ','.join(str(x) for x in bytes.fromhex(key)) + '};\n')
    for path, data in [(header, content), (existing, json.dumps({'key_hex': key, 'port': 42132}) + '\n')]:
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(data)
        os.chmod(path, 0o600)
        os.chown(path, uid, gid)
    print('Private Wi-Fi/build configuration prepared. Credentials were not printed.')

if __name__ == '__main__':
    main()
