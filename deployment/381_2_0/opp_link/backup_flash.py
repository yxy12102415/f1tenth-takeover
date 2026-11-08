#!/usr/bin/env python3
"""Read-only segmented ESP32 backup; ESC must be powered off (reset required)."""
import hashlib
import json
import os
from pathlib import Path
import time
import esptool

ROOT=Path(__file__).resolve().parent
PORT='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
SIZE=0x400000
CHUNK=0x10000

def connect(baud=460800):
    e=esptool.detect_chip(PORT,115200)
    if e.CHIP_NAME!='ESP32':raise RuntimeError('Unexpected device')
    e=e.run_stub();e.flash_set_parameters(SIZE)
    if baud!=115200:e.change_baud(baud)
    e._port.timeout=3
    e._port.write_timeout=3
    return e

def main():
    os.umask(0o077)
    output=ROOT/'private/esp32_before_opp_link_20261107.bin'
    partial=output.with_suffix('.partial')
    e=connect()
    try:
        with partial.open('wb') as f:
            for offset in range(0,SIZE,CHUNK):
                for attempt in range(3):
                    try:
                        data=e.read_flash(offset,CHUNK)
                        if len(data)!=CHUNK:raise RuntimeError('Short chunk')
                        break
                    except Exception as exc:
                        print('Read retry at 0x%x: %s'%(offset,type(exc).__name__),flush=True)
                        e._port.close()
                        if attempt==2:raise
                        time.sleep(.2);e=connect(115200)
                f.write(data);f.flush()
                print('BACKUP %.1f%%'%((offset+CHUNK)*100/SIZE),flush=True)
        data=partial.read_bytes();device_md5=e.flash_md5sum(0,SIZE)
        if len(data)!=SIZE or hashlib.md5(data).hexdigest()!=device_md5:
            raise RuntimeError('Full-flash checksum mismatch; backup not accepted')
        partial.replace(output)
        p=ROOT/'STATUS.json';s=json.loads(p.read_text())
        s.update(original_backup_verified=True,original_backup_sha256=hashlib.sha256(data).hexdigest(),
                 original_backup_bytes=len(data))
        p.write_text(json.dumps(s,indent=2)+'\n')
        print('BACKUP VERIFIED: 4 MiB; full-flash digest matches device',flush=True)
        e.hard_reset()
    finally:
        e._port.close()

if __name__=='__main__':main()
