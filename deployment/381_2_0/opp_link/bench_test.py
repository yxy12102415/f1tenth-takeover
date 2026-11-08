#!/usr/bin/env python3
"""ESC POWER OFF only. Reboots ESP32; verifies inhibited firmware over real Wi-Fi."""
import argparse
import json
from pathlib import Path
import re
import select
import socket
import time
import serial
from protocol import Link, ROOT, encode

def reset_board(port):
    p = serial.Serial(port=None, baudrate=115200, timeout=.2)
    p.dtr = False; p.rts = False; p.port = port; p.open()
    p.rts = True; time.sleep(.1); p.rts = False
    began = time.monotonic(); address = None; banner = False
    try:
        while time.monotonic()-began < 25:
            line = p.readline().decode('utf8', errors='replace')
            if 'OPP_LINK_BENCH_V1 MOTOR_INHIBITED' in line: banner = True
            match = re.search(r'OPP_BENCH ip=([0-9.]+).*inhibited=1 pwm_ok=1', line)
            if match and match[1] != '0.0.0.0': address = match[1]
            if banner and address and time.monotonic()-began > 5.2:
                return address
    finally:
        p.close()
    raise RuntimeError('No healthy inhibited Wi-Fi firmware after reboot')

def drain(link, observations):
    for _ in range(100):
        if not select.select([link.sock], [], [], 0)[0]: break
        try:
            f = link.receive()
            observations.append(dict(at=time.monotonic(), **f))
        except ValueError:
            continue

def send_for(link, seconds, enable, observations):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        link.command(enable, 1100 if enable else 1000)
        time.sleep(.02); drain(link, observations)
    if link.last_ack_at is None or time.monotonic()-link.last_ack_at > .1:
        raise RuntimeError('No fresh acknowledgements')

def scenario(port, fault):
    ip = reset_board(port)
    link = Link(ip); observations = []; report = {'fault':fault, 'ip':ip}
    try:
        link.discover(); send_for(link,.25,False,observations)
        assert link.latest['state']==2, link.latest
        send_for(link,.3,True,observations)
        assert link.latest['state']==3, link.latest
        began = time.monotonic()
        if fault=='release':
            send_for(link,.15,False,observations)
        elif fault in ('timeout','replay','bad_mac','stale_command'):
            packet=link.command(True,1100)
            if fault=='bad_mac': packet=packet[:-1]+bytes([packet[-1]^1])
            until=time.monotonic()+.40
            while time.monotonic()<until:
                if fault in ('replay','bad_mac'): link.sock.sendto(packet,link.peer)
                if fault=='stale_command':
                    link.sequence+=1
                    packet=encode(link.key,2,link.boot,link.session,link.sequence,True,1100,1)
                    link.sock.sendto(packet,link.peer)
                time.sleep(.02); drain(link,observations)
        else:
            raise ValueError(fault)
        drain(link,observations)
        assert link.latest['state']==4, link.latest
        assert observations and all(f['motor']==1000 and f['flags']==128 for f in observations)
        latches=[f['at'] for f in observations if f['at']>=began and f['state']==4]
        assert latches
        report.update(passed=True, latch_observed_after_s=min(latches)-began,
                      observations=len(observations), applied_motor_us=1000,
                      actual_pwm_measured=False, physical_motion_tested=False)
        return report
    finally:
        link.close()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',default='/dev/ttyUSB0')
    args=parser.parse_args()
    reports=[]
    try:
        for fault in ('release','timeout','replay','bad_mac','stale_command'):
            result=scenario(args.port,fault); reports.append(result)
            print(json.dumps(result),flush=True)
    finally:
        out=ROOT/'reports';out.mkdir(exist_ok=True)
        (out/'bench_wifi.json').write_text(json.dumps({'passed':len(reports)==5,
            'scope':'Inhibited receiver, actual ESP32/Wi-Fi; no vehicle motion',
            'tests':reports},indent=2)+'\n')

if __name__=='__main__': main()
