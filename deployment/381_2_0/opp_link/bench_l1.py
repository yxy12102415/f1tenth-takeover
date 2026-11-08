#!/usr/bin/env python3
"""One L1 press/release on inhibited ESP32. Does not start or command F1.

Linux joystick events are used only for this bench demonstration, not as the
production heartbeat/deadman source. The receiver limits a bench request to 5 s.
"""
import argparse
import json
import os
import select
import struct
import time
from bench_test import reset_board, drain
from protocol import Link, ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--joystick',default='/dev/input/js0');args=p.parse_args()
    fd=os.open(args.joystick,os.O_RDONLY|os.O_NONBLOCK)
    link=None;trace=[];transitions=[];held=False;release_seen=False;pressed=False;passed=False
    try:
        link=Link(reset_board(args.port));link.discover()
        print('BENCH ONLY: motor fixed at 1000 us. Release L1, then hold for 1-2 s and release.',flush=True)
        started=time.monotonic();until=started+120;previous=None
        while time.monotonic()<until:
            while select.select([fd],[],[],0)[0]:
                data=os.read(fd,8)
                if len(data)!=8:raise RuntimeError('Joystick disconnected')
                _,value,kind,number=struct.unpack('<IhBB',data)
                if kind & 1 and number==4:
                    held=bool(value)
                    if not held:release_seen=True
            enable=held and release_seen
            link.command(enable,1100 if enable else 1000)
            time.sleep(.02);drain(link,trace)
            if time.monotonic()-(link.last_ack_at if link.last_ack_at is not None else started)>.25:
                raise RuntimeError('Acknowledgement timeout')
            state=link.latest['state']
            if state!=previous:
                item=dict(at=time.monotonic(),l1_held=held,receiver_state=state,applied_motor_us=link.latest['motor'])
                transitions.append(item);print(json.dumps(item),flush=True);previous=state
            if state==3:pressed=True
            if state==4:
                if pressed and not held:
                    passed=True
                    print('PASS: physical L1 press/release reached ESP32; motor output stayed inhibited.',flush=True)
                    return
                raise RuntimeError('Receiver stopped before a complete L1 press/release')
        raise RuntimeError('No complete L1 press/release within 120 seconds')
    finally:
        if link:link.close()
        os.close(fd)
        (ROOT/'reports').mkdir(exist_ok=True)
        (ROOT/'reports/l1_bench.json').write_text(json.dumps({'passed':passed,'transitions':transitions,
            'physical_motor_motion_tested':False},indent=2)+'\n')

if __name__=='__main__':main()
