"""Authenticated bench-only UDP protocol. No ROS or actuator publishers."""
import hashlib
import hmac
import json
from pathlib import Path
import secrets
import socket
import struct
import time

WIRE = struct.Struct('<IBBBBQQIHHI')
MAGIC = 0x3150504f
ROOT = Path(__file__).resolve().parent

def encode(key, kind, boot, session, sequence=0, enable=False, motor=1000, tick=0):
    body = WIRE.pack(MAGIC, 1, kind, int(enable), 0, boot, session, sequence, motor, 1600, tick)
    return body + hmac.new(key, body, hashlib.sha256).digest()

def decode(key, data):
    if len(data) != WIRE.size + 32 or not hmac.compare_digest(
            data[-32:], hmac.new(key, data[:-32], hashlib.sha256).digest()):
        raise ValueError('Invalid packet or authentication')
    values = WIRE.unpack(data[:-32])
    result = dict(zip(('magic', 'version', 'kind', 'flags', 'state', 'boot', 'session',
                       'sequence', 'motor', 'steering', 'tick'), values))
    if result['magic'] != MAGIC or result['version'] != 1:
        raise ValueError('Invalid protocol')
    return result

class Link:
    def __init__(self, address):
        cfg = json.loads((ROOT/'private/host.json').read_text())
        self.key = bytes.fromhex(cfg['key_hex'])
        self.peer = (socket.gethostbyname(address), cfg['port'])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(.1)
        self.session = secrets.randbits(64) or 1
        self.boot = 0
        self.sequence = 0
        self.latest = None
        self.clock_at = None
        self.clock_tick = 0
        self.sent = {}
        self.last_ack_at = None
        self.ack_tick = None
        self.ack_sequence = 0

    def discover(self):
        # A new challenge per probe prevents delayed replies to a timed-out
        # probe from being mistaken for a low-latency clock sample.
        for _ in range(40):
            time.sleep(.02)
            self.session = secrets.randbits(64) or 1
            self.sock.sendto(encode(self.key, 1, 0, self.session), self.peer)
            sent_at = time.monotonic()
            try:
                while True:
                    data, peer = self.sock.recvfrom(256)
                    f = decode(self.key, data)
                    if time.monotonic()-sent_at > .1: break
                    if peer == self.peer and f['session'] == self.session and f['kind'] == 3: break
                received_at=time.monotonic()
                if peer != self.peer or f['session'] != self.session or f['kind'] != 3: continue
                if received_at-sent_at > .04: continue
                if f['flags'] != 0x80 or f['motor'] != 1000 or f['steering'] != 1600:
                    raise RuntimeError('Receiver is not the inhibited bench firmware')
                if f['state'] != 0: raise RuntimeError('Receiver already used; reset ESP32 for a new bench session')
                self.boot = f['boot']; self.latest = f
                self.clock_at = (sent_at+received_at)/2; self.clock_tick = f['tick']
                return f
            except (socket.timeout, ValueError):
                continue
        raise RuntimeError('ESP32 discovery timed out')

    def command(self, enable=False, motor=1000):
        if not self.boot: raise RuntimeError('Discover first')
        now = time.monotonic()
        if now-self.clock_at > .25:
            raise RuntimeError('Receiver clock/acknowledgement stale')
        self.sequence += 1
        tick = (self.clock_tick + int((now-self.clock_at)*1000)) & 0xffffffff
        packet = encode(self.key, 2, self.boot, self.session, self.sequence, enable, motor, tick)
        self.sent = {s:t for s,t in self.sent.items() if now-t <= .25}
        self.sent[self.sequence] = now
        self.sock.sendto(packet, self.peer)
        return packet

    def receive(self):
        data, peer = self.sock.recvfrom(256)
        f = decode(self.key, data)
        if peer != self.peer or f['boot'] != self.boot or f['session'] != self.session or f['kind'] != 3:
            raise ValueError('Foreign status')
        if f['flags'] != 0x80 or f['motor'] != 1000 or f['steering'] != 1600:
            raise RuntimeError('Bench output invariant violated')
        if self.latest is not None:
            delta = (f['tick']-self.latest['tick']) & 0xffffffff
            if delta >= 0x80000000 or f['sequence'] < self.latest['sequence']:
                raise ValueError('Reordered or replayed status')
        now = time.monotonic()
        # Only a fresh echo of a new command renews host link health.
        sent = self.sent.get(f['sequence'])
        tick_new = self.ack_tick is None or 0 < ((f['tick']-self.ack_tick) & 0xffffffff) < 0x80000000
        if sent is not None and now-sent <= .1 and f['sequence'] > self.ack_sequence and tick_new:
            self.ack_sequence = f['sequence']; self.ack_tick = f['tick']
            self.last_ack_at = now
            self.clock_at = now; self.clock_tick = f['tick']
        self.latest = f
        return f

    def close(self):
        # Best effort explicit stop; independent receiver expiry covers lost packets.
        try:
            for _ in range(3): self.command(False, 1000)
        except (OSError, RuntimeError):
            pass
        self.sock.close()
