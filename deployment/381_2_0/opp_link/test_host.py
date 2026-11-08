import unittest
import hashlib
import hmac
import socket
from protocol import encode, decode, Link, WIRE, MAGIC
from shadow_gate import Gate

class HostTests(unittest.TestCase):
    def test_discovery_rejects_delayed_previous_challenge(self):
        key=bytes(range(32));peer=('127.0.0.1',42132)
        class Socket:
            def __init__(self):self.old=None;self.queue=[]
            def sendto(self,data,address):
                session=decode(key,data)['session']
                def reply(s,tick):
                    body=WIRE.pack(MAGIC,1,3,128,0,123,s,0,1000,1600,tick)
                    return body+hmac.new(key,body,hashlib.sha256).digest(),peer
                if self.old is None:
                    self.old=session;self.queue=[None]
                else:self.queue=[reply(self.old,5000),reply(session,7000)]
            def recvfrom(self,size):
                item=self.queue.pop(0)
                if item is None:raise socket.timeout()
                return item
        link=Link.__new__(Link);link.key=key;link.peer=peer;link.sock=Socket()
        result=link.discover()
        self.assertEqual(result['tick'],7000)
        self.assertNotEqual(link.session,link.sock.old)

    def test_authentication_and_shape(self):
        key=bytes(range(32)); packet=encode(key,2,123,456,1,True,1100,6000)
        self.assertEqual(len(packet),68)
        self.assertEqual(decode(key,packet)['motor'],1100)
        for index in range(len(packet)):
            altered=packet[:index]+bytes([packet[index]^1])+packet[index+1:]
            with self.assertRaises(ValueError): decode(key,altered)
        with self.assertRaises(ValueError): decode(key,packet[:-1])

    def ready(self):
        g=Gate();g.update('joy',False,9.9)
        for k,v in {'joy':True,'permit':True,'health':True,
                    'window':dict(published_mono=10.,latched=False,started=9.99,reason='',health_ready=True),
                    'actuator':dict(input_age_sec=.01,rpm=100.,stop_reason=None,motor_mode='speed')}.items():
            g.update(k,v,10.)
        return g

    def test_requires_actual_downstream_output(self):
        g=self.ready();self.assertTrue(g.evaluate(10.,True)[0])
        g.update('actuator',dict(input_age_sec=.01,rpm=0.,stop_reason=None,motor_mode='current'),10.02)
        self.assertEqual(g.evaluate(10.02,True),(False,'actuator_stopped'))

    def test_ack_loss_latches(self):
        g=self.ready();self.assertTrue(g.evaluate(10.,True)[0])
        self.assertFalse(g.evaluate(10.02,False)[0]);self.assertFalse(g.evaluate(10.03,True)[0])

    def test_missing_and_stale_sources(self):
        for key in ('joy','permit','health','window','actuator'):
            g=self.ready();del g.values[key];self.assertFalse(g.evaluate(10.,True)[0])
        g=self.ready();self.assertFalse(g.evaluate(10.251,True)[0])

    def test_release_and_terminal_window(self):
        g=self.ready();g.evaluate(10.,True);g.update('joy',False,10.01)
        g.update('joy',True,10.02);self.assertFalse(g.evaluate(10.02,True)[0])
        g=self.ready();g.values['window'][0]['latched']=True
        self.assertFalse(g.evaluate(10.,True)[0])

    def test_stale_source_and_invalid_numbers(self):
        g=self.ready();g.values['window'][0]['published_mono']=9.
        self.assertFalse(g.evaluate(10.,True)[0])
        g=self.ready();g.values['actuator'][0]['rpm']=float('nan')
        self.assertFalse(g.evaluate(10.,True)[0])

if __name__=='__main__':unittest.main()
