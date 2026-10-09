"""SDK-free checks for passive FTP DDS callbacks and reliable shutdown."""
import json
import os
import signal
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import g1_ftp_readonly_observer as observer


class _FakeSubscriber:
    instances = []

    def __init__(self, topic, message_type):
        self.topic = topic
        self.handler = None
        self.closed = False
        self.instances.append(self)

    def Init(self, handler=None, queueLen=0):
        if handler is None:
            raise AssertionError('Blocking Read() must not be used')
        self.handler = handler
        self.handler(SimpleNamespace(angle_act=[0, 100, 200, 300, 400, 500]))

    def Close(self):
        self.closed = True

    def Read(self, *args, **kwargs):
        raise AssertionError('Blocking Read() must not be called')


def _fake_sdk():
    channel = types.ModuleType('unitree_sdk2py.core.channel')
    channel.ChannelFactoryInitialize = lambda *_a, **_kw: None
    channel.ChannelSubscriber = _FakeSubscriber
    core = types.ModuleType('unitree_sdk2py.core')
    core.channel = channel
    unitree = types.ModuleType('unitree_sdk2py')
    unitree.core = core
    inspire = types.ModuleType('inspire_sdkpy')
    inspire.inspire_dds = SimpleNamespace(inspire_hand_state=object)
    return {
        'unitree_sdk2py': unitree,
        'unitree_sdk2py.core': core,
        'unitree_sdk2py.core.channel': channel,
        'inspire_sdkpy': inspire,
    }


class FtpShutdownTests(unittest.TestCase):
    def run_fake_observer(self, sig):
        _FakeSubscriber.instances.clear()
        handlers = {}
        previous = {}
        with tempfile.TemporaryDirectory() as folder:
            sockpath = os.path.join(folder, 'ftp.sock')

            def register(sig_num, callback):
                previous[sig_num] = handlers.get(sig_num)
                handlers[sig_num] = callback

            def terminate_after_listening():
                for _ in range(100):
                    if len(_FakeSubscriber.instances) == 2 and all(
                        s.handler is not None for s in _FakeSubscriber.instances
                    ) and sockpath and os.path.exists(sockpath):
                        break
                    time.sleep(.01)
                else:
                    raise AssertionError('Observer never started socket')
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                    client.settimeout(1)
                    client.connect(sockpath)
                    client.sendall(b'{"kind":"ftp_status"}\n')
                    result = json.loads(client.recv(8192))
                    self.assertEqual(result['diagnostics']['received_messages'],
                                     {'left': 1, 'right': 1})
                    self.assertTrue(result['both_fresh'])
                handlers[sig](sig, None)

            with mock.patch.dict(sys.modules, _fake_sdk()), mock.patch.object(
                    observer.signal, 'signal', side_effect=register), mock.patch.object(
                    observer.Path, 'exists', return_value=True):
                trigger = threading.Thread(target=terminate_after_listening)
                trigger.start()
                try:
                    observer.main(['--interface', 'eth0', '--socket', sockpath])
                finally:
                    trigger.join(timeout=3)
                    self.assertFalse(trigger.is_alive(), 'Observer blocked shutdown')

            self.assertFalse(os.path.lexists(sockpath), 'Socket leaked after shutdown')
            self.assertEqual(len(_FakeSubscriber.instances), 2)
            self.assertTrue(all(sub.closed for sub in _FakeSubscriber.instances))
            self.assertEqual([sub.topic for sub in _FakeSubscriber.instances], [
                'rt/inspire_hand/state/l', 'rt/inspire_hand/state/r'])

    def test_sigint_closes_callbacks_and_ipc(self):
        self.run_fake_observer(signal.SIGINT)

    def test_sigterm_closes_callbacks_and_ipc(self):
        self.run_fake_observer(signal.SIGTERM)

    def test_does_not_use_polling_read_or_create_publishers(self):
        source = Path(observer.__file__).read_text()
        self.assertNotIn('sub.Read(', source)
        self.assertNotIn('ChannelPublisher', source)
        self.assertNotIn('inspire_hand/ctrl/', source)


if __name__ == '__main__':
    unittest.main()
