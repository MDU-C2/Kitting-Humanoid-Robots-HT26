import os
import tempfile
import threading
import unittest

import numpy as np

from xr_ipc_protocol import IpcError, ping, send_sample
from xr_offline_receiver import LocalServer, RequestHandler


class FakeXr:
    def handle(self, message):
        if message.get('kind') == 'ping':
            return {'kind': 'pong', 'hardware_connected': False}
        return {'kind': 'sample_ack', 'source': message['source'],
                'seq': message['seq'], 'tau_ff': [0.1] * 14,
                'hardware_connected': False}


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tempdir.name, 'bridge.sock')
        self.server = LocalServer(self.path, RequestHandler)
        self.server.receiver = FakeXr()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.tempdir.cleanup()

    def test_ping(self):
        self.assertFalse(ping(self.path)['hardware_connected'])

    def test_q_to_xr_and_ack(self):
        ans = send_sample([0.0] * 14, 'trajectory', 42, self.path)
        self.assertEqual(ans['seq'], 42)
        self.assertEqual(ans['tau_ff'], [0.1] * 14)

    def test_numpy_trajectory_samples(self):
        # The trajectory adapter returns ndarray with numpy scalar elements.
        for dtype in (np.float64, np.float32, np.int64):
            with self.subTest(dtype=dtype):
                q = np.zeros(14, dtype=dtype)
                q[3] = dtype(0.05) if dtype != np.int64 else dtype(1)
                ack = send_sample(q, 'trajectory', 44, self.path)
                self.assertEqual(ack['seq'], 44)
                self.assertEqual(ack['tau_ff'], [0.1] * 14)

    def test_non_real_and_bool_rejected(self):
        for value in (True, '0.0', 0.5j, float('inf')):
            with self.subTest(value=value), self.assertRaises(IpcError):
                send_sample([value] * 14, 'trajectory', 45, self.path)

    def test_hold(self):
        self.assertEqual(send_sample([0.2] * 14, 'hold', 43, self.path)['source'], 'hold')

    def test_invalid_shapes_fail(self):
        with self.assertRaises(IpcError):
            send_sample([0] * 13, 'trajectory', 1, self.path)
        with self.assertRaises(IpcError):
            send_sample([float('nan')] * 14, 'trajectory', 1, self.path)
        with self.assertRaises(IpcError):
            send_sample([0] * 14, 'vr', 1, self.path)

    def test_missing_receiver_fails_closed(self):
        with self.assertRaises(IpcError):
            ping(self.path + '.missing')


if __name__ == '__main__':
    unittest.main()
