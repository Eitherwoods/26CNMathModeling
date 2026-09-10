"""Only in-memory doubles and an owned ephemeral loopback HTTP server are used."""
from http.server import BaseHTTPRequestHandler, HTTPServer
import tempfile
from pathlib import Path
import threading
import unittest

from .offline_stub import OfflineStub, Source
from .protocol import (RobotClient, HttpTransport, ProtocolError, ActionRejected,
                       UncertainAction, encode, decode)


class ProtocolTests(unittest.TestCase):
    def client(self, sources=(), **kw):
        stub = OfflineStub(sources=sources)
        return RobotClient('offline-team', stub, backoff=0, **kw), stub

    def test_attachment_timing_and_log(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'trace.jsonl'
            c, _ = self.client(log_path=path)
            result = [c.enter(), c.measure(300, 400, 1), c.measure(300, 400, 2),
                      c.clear(300, 0, 3), c.measure(300, 0, 2), c.exit()]
            self.assertEqual([r['virtual_time_s'] for r in result], [0, 105, 111, 194, 199, 199])
            self.assertEqual(c.state.channel, 2)
            self.assertEqual(c.state.phase, 'exited')
            rows = [decode(s.encode()) for s in path.read_text('utf-8').splitlines()]
            self.assertEqual(len(rows), 12)
            ids = [decode(r['body_utf8'].encode())['request_id'] for r in rows if r['event'] == 'request']
            self.assertEqual(len(set(ids)), 6)

    def test_lost_response_exact_replay(self):
        c, stub = self.client()
        c.enter()
        calls = []
        def drop(path, body, timeout):
            calls.append((path, body))
            response = stub(path, body, timeout)
            if len(calls) == 1:
                raise TimeoutError('accepted, then response lost')
            return response
        c.transport = drop
        self.assertEqual(c.measure(300, 400, 1)['virtual_time_s'], 105)
        self.assertEqual(calls[0], calls[1])
        self.assertEqual(stub.executed, 2)

    def test_exhausted_retries_block_new_actions(self):
        c, stub = self.client(attempts=1)
        c.enter()
        def drop(path, body, timeout):
            stub(path, body, timeout)
            raise ConnectionResetError('lost')
        c.transport = drop
        with self.assertRaises(UncertainAction): c.measure(300, 400, 1)
        with self.assertRaises(UncertainAction): c.clear(0, 0, 1)
        c.transport = stub
        self.assertEqual(c.retry_pending()['virtual_time_s'], 105)
        self.assertEqual(stub.executed, 2)

    def test_business_rejection_preserves_state(self):
        c, stub = self.client()
        c.enter(); c.measure(300, 400, 1)
        c.transport = lambda *args: stub.reply()
        with self.assertRaises(ActionRejected): c.measure(0, 0, 2)
        self.assertEqual(c.state.virtual_time_s, 105)
        self.assertEqual(c.state.position, (300, 400))
        self.assertEqual(c.state.channel, 1)

    def test_invalid_requests_and_id_conflict(self):
        stub = OfflineStub()
        p = dict(arena_id='default', robot_id='offline-team', request_id='a')
        self.assertEqual(stub('/enter', encode({**p, 'typo': 1}))[0], 200)
        self.assertFalse(decode(stub('/enter', encode({**p, 'typo': 1}))[1])['accepted'])
        self.assertTrue(decode(stub('/enter', encode(p))[1])['accepted'])
        self.assertEqual(stub('/exit', encode(p))[0], 409)
        self.assertEqual(stub('/enter/', encode(p))[0], 404)
        self.assertEqual(stub('/enter', b'{"a":1,"a":2}')[0], 400)
        for value in (True, 1.5, 21, 0):
            q = {**p, 'request_id': 'b', 'position': {'x': 0, 'y': 0}, 'channel': value}
            self.assertEqual(stub('/measure', encode(q))[0], 400)

        for value in (-1, 1200.1, float('inf')):
            with self.assertRaises(ValueError): OfflineStub(remaining=value)

    def test_measure_and_clear_results(self):
        c, _ = self.client([Source(1, 100, 0), Source(2, 0, 0, direction_deg=0)])
        c.enter()
        self.assertEqual(c.measure(0, 0, 1)['svd_deg'], 0)
        self.assertEqual(c.measure(95, 0, 1)['measure_result'], 'near')
        self.assertEqual(c.clear(80, 0, 1)['clear_result'], 'success')
        self.assertEqual(c.clear(80, 0, 1)['clear_result'], 'no_target_in_range')
        self.assertEqual(c.measure(-10, 0, 2)['measure_result'], 'no_signal')
        self.assertEqual(c.clear(-10, 0, 2)['clear_result'], 'success')
        self.assertEqual(c.state.cleared_channels, {1, 2})

    def test_deadline_and_local_validation(self):
        now = [10.0]
        stub = OfflineStub(remaining=7)
        c = RobotClient('offline-team', stub, clock=lambda: now[0])
        c.enter()
        self.assertEqual(c.remaining_real_duration_s, 7)
        for x in (float('nan'), float('inf'), True, 2000001):
            with self.assertRaises(ValueError): c.measure(x, 0, 1)
        now[0] = 17
        with self.assertRaises(ProtocolError): c.measure(0, 0, 1)
        self.assertEqual(stub.executed, 1)

    def test_malformed_response_retains_pending(self):
        c, stub = self.client()
        c.transport = lambda *args: (200, b'{"accepted":true}')
        with self.assertRaises(UncertainAction): c.enter()
        with self.assertRaises(UncertainAction): c.enter()
        c.transport = stub
        self.assertTrue(c.retry_pending()['accepted'])

    def test_http_errors_are_not_network_retries(self):
        for status in (400, 404, 405, 409, 413, 415, 429, 500):
            with self.subTest(status=status):
                c, stub = self.client()
                calls = []
                def fail(*args):
                    calls.append(args)
                    return stub.reply(status)
                c.transport = fail
                with self.assertRaises(ActionRejected) as raised: c.enter()
                self.assertEqual(raised.exception.status, status)
                self.assertEqual(len(calls), 1)

    def test_real_http_to_owned_stub_only(self):
        stub = OfflineStub()
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                self.server.headers_seen = self.headers
                status, raw = stub(self.path, self.rfile.read(int(self.headers['Content-Length'])))
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers(); self.wfile.write(raw)
            def log_message(self, *args): pass
        with HTTPServer(('127.0.0.1', 0), Handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                url = f'http://127.0.0.1:{server.server_port}'
                with self.assertRaises(ValueError): HttpTransport(url)
                transport = HttpTransport(url, allow_network=True)
                status, raw = transport('/enter', b'{"broken":', 2)
                self.assertEqual(status, 400)
                self.assertFalse(decode(raw)['accepted'])
                c = RobotClient('offline-team', transport)
                c.enter(); self.assertEqual(c.measure(300, 400, 1)['virtual_time_s'], 105); c.exit()
                self.assertEqual(server.headers_seen['Content-Type'], 'application/json')
            finally:
                server.shutdown(); thread.join()

    def test_concurrent_client_calls_are_serialized(self):
        c, stub = self.client()
        c.enter()
        entered = threading.Event()
        release = threading.Event()
        calls = []
        errors = []
        def transport(path, body, timeout):
            calls.append(body)
            if len(calls) == 1:
                entered.set()
                if not release.wait(2):
                    raise TimeoutError('test synchronization failed')
            return stub(path, body, timeout)
        c.transport = transport
        def measure(ch):
            try: c.measure(0, 0, ch)
            except Exception as exc: errors.append(exc)
        a = threading.Thread(target=measure, args=(1,))
        b = threading.Thread(target=measure, args=(2,))
        a.start()
        self.assertTrue(entered.wait(2))
        b.start()
        self.assertEqual(len(calls), 1)
        release.set(); a.join(2); b.join(2)
        self.assertFalse(a.is_alive() or b.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(stub.executed, 3)
        self.assertEqual(c.state.virtual_time_s, 11)


if __name__ == '__main__':
    unittest.main()
