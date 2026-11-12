import json
import threading
import unittest

from web_server import IntegratedServer


class ReplayTests(unittest.TestCase):
    def test_new_client_gets_merged_snapshot_not_old_queue(self):
        server = IntegratedServer()
        first, second = object(), object()
        server.add_client(first)
        server.broadcast_status({"mode": "SITL", "leader": 1})
        server.broadcast_status({"leader": 2})
        server.broadcast_telemetry_dedup({"2": {"alt": 10}})
        server.add_client(second)
        received = []
        while not server._out_queue.empty():
            message, recipients = server._out_queue.get_nowait()
            if second in recipients:
                received.append(json.loads(message))
                self.assertEqual(recipients, (second,))
        self.assertEqual(
            received,
            [
                {"type": "status", "data": {"mode": "SITL", "leader": 2}},
                {"type": "telemetry", "data": {"2": {"alt": 10}}},
            ],
        )

    def test_slow_sender_does_not_lock_producers(self):
        server = IntegratedServer()
        sending, release, produced = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )

        class Client:
            def send_message(self, message):
                sending.set()
                release.wait(2)

        server.add_client(Client())
        server.broadcast_status({"mode": "SITL"})
        sender = threading.Thread(target=server._sender_loop, daemon=True)
        sender.start()
        try:
            self.assertTrue(sending.wait(1))

            def produce():
                server.broadcast_status({"leader": 2})
                produced.set()

            threading.Thread(target=produce, daemon=True).start()
            self.assertTrue(produced.wait(1))
        finally:
            server.running = False
            release.set()
            sender.join(2)


if __name__ == "__main__":
    unittest.main()
