"""TCP access to the base receiver, standing in for its USB serial port.

LAAIRS opens it as the serial URL socket://HOST:PORT. Every client gets the receiver's epoch output, and
the answers to its own UBX messages; the receiver keeps its configuration when clients come and go, like
the real one when the port is reopened.
"""

# AP_FLAKE8_CLEAN

import socket
import threading
import time

from . import ubx


class BaseServer:
    def __init__(self, receiver, host='127.0.0.1', port=3101, log=print):
        self.receiver = receiver
        self.log = log
        self.lock = threading.Lock()
        self.clients = []
        self.stop = threading.Event()
        self.sock = socket.create_server((host, port), reuse_port=False)
        self.address = self.sock.getsockname()

    def serve_forever(self):
        threading.Thread(target=self.accept_loop, daemon=True).start()
        next_epoch = time.time()
        while not self.stop.is_set():
            next_epoch += self.receiver.meas_period_ms / 1000.0
            delay = next_epoch - time.time()
            if delay < -1:
                next_epoch = time.time()
            elif delay > 0:
                self.stop.wait(delay)
            with self.lock:
                out = self.receiver.epoch(time.time())
                clients = list(self.clients)
            if out:
                for c in clients:
                    self.send(c, out)

    def shutdown(self):
        self.stop.set()
        self.sock.close()
        with self.lock:
            for c in self.clients:
                c.close()

    def accept_loop(self):
        while not self.stop.is_set():
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.clients.append(conn)
            self.log(f'client {addr[0]}:{addr[1]} connected')
            threading.Thread(target=self.client_loop, args=(conn, addr), daemon=True).start()

    def client_loop(self, conn, addr):
        parser = ubx.Parser()
        try:
            while not self.stop.is_set():
                data = conn.recv(4096)
                if not data:
                    break
                for cls, msg_id, payload in parser.feed(data):
                    with self.lock:
                        answer = self.receiver.handle(cls, msg_id, payload, time.time())
                    if answer:
                        self.send(conn, answer)
        except OSError:
            pass
        with self.lock:
            if conn in self.clients:
                self.clients.remove(conn)
        conn.close()
        self.log(f'client {addr[0]}:{addr[1]} disconnected')

    def send(self, conn, data):
        try:
            conn.sendall(data)
        except OSError:
            pass  # the client's reader sees the closed socket and drops it
