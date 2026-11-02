# AP_FLAKE8_CLEAN
"""
The SSH server of the charge source's BeagleBone (support.skycharge.de/docs/how-to-connect).

As on the real pad, only "root" logs in, only with a public key (password authentication is off),
and by default only with SHA-1 "ssh-rsa" signatures: the pad's OpenSSH predates rsa-sha2-256/512,
which is why LAAIRS has to pin paramiko below 5. --modern-rsa lifts that restriction.

Exec requests run through the twin's shell (skycharge-cli and friends); a shell request gets a
prompt. "skycharge-cli monitor" streams until the client closes the channel.
"""

import base64
import os
import socket
import threading

import paramiko

PROMPT = 'root@skydevice:~# '


def load_authorized_keys(paths):
    """Public keys from OpenSSH authorized_keys files, or derived from private key files."""
    keys = []
    for path in paths:
        with open(path) as f:
            text = f.read()
        if 'PRIVATE KEY' in text:
            keys.append(paramiko.RSAKey.from_private_key_file(path))
            continue
        for line in text.splitlines():
            fields = line.split()
            if len(fields) >= 2 and fields[0] in ('ssh-rsa', 'ssh-ed25519', 'ecdsa-sha2-nistp256'):
                data = base64.b64decode(fields[1])
                keys.append(paramiko.PKey.from_type_string(fields[0], data))
    return keys


def host_key(path):
    """The server's RSA host key, created on first use."""
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        key = paramiko.RSAKey.generate(2048)
        key.write_private_key_file(path)
        return key
    return paramiko.RSAKey.from_private_key_file(path)


class _Server(paramiko.ServerInterface):
    """One per connection. Remembers, per channel, whether it asked for a command or a shell."""

    def __init__(self, ssh):
        self.ssh = ssh
        self.requests = {}
        self.cond = threading.Condition()

    def wait_request(self, channel, timeout):
        """(True, command) for an exec, (True, None) for a shell, (False, None) if neither came."""
        with self.cond:
            self.cond.wait_for(lambda: channel.get_id() in self.requests, timeout)
            if channel.get_id() not in self.requests:
                return False, None
            return True, self.requests[channel.get_id()]

    def _request(self, channel, command):
        with self.cond:
            self.requests[channel.get_id()] = command
            self.cond.notify_all()

    def get_allowed_auths(self, username):
        return 'publickey'

    def check_auth_password(self, username, password):
        return paramiko.AUTH_FAILED

    def check_auth_publickey(self, username, key):
        if username == 'root' and any(key == k for k in self.ssh.authorized):
            return paramiko.AUTH_SUCCESSFUL
        return paramiko.AUTH_FAILED

    def check_channel_request(self, kind, chanid):
        if kind == 'session':
            return paramiko.OPEN_SUCCEEDED
        return paramiko.OPEN_FAILED_ADMINISTRATIVELY_PROHIBITED

    def check_channel_exec_request(self, channel, command):
        self._request(channel, command.decode('utf-8', 'replace') if isinstance(command, bytes) else command)
        return True

    def check_channel_shell_request(self, channel):
        self._request(channel, None)
        return True

    def check_channel_pty_request(self, channel, term, width, height, pixelwidth, pixelheight, modes):
        return True

    def check_channel_env_request(self, channel, name, value):
        return True


class SshServer:
    """Accepts SSH connections on (host, port) and serves each in a thread."""

    def __init__(self, twin, host, port, authorized_keys, host_key_path, legacy_rsa_only=True, log=print):
        self.twin = twin
        self.host = host
        self.port = port
        self.authorized = load_authorized_keys(authorized_keys)
        if not self.authorized:
            raise ValueError('no authorized keys')
        self.host_key = host_key(host_key_path)
        self.legacy_rsa_only = legacy_rsa_only
        self.log = log
        self.transports = set()
        self.lock = threading.Lock()
        self.sock = None
        self.stopping = threading.Event()

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(16)
        self.sock.settimeout(0.5)
        threading.Thread(target=self._accept_loop, daemon=True, name='ssh-accept').start()

    def stop(self):
        self.stopping.set()
        self.drop_all()
        if self.sock:
            self.sock.close()

    def drop_all(self):
        """Close every session, as a rebooting pad does."""
        with self.lock:
            transports = list(self.transports)
        for t in transports:
            t.close()

    def _accept_loop(self):
        while not self.stopping.is_set():
            try:
                client, addr = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if self.twin.device.booting:
                # the BeagleBone is still booting: nothing listens yet
                client.close()
                continue
            threading.Thread(target=self._serve, args=(client, addr), daemon=True, name='ssh-session').start()

    def _serve(self, client, addr):
        disabled = {'pubkeys': ['rsa-sha2-512', 'rsa-sha2-256']} if self.legacy_rsa_only else {}
        transport = paramiko.Transport(client, disabled_algorithms=disabled)
        transport.add_server_key(self.host_key)
        server = _Server(self)
        try:
            transport.start_server(server=server)
        except (paramiko.SSHException, EOFError, OSError) as e:
            self.log(f'ssh: handshake with {addr[0]} failed: {e}')
            transport.close()
            return
        with self.lock:
            self.transports.add(transport)
        try:
            while transport.is_active():
                channel = transport.accept(1.0)
                if channel is None:
                    continue
                threading.Thread(target=self._channel, args=(channel, server), daemon=True,
                                 name='ssh-channel').start()
        finally:
            with self.lock:
                self.transports.discard(transport)
            transport.close()

    def _channel(self, channel, server):
        ok, command = server.wait_request(channel, 10)
        if not ok:
            channel.close()
            return
        stop = threading.Event()
        interactive = command is None

        def send(text, stderr=False):
            data = (text.replace('\n', '\r\n') if interactive else text).encode()
            try:
                if stderr and not interactive:
                    channel.sendall_stderr(data)
                else:
                    channel.sendall(data)
            except (OSError, EOFError):
                stop.set()

        def out(text):
            send(text)

        def err(text):
            send(text, stderr=True)

        if interactive:
            self._interactive(channel, out, err)
            return
        threading.Thread(target=self._watch_close, args=(channel, stop), daemon=True).start()
        self.log(f'ssh: exec {command}')
        rc = self.twin.shell.run(command, out, err, stop)
        try:
            channel.send_exit_status(rc)
        except (OSError, EOFError):
            pass
        channel.close()

    @staticmethod
    def _watch_close(channel, stop):
        # Like sshd, a command outlives EOF on its stdin (paramiko clients send it as soon as the stdin file
        # is dropped); only a closed channel or connection stops it.
        while not channel.closed and channel.get_transport() is not None and channel.get_transport().is_active():
            if stop.wait(0.2):
                return
        stop.set()

    def _interactive(self, channel, out, err):
        out('Skycharge digital twin. "skycharge-cli" drives the charger, "skytwin help" the twin.\n')
        buf = ''
        out(PROMPT)
        while True:
            try:
                data = channel.recv(1024)
            except (OSError, EOFError):
                break
            if not data:
                break
            for ch in data.decode('utf-8', 'replace'):
                if ch in '\r\n':
                    channel.sendall(b'\r\n')
                    line = buf.strip()
                    buf = ''
                    if line in ('exit', 'logout'):
                        channel.send_exit_status(0)
                        channel.close()
                        return
                    if line:
                        self._run_interruptible(channel, line, out, err)
                    out(PROMPT)
                elif ch == '\x03':
                    buf = ''
                    out('^C\n' + PROMPT)
                elif ch in '\x7f\x08':
                    if buf:
                        buf = buf[:-1]
                        channel.sendall(b'\x08 \x08')
                else:
                    buf += ch
                    channel.sendall(ch.encode())
        channel.close()

    def _run_interruptible(self, channel, line, out, err):
        """Run one line; Ctrl-C (or a closed channel) stops a streaming command such as monitor."""
        stop = threading.Event()
        worker = threading.Thread(target=self.twin.shell.run, args=(line, out, err, stop), daemon=True)
        worker.start()
        while worker.is_alive():
            if channel.recv_ready():
                data = channel.recv(64)
                if not data or b'\x03' in data:
                    stop.set()
                    if data:
                        out('^C\n')
            elif channel.closed:
                stop.set()
            worker.join(0.1)
