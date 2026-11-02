# AP_FLAKE8_CLEAN
"""The twin: charge source + plant + the SSH server and JSON API that expose it."""

import os
import threading
import time

from . import conf as conf_mod
from .device import ChargeSource
from .shell import Shell


class Twin:
    def __init__(self, conf_path=None, plant=None, rate_hz=20.0, log=print, clock=time.monotonic):
        self.conf_path = conf_path
        self.conf = conf_mod.load(conf_path) if conf_path else conf_mod.Conf()
        for w in self.conf.warnings:
            log(f'skycharge.conf: {w}')
        self.plant = plant
        self.device = ChargeSource(self.conf, plant, clock=clock)
        self.shell = Shell(self)
        self.rate_hz = rate_hz
        self.log = log
        self.ssh = None
        self.http = None
        self._stop = threading.Event()
        self._thread = None
        self._event_seq = 0

    def conf_text(self):
        if self.conf_path and os.path.exists(self.conf_path):
            with open(self.conf_path) as f:
                return f.read()
        return ''

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True, name='skycharge-twin')
        self._thread.start()
        if self.ssh:
            self.ssh.start()
        if self.http:
            self.http.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(2)
        if self.ssh:
            self.ssh.stop()
        if self.http:
            self.http.stop()
        if self.plant:
            self.plant.close()

    def power_cycle(self, secs=None):
        self.device.power_cycle(secs)
        if self.ssh:
            # the sessions die with the BeagleBone; do it after the caller's reply had a moment to leave
            threading.Timer(0.3, self.ssh.drop_all).start()

    def _loop(self):
        period = 1.0 / self.rate_hz
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if self.plant is not None:
                self.plant.step(now - last)
            last = now
            self.device.step()
            for seq, t, text in self.device.events_since(self._event_seq):
                self._event_seq = seq
                self.log(f'{time.strftime("%H:%M:%S", time.localtime(t))} {text}')
            self._stop.wait(period)
