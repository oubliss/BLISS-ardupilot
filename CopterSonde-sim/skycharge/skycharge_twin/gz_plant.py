# AP_FLAKE8_CLEAN
"""
The plant in Gazebo: the CopterSonde on the BOLOGNINI S1 pad of the CopterSonde world.

The twin is the charger's electronics; Gazebo is the physics. Over two gz-transport topics (StringMsg
with a JSON object, so no custom message types are needed):

  <command topic>  twin -> Gazebo, 20 Hz: {"mode": "off|scan|charge", "voltage": V, "current_limit": A}
  <sense topic>    Gazebo -> twin:        {"contact": bool, "voltage": V, "current": A,
                                           "pack_voltage": V, "soc": 0..1,
                                           "ambient_temperature": C, "ambient_humidity": %}

The CopterSondePlugin (ardupilot_gazebo, main-CopterSonde) decides the contact from where the vehicle
rests on the pad, computes the charging current with the model in plant.py and adds it to the battery
it reports to SITL. If the twin stops publishing, the plugin turns the output off after 1 s.
"""

import json
import threading
import time

from gz.msgs.stringmsg_pb2 import StringMsg
from gz.transport import Node

from .plant import Measurement, PsuCommand


class GazeboPlant:
    def __init__(self, command_topic='/skycharge/psu', sense_topic='/skycharge/sink', timeout=1.0, log=print):
        self.command_topic = command_topic
        self.sense_topic = sense_topic
        self.timeout = timeout
        self.log = log
        self._lock = threading.Lock()
        self._last = None
        self._last_time = 0.0
        self._ambient = Measurement()
        self._warned = False
        self.node = Node()
        self.pub = self.node.advertise(command_topic, StringMsg)
        if not self.node.subscribe(StringMsg, sense_topic, self._on_sense):
            raise RuntimeError(f'cannot subscribe to {sense_topic}')

    def _on_sense(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        m = Measurement(contact=bool(d.get('contact')), voltage=float(d.get('voltage', 0.0)),
                        current=float(d.get('current', 0.0)),
                        ambient_temperature=float(d.get('ambient_temperature', 25.0)),
                        ambient_humidity=float(d.get('ambient_humidity', 45.0)),
                        pack_voltage=float(d.get('pack_voltage', 0.0)), pack_soc=float(d.get('soc', -1.0)))
        with self._lock:
            self._last = m
            self._last_time = time.monotonic()
            self._ambient = m
            if self._warned:
                self._warned = False
                self.log(f'gazebo: {self.sense_topic} is back')

    def command(self, command: PsuCommand):
        msg = StringMsg()
        msg.data = json.dumps(command.as_dict())
        self.pub.publish(msg)

    def step(self, dt):
        pass

    def measure(self):
        with self._lock:
            fresh = self._last is not None and time.monotonic() - self._last_time < self.timeout
            if fresh:
                return self._last
            if not self._warned:
                self._warned = True
                self.log(f'gazebo: nothing on {self.sense_topic} - is the CopterSonde world running '
                         '(same GZ_PARTITION)?')
            a = self._ambient
            return Measurement(valid=False, ambient_temperature=a.ambient_temperature,
                               ambient_humidity=a.ambient_humidity)

    def close(self):
        self.command(PsuCommand())
