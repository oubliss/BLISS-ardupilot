# AP_FLAKE8_CLEAN
"""
The physical side of the charger: what the source board's PSU output is connected to.

The source board commands its PSU (PsuCommand) and senses the output (Measurement). Between them
are the pad tiles, the contact unit, the sink board and the battery. Two plants implement that:

- StandalonePlant (here): a battery and a contact switch inside the twin, for bench use and tests.
- GazeboPlant (gz_plant.py): the CopterSonde in Gazebo, whose CopterSondePlugin owns the battery
  and decides the contact from the vehicle's pose on the BOLOGNINI S1 pad.

Both use the same electrical model. In CV the PSU holds V_set at its terminals; in CC it limits the
current to I_lim. With the open-circuit voltage OCV, internal resistance R_int, contact/wire/sink
resistance R_path, and the drone's own load I_load (the flight controller stays powered on the pad),
the charging current is

    I = clamp((V_set - OCV + R_int * I_load) / (R_path + R_int), 0, I_lim)

The battery terminal voltage is OCV + R_int * (I - I_load), and the PSU terminal voltage, which the
source board reports, is that plus R_path * I. The sink has reverse-current protection, so the
current never flows back into the pad.
"""

import threading
from dataclasses import dataclass

# Open-circuit cell voltage against state of charge (0..1)
OCV_CURVES = {
    'Li-Po': ((0.0, 3.27), (0.05, 3.61), (0.10, 3.69), (0.20, 3.73), (0.30, 3.77), (0.40, 3.79),
              (0.50, 3.82), (0.60, 3.87), (0.70, 3.92), (0.80, 3.98), (0.90, 4.06), (1.0, 4.20)),
    'Li-Ion': ((0.0, 3.00), (0.05, 3.30), (0.10, 3.45), (0.20, 3.55), (0.30, 3.62), (0.40, 3.68),
               (0.50, 3.74), (0.60, 3.81), (0.70, 3.89), (0.80, 3.97), (0.90, 4.06), (1.0, 4.20)),
}


def cell_ocv(soc, chemistry='Li-Ion'):
    curve = OCV_CURVES.get(chemistry, OCV_CURVES['Li-Ion'])
    soc = min(max(soc, 0.0), 1.0)
    for (s0, v0), (s1, v1) in zip(curve, curve[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return curve[-1][1]


@dataclass
class PsuCommand:
    """What the source board asks of its output stage."""
    mode: str = 'off'          # off | scan (detection voltage, no power) | charge
    voltage: float = 0.0       # V, CV set point
    current_limit: float = 0.0  # A, CC limit

    def as_dict(self):
        return {'mode': self.mode, 'voltage': round(self.voltage, 3), 'current_limit': round(self.current_limit, 3)}


@dataclass
class Measurement:
    """What the source board senses, plus the environment of the pad."""
    contact: bool = False      # a sink is connected across the tiles
    voltage: float = 0.0       # V at the PSU terminals while charging
    current: float = 0.0       # A delivered
    ambient_temperature: float = 25.0   # deg C at the pad
    ambient_humidity: float = 45.0      # %
    valid: bool = True         # False when the simulator is not talking to us
    pack_voltage: float = 0.0  # V at the battery terminals (truth, for the twin's own log)
    pack_soc: float = -1.0     # 0..1 (truth), -1 unknown


def charge_current(command, ocv, r_int, r_path, load):
    """Current into the sink for a PSU command, from the model in the module docstring."""
    if command.mode != 'charge' or command.current_limit <= 0:
        return 0.0
    i = (command.voltage - ocv + r_int * load) / (r_path + r_int)
    return min(max(i, 0.0), command.current_limit)


class StandalonePlant:
    """A battery on a contact unit, entirely inside the twin."""

    def __init__(self, cells=6, capacity_ah=9.0, soc=0.6, chemistry='Li-Ion', r_int=0.03, r_path=0.08,
                 load=0.8, ambient_temperature=25.0, ambient_humidity=45.0, contact=False):
        self.cells = cells
        self.capacity_ah = capacity_ah
        self.soc = soc
        self.chemistry = chemistry
        self.r_int = r_int
        self.r_path = r_path
        self.load = load
        self.ambient_temperature = ambient_temperature
        self.ambient_humidity = ambient_humidity
        self.contact = contact
        self._command = PsuCommand()
        self._current = 0.0
        self._lock = threading.Lock()

    def ocv(self):
        return self.cells * cell_ocv(self.soc, self.chemistry)

    def command(self, command):
        with self._lock:
            self._command = command

    def step(self, dt):
        with self._lock:
            cmd = self._command if self.contact else PsuCommand()
            i = charge_current(cmd, self.ocv(), self.r_int, self.r_path, self.load)
            self._current = i
            drawn = self.load if self.load > 0 else 0.0
            self.soc = min(max(self.soc + (i - drawn) * dt / 3600.0 / self.capacity_ah, 0.0), 1.0)

    def measure(self):
        with self._lock:
            i = self._current
            pack = self.ocv() + self.r_int * (i - self.load)
            return Measurement(contact=self.contact, voltage=pack + self.r_path * i if i > 0 else 0.0,
                               current=i, ambient_temperature=self.ambient_temperature,
                               ambient_humidity=self.ambient_humidity, pack_voltage=pack, pack_soc=self.soc)

    def place(self, on=True):
        with self._lock:
            self.contact = on

    def close(self):
        pass
