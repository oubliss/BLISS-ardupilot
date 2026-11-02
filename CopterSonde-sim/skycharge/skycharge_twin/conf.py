# AP_FLAKE8_CLEAN
"""
/etc/skycharge.conf: the persistent configuration that skycharged loads at start.

Source settings carry the prefix "mux-hw2-", sink settings "sink-"
(support.skycharge.de/docs/configuration-file). Runtime changes made with set-dev-param(s) and
set-sink-param(s) are lost when skycharged restarts, because it writes this file back to the boards.
The twin keeps that behaviour: a simulated power cycle reloads this file.
"""

import uuid
import zlib
from dataclasses import dataclass, field

from . import params


@dataclass
class Conf:
    path: str = ''
    user_uuid: str = '00000000-0000-0000-0000-000000000000'
    device_uuid: str = ''
    device_name: str = 'skydevice'
    broker_url: str = ''
    mux_type: str = 'hw2'
    mux_dev: str = '/dev/ttyS4'
    dev: dict = field(default_factory=lambda: params.defaults(params.DEV_PARAMS))
    sink: dict = field(default_factory=lambda: params.defaults(params.SINK_PARAMS))
    warnings: list = field(default_factory=list)

    @property
    def dev_id(self):
        """The 8-hex-digit ID that list-devs prints and --id selects (a CRC32 of the device)."""
        return f'{zlib.crc32((self.device_uuid + self.device_name).encode()) & 0xffffffff:08X}'


def load(path=None, text=None):
    """Parse a skycharge.conf file (or its text). Unknown keys are kept as warnings, like skycharged."""
    conf = Conf(path=path or '')
    if text is None and path:
        with open(path) as f:
            text = f.read()
    for lineno, raw in enumerate((text or '').splitlines(), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if '=' not in line:
            conf.warnings.append(f'line {lineno}: no "=" in "{raw.strip()}"')
            continue
        key, value = (s.strip() for s in line.split('=', 1))
        value = value.strip('<>').strip()
        try:
            if key == 'user-uuid':
                conf.user_uuid = value
            elif key == 'device-uuid':
                conf.device_uuid = value
            elif key == 'device-name':
                conf.device_name = value[:15]
            elif key == 'broker-url':
                conf.broker_url = value
            elif key == 'mux-type':
                conf.mux_type = value
            elif key == 'mux-dev':
                conf.mux_dev = value
            elif key.startswith('mux-hw2-') and params.find_param(params.DEV_PARAMS, key[8:]):
                p = params.find_param(params.DEV_PARAMS, key[8:])
                conf.dev[p.name] = params.parse_value(p, value)
            elif key.startswith('sink-') and params.find_param(params.SINK_PARAMS, key[5:]):
                p = params.find_param(params.SINK_PARAMS, key[5:])
                conf.sink[p.name] = params.parse_value(p, value)
            else:
                conf.warnings.append(f'line {lineno}: unknown setting "{key}"')
        except params.ParamError:
            conf.warnings.append(f'line {lineno}: bad value "{value}" for "{key}"')
    if not conf.device_uuid:
        conf.device_uuid = str(uuid.uuid5(uuid.NAMESPACE_DNS, conf.device_name + '.skycharge-twin'))
    return conf
