# AP_FLAKE8_CLEAN
"""
Skycharge source (MUX HW2) and sink parameters: names, order, text formats and parsers.

Everything here follows libskycharge (https://github.com/Skycharge/libskycharge, libskycharge.c and
libskycharge.h, v2.2.2), so the twin prints and accepts exactly what skycharge-cli does:
- names match case-insensitively, with '-' and '_' treated as the same character;
- enumerations print as "Li-Po (0x00)" in plain output and as "Li-Po" in JSON;
- numbers parse like sscanf("0x%x") then sscanf("%u").
"""

import re
from dataclasses import dataclass

# enum sky_psu_type
PSU_TYPES = {1: 'RSP-750-48', 2: 'RSP-1600-48', 3: 'RSP-1600-24', 4: 'RSP-750-24'}
PSU_ALIASES = {'rsp-750-48': 1, 'rsp750-48': 1, 'rsp-750-24': 4, 'rsp750-24': 4,
               'rsp-1600-24': 3, 'rsp1600-24': 3, 'rsp-1600-48': 2, 'rsp1600-48': 2}
# Output voltage range (V) and current limit (A) of each PSU model behind the source board
PSU_LIMITS = {1: (11.0, 57.0, 15.0), 2: (11.0, 57.0, 30.0), 3: (9.6, 30.0, 30.0), 4: (9.6, 30.0, 15.0)}

# enum sky_detect_mode
DETECT_PLC, DETECT_RESISTANCE, DETECT_CAPACITY = 0, 1, 2
DETECT_MODES = {DETECT_PLC: 'PLC', DETECT_RESISTANCE: 'RESISTANCE', DETECT_CAPACITY: 'CAPACITY'}

# enum sky_batt_type: the SDK knows only these two
BATT_LIPO, BATT_LION = 0, 1
BATT_TYPES = {BATT_LIPO: 'Li-Po', BATT_LION: 'Li-Ion'}
BATT_ALIASES = {'li-po': BATT_LIPO, 'lipo': BATT_LIPO, 'li-ion': BATT_LION, 'liion': BATT_LION, 'lion': BATT_LION}

# enum sky_sink_capabilities (bit numbers)
CAP_PLC_WHILE_CHARGING = 0
CAP_EXTENDED_PASSTHRU = 1


class ParamError(ValueError):
    """A value that skycharge-cli refuses to parse."""


@dataclass(frozen=True)
class Param:
    name: str
    kind: str     # uint, bool, calib, hex, psu, detect, batt, caps, uart
    default: int


def calib_point(set_value, read_value):
    return (set_value & 0xffff) << 16 | (read_value & 0xffff)


# enum sky_dev_param for SKY_MUX_HW2, in enum order (the order show-dev-params prints).
# Defaults are the ones on support.skycharge.de/docs/source-board-configuration.
DEV_PARAMS = (
    Param('psu-type', 'psu', 1),
    Param('detect-mode', 'detect', DETECT_CAPACITY),
    Param('psu-fixed-voltage-mv', 'uint', 0),
    Param('psu-fixed-current-ma', 'uint', 0),
    Param('psu-use-fixed-v-i', 'bool', 0),
    Param('nr-bad-heartbeats', 'uint', 20),
    Param('ignore-inval-charging-settings', 'bool', 0),
    Param('ignore-low-batt-voltage', 'bool', 0),
    Param('error-indication-timeout-secs', 'uint', 5),
    Param('keep-silence', 'bool', 0),
    Param('ignore-voltage-on-output', 'bool', 1),
    Param('min-sense-current-ma', 'uint', 50),
    Param('repeat-charge-after-mins', 'uint', 10),
    Param('sense-voltage-calib-point1-mv', 'calib', calib_point(15000, 15000)),
    Param('sense-voltage-calib-point2-mv', 'calib', calib_point(50000, 50000)),
    Param('sense-current-calib-point1-ma', 'calib', calib_point(1000, 1000)),
    Param('sense-current-calib-point2-ma', 'calib', calib_point(10000, 10000)),
    Param('psu-voltage-calib-point1-mv', 'calib', calib_point(10000, 10000)),
    Param('psu-voltage-calib-point2-mv', 'calib', calib_point(20000, 20000)),
    Param('psu-current-calib-point1-ma', 'calib', calib_point(1000, 1000)),
    Param('psu-current-calib-point2-ma', 'calib', calib_point(5000, 5000)),
)

# enum sky_sink_param, in enum order. Defaults from support.skycharge.de/docs/sink-board-configuration.
SINK_PARAMS = (
    Param('capabilities', 'caps', 0),
    Param('batt-type', 'batt', BATT_LIPO),
    Param('batt-capacity-mah', 'uint', 5000),
    Param('batt-min-voltage-mv', 'uint', 18000),
    Param('batt-max-voltage-mv', 'uint', 25200),
    Param('charging-max-current-ma', 'uint', 15000),
    Param('cutoff-min-current-ma', 'uint', 500),
    Param('cutoff-timeout-ms', 'uint', 1000),
    Param('precharge-current-ma', 'uint', 1000),
    Param('precharge-delay-secs', 'uint', 1),
    Param('precharge-secs', 'uint', 30),
    Param('total-charge-secs', 'uint', 7200),
    Param('user-uart-config', 'uart', 0),
    Param('user-data1', 'hex', 0),
    Param('user-data2', 'hex', 0),
    Param('user-data3', 'hex', 0),
    Param('user-data4', 'hex', 0),
)


def _norm(name):
    return name.lower().replace('_', '-')


def find_param(table, name):
    """The Param called name (libskycharge strcasecmp_ignore_dashes), or None."""
    key = _norm(name)
    for p in table:
        if _norm(p.name) == key:
            return p
    return None


def defaults(table):
    return {p.name: p.default for p in table}


def parse_unsigned(text):
    """sscanf("0x%x") then sscanf("%u"): leading hex or decimal digits, the rest is ignored."""
    m = re.match(r'\s*0x([0-9a-fA-F]+)', text)
    if m:
        return int(m.group(1), 16) & 0xffffffff
    m = re.match(r'\s*([+-]?)(\d+)', text)
    if not m:
        raise ParamError(text)
    value = int(m.group(2))
    return (-value if m.group(1) == '-' else value) & 0xffffffff


def parse_bool(text):
    t = text.lower()
    if t in ('false', 'no', '0'):
        return 0
    if t in ('true', 'yes', '1'):
        return 1
    raise ParamError(text)


def parse_value(param, text):
    """The stored integer for text, or ParamError, as sky_*param_value_from_str() decides."""
    kind = param.kind
    if kind in ('uint', 'hex'):
        return parse_unsigned(text)
    if kind == 'bool':
        return parse_bool(text)
    if kind == 'calib':
        m = re.match(r'\s*(\d+):(\d+)', text)
        if not m or int(m.group(1)) > 0xffff or int(m.group(2)) > 0xffff:
            raise ParamError(text)
        return calib_point(int(m.group(1)), int(m.group(2)))
    if kind == 'psu':
        if text.lower() in PSU_ALIASES:
            return PSU_ALIASES[text.lower()]
        v = parse_unsigned(text)
        if v not in PSU_TYPES:
            raise ParamError(text)
        return v
    if kind == 'detect':
        for v, s in DETECT_MODES.items():
            if text.lower() == s.lower():
                return v
        v = parse_unsigned(text)
        if v not in DETECT_MODES:
            raise ParamError(text)
        return v
    if kind == 'batt':
        if text.lower() in BATT_ALIASES:
            return BATT_ALIASES[text.lower()]
        v = parse_unsigned(text)
        if v not in BATT_TYPES:
            raise ParamError(text)
        return v
    if kind == 'caps':
        v = 0
        if 'plc_while_charging' in text.lower():
            v |= 1 << CAP_PLC_WHILE_CHARGING
        if 'extended_passthru' in text.lower():
            v |= 1 << CAP_EXTENDED_PASSTHRU
        if text.lower() == 'nil':
            return 0
        if not v:
            v = parse_unsigned(text)
            if v & ~((1 << CAP_PLC_WHILE_CHARGING) | (1 << CAP_EXTENDED_PASSTHRU)):
                raise ParamError(text)
        return v
    if kind == 'uart':
        m = re.match(r'(\d+)-(\d+)-(.)-(\d+)', text)
        if not m:
            raise ParamError(text)
        baud, word, parity, stop = int(m.group(1)), int(m.group(2)), m.group(3).upper(), int(m.group(4))
        if word not in (8, 9) or stop not in (1, 2) or parity not in 'NOE' or baud >= 1 << 18:
            raise ParamError(text)
        return baud | (word - 8) << 18 | (stop - 1) << 19 | {'N': 0, 'E': 1, 'O': 3}[parity] << 20
    raise ParamError(text)


def format_value(param, v, numeric):
    """Text of a stored value: numeric=True is the plain-output form, False the JSON form."""
    kind = param.kind

    def enum(names, unknown):
        text = names.get(v, unknown)
        return f'{text} (0x{v:02x})' if numeric else text

    if kind == 'uint':
        return str(v)
    if kind == 'hex':
        return f'0x{v:08x}'
    if kind == 'bool':
        return 'true' if v else 'false'
    if kind == 'calib':
        set_value, read_value = v >> 16, v & 0xffff
        return f'{set_value:5d}:{read_value}' if numeric else f'{set_value}:{read_value}'
    if kind == 'psu':
        return enum(PSU_TYPES, 'unknown psu')
    if kind == 'detect':
        return enum(DETECT_MODES, 'unknown mode')
    if kind == 'batt':
        return enum(BATT_TYPES, 'unknown type')
    if kind == 'caps':
        caps = []
        if v & 1 << CAP_PLC_WHILE_CHARGING:
            caps.append('PLC_WHILE_CHARGING')
        if v & 1 << CAP_EXTENDED_PASSTHRU:
            caps.append('EXTENDED_PASSTHRU')
        text = ' | '.join(caps) or 'NIL'
        return f'{text} (0x{v:02x})' if numeric else text
    if kind == 'uart':
        baud, word, stop, parity = v & 0x3ffff, (v >> 18) & 1, (v >> 19) & 1, (v >> 20) & 3
        return f"{baud}-{9 if word else 8}-{'N' if parity == 0 else 'O' if parity == 3 else 'E'}-{2 if stop else 1}"
    return str(v)
