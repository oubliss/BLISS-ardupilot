# AP_FLAKE8_CLEAN
"""
The Skycharge JSON API (support.skycharge.de/docs/json, skyhttpd.c), served by the twin itself.

On the real system these requests go to api.skycharge.de, which relays them to the pad. Every request
carries user-uuid (403 if it does not match skycharge.conf); device requests also carry device-uuid
(400 if malformed). Responses are JSON with an "errno" field. Paths work with and without "/v1".
"""

import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qsl, urlparse

from . import params

HELP = '<html><body><h1>Skycharge JSON API</h1><p>See https://support.skycharge.de/docs/json</p></body></html>'


class _Handler(BaseHTTPRequestHandler):
    server_version = 'skyhttpd/2.2.2'

    def log_message(self, fmt, *args):
        self.server.api.log('http: ' + fmt % args)

    def _send(self, status, body=b'', ctype='application/json'):
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, (json.dumps(obj, indent='\t') + '\n').encode())

    def do_GET(self):
        self._dispatch('GET')

    def do_PUT(self):
        self._dispatch('PUT')

    def _dispatch(self, method):
        api = self.server.api
        url = urlparse(self.path)
        path = url.path[3:] if url.path.startswith('/v1/') else url.path
        query = dict(parse_qsl(url.query, keep_blank_values=True))
        if not api.rate_ok(self.client_address[0]):
            self._send(429)
            return
        if query.get('user-uuid') != api.twin.device.conf.user_uuid:
            self._send(403)
            return
        dev_uuid = query.get('device-uuid')
        if dev_uuid is not None:
            try:
                uuid.UUID(dev_uuid)
            except ValueError:
                self._send(400)
                return
        handler = api.routes.get((method, path))
        if handler is None:
            self._send(404, HELP.encode(), 'text/html')
            return
        if path != '/devs-list' and dev_uuid != api.twin.device.conf.device_uuid:
            self._json({'errno': 'ENODEV'})
            return
        if api.twin.device.booting or not api.twin.device.daemon_running:
            self._json({'errno': 'ETIMEDOUT'})
            return
        self._json(handler(query))


class HttpApi:
    def __init__(self, twin, host, port, rate_limit=0.0, log=print):
        self.twin = twin
        self.log = log
        self.rate_limit = rate_limit
        self._hits = {}
        self._lock = threading.Lock()
        self.routes = {
            ('GET', '/devs-list'): self.devs_list,
            ('GET', '/charging-state'): self.charging_state,
            ('GET', '/charging-params'): self.charging_params,
            ('GET', '/sink-charging-params'): self.sink_charging_params,
            ('PUT', '/charging-params'): self.put_charging_params,
            ('PUT', '/sink-charging-params'): self.put_sink_charging_params,
            ('GET', '/scan-resume'): self.scan_resume,
            ('GET', '/scan-stop'): self.scan_stop,
            ('GET', '/droneport-open'): self.no_droneport,
            ('GET', '/droneport-close'): self.no_droneport,
            ('GET', '/droneport-state'): self.no_droneport,
            ('GET', '/gps-data'): self.no_droneport,
        }
        self.httpd = ThreadingHTTPServer((host, port), _Handler)
        self.httpd.daemon_threads = True
        self.httpd.api = self

    def start(self):
        threading.Thread(target=self.httpd.serve_forever, daemon=True, name='http').start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()

    def rate_ok(self, addr):
        """At most rate_limit requests per second per client (0: unlimited); the cloud default is 3."""
        if self.rate_limit <= 0:
            return True
        now = time.monotonic()
        with self._lock:
            hits = [t for t in self._hits.get(addr, []) if now - t < 1.0]
            ok = len(hits) < self.rate_limit
            if ok:
                hits.append(now)
            self._hits[addr] = hits
            return ok

    @property
    def device(self):
        return self.twin.device

    def devs_list(self, query):
        c = self.device.conf
        return {'errno': 'SUCCESS', 'devices': [{'device-uuid': c.device_uuid, 'device-name': c.device_name}]}

    def charging_state(self, query):
        st = self.device.charging_state()
        return {'errno': 'SUCCESS', 'charging-state': {
            'state': st['state'].name,
            'reason': st['reason'].name,
            'voltage-mV': st['voltage_mV'],
            'current-mA': st['current_mA'],
            'charging-time-sec': st['charging_secs'],
            'state-of-charge-perc': st['state_of_charge'],
            'until-full-charge-time-sec': st['until_full_secs'],
            'source-temperature-C': st['mux_temperature_C'],
            'sink-temperature-C': st['sink_temperature_C'],
            'energy-mWh': st['energy_mWh'],
            'charge-mAh': st['charge_mAh'],
            'tx-bytes': 0, 'tx-packets': 0, 'tx-err-bytes': 0, 'tx-err-packets': 0,
            'rx-bytes': 0, 'rx-packets': 0, 'rx-err-bytes': 0, 'rx-err-packets': 0,
        }}

    @staticmethod
    def _params(table, values):
        return {p.name: params.format_value(p, values[p.name], False) for p in table}

    def charging_params(self, query):
        return {'errno': 'SUCCESS', 'params': self._params(params.DEV_PARAMS, self.device.get_dev_params())}

    def sink_charging_params(self, query):
        if self.device.get_dev_params()['detect-mode'] == params.DETECT_PLC and not self.device.sink_linked():
            return {'errno': 'ENOTCONN'}
        return {'errno': 'SUCCESS', 'params': self._params(params.SINK_PARAMS, self.device.get_sink_params())}

    def _put(self, query, table, setter):
        values = {}
        for key, value in query.items():
            if key in ('user-uuid', 'device-uuid'):
                continue
            p = params.find_param(table, key)
            if p is None:
                return {'errno': 'EINVAL'}
            try:
                values[p.name] = params.parse_value(p, value)
            except params.ParamError:
                return {'errno': 'EINVAL'}
        if values:
            setter(values)
        return {'errno': 'SUCCESS'}

    def put_charging_params(self, query):
        return self._put(query, params.DEV_PARAMS, self.device.set_dev_params)

    def put_sink_charging_params(self, query):
        if self.device.get_dev_params()['detect-mode'] == params.DETECT_PLC and not self.device.sink_linked():
            return {'errno': 'ENOTCONN'}
        return self._put(query, params.SINK_PARAMS, self.device.set_sink_params)

    def scan_resume(self, query):
        self.device.scan_resume()
        return {'errno': 'SUCCESS'}

    def scan_stop(self, query):
        self.device.scan_stop()
        return {'errno': 'SUCCESS'}

    def no_droneport(self, query):
        return {'errno': 'EOPNOTSUPP'}
