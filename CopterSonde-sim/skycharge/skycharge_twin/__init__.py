# AP_FLAKE8_CLEAN
"""
Skycharge digital twin: the charge source of a Skycharge BOLOGNINI S1 pad, as a black box with the pad's
interfaces (skycharge-cli over SSH, the JSON API) and its behaviour (states, meters, parameters).

  device.py     MUX HW2 state machine, meters and skycharged
  params.py     source and sink parameters, as libskycharge formats and parses them
  conf.py       /etc/skycharge.conf
  cli.py        skycharge-cli
  shell.py      the BeagleBone shell behind SSH, with the twin's own "skytwin" control command
  ssh_server.py SSH server (key-only root login, SHA-1 ssh-rsa only)
  http_api.py   the JSON API (/v1/charging-state ...)
  plant.py      electrical model and the bench plant (a battery inside the twin)
  gz_plant.py   the CopterSonde in Gazebo as the plant
"""
