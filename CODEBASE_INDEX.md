# Codebase index

## Project

SwarmControl is a Python ground control station for ArduPilot/PX4 drones over MAVLink. The CLI connects to SITL TCP endpoints or real serial telemetry, runs telemetry handlers and swarm commands, and serves a static browser dashboard over HTTP plus telemetry/command WebSocket. An optional Tkinter cockpit can be enabled with `--control`.

## Start here

- [README.md](README.md): setup, launch modes, operational workflows, CLI flags.
- [DroneContUI.py](DroneContUI.py): compatibility launcher and entry point.
- [swarmcontrol/cli.py](swarmcontrol/cli.py): argument parsing and application startup.
- [swarmcontrol/config.py](swarmcontrol/config.py): shared rates, timeouts, URLs, and safety constants.

## Python modules

| Path | Responsibility |
| --- | --- |
| [swarmcontrol/connections.py](swarmcontrol/connections.py) | Opens SITL and serial MAVLink connections; shared serial multiplexing and heartbeat watchdog/reconnect. |
| [swarmcontrol/telemetry.py](swarmcontrol/telemetry.py) | Per-drone telemetry state, MAVLink stream requests, and message processing. |
| [swarmcontrol/swarm.py](swarmcontrol/swarm.py) | Leader selection, manual follow, autonomous missions, swarm commands, and inactivity RTL watchdog. |
| [swarmcontrol/formation.py](swarmcontrol/formation.py) | Formation offsets and 3D separation/collision calculations. |
| [swarmcontrol/gui.py](swarmcontrol/gui.py) | Optional Tkinter cockpit integration. |
| [commands.py](commands.py) | MAVLink arm, disarm, takeoff, GPS navigation, RTL, and land commands. |
| [web_server.py](web_server.py) | Integrated static HTTP server and WebSocket server; broadcasts telemetry/status/logs and routes commands. |
| [websocket_protocol.py](websocket_protocol.py) | Low-level WebSocket handshake and frame protocol helpers. |
| [static/index.html](static/index.html) | Main web dashboard UI and client-side controls. |
| [static/map.html](static/map.html) | Map view. |

## Runtime flow

1. `DroneContUI.py` calls `swarmcontrol.cli.main()`.
2. CLI parses mode and drone targets, establishes connections, builds telemetry handlers, and starts the dashboard (plus optional GUI/browser).
3. Telemetry updates feed the dashboard; WebSocket commands are dispatched into swarm command handling.
4. Swarm logic calculates formations and mission waypoints, then calls the MAVLink helpers in `commands.py`.

## Project configuration

- [pyproject.toml](pyproject.toml): package metadata and Python dependencies (uv project is configured as non-package).
- [uv.lock](uv.lock), [requirements.txt](requirements.txt): locked and legacy dependency manifests.
- [setup_venv.bat](setup_venv.bat): legacy Windows environment setup.
