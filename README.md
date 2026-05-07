# Navibot

Python monorepo for a Raspberry Pi controlled robot with four time-of-flight sensors, fiducial-marker self docking, and an encrypted web control portal.

## Repository Layout

- `src/navibot/robot`: robot runtime, hardware abstraction, lifecycle orchestration.
- `src/navibot/sensors`: sensor drivers and readings, including TOF range sensors.
- `src/navibot/navigation`: obstacle avoidance and navigation state machines.
- `src/navibot/docking`: fiducial marker detection and docking alignment logic.
- `src/navibot/control`: command models and robot control services.
- `src/navibot/crypto`: encryption, pairing, and secure session primitives.
- `src/navibot/server`: Python web server API and websocket entry points.
- `apps/robot`: executable robot runtime entry point.
- `apps/webserver`: executable web server entry point.
- `web`: browser portal templates and static assets.
- `tests`: unit, integration, and future end-to-end tests.
- `docs`: architecture and hardware notes.

## Development

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev,server,vision]"
pytest
```

## Runtime Entry Points

```powershell
python -m apps.robot
python -m apps.webserver
```

## Raspberry Pi Hardware Tests

Install Pi GPIO dependencies on the Raspberry Pi:

```bash
python -m pip install -e ".[rpi]"
```

Lift the robot so the wheels cannot drive away, then run the motor smoke test:

```bash
python scripts/motor_smoke_test.py
```

Use `--yes` to skip the confirmation prompt after you trust the setup:

```bash
python scripts/motor_smoke_test.py --speed 0.25 --step-seconds 1 --yes
```
