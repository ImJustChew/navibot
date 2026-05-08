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

Interactive WASD drive:

```bash
python3 scripts/wasd_drive_test.py --speed 0.18 --pulse-seconds 0.15
```

Encoder count watchers:

```bash
python3 scripts/encoder_count_watch.py --wheel left
python3 scripts/encoder_count_watch.py --wheel right
```

PID distance drive:

```bash
python3 scripts/drive_pid_distance_test.py --distance-mm 200 --gear-ratio 105.6 --target-speed-mm-s 50 --min-pwm 0.12 --max-pwm 0.28 --yes
```

VL53L1X TOF sensors:

```bash
python3 scripts/vl53l1x_xshut_read.py
```

INA219 current sensor:

```bash
python3 scripts/ina219_read.py
```

Battery guard:

```bash
python3 scripts/battery_guard.py
```

Use `--dry-run` only for threshold testing; the default behavior powers off on sustained critical battery voltage.

Install the battery guard as a boot service on the Raspberry Pi:

```bash
bash scripts/install_battery_guard_service.sh
```

Unified robot status:

```bash
python3 scripts/robot_status_loop.py --pretty
```
