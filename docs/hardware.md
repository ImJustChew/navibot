# Hardware Notes

## Planned Sensors

- Front TOF sensor
- Left TOF sensor
- Right TOF sensor
- Rear TOF sensor

Record exact sensor model, I2C addresses, GPIO pins, power budget, and mounting offsets here before implementing drivers.

## TOF Sensor Bring-Up

The four VL53L1X sensors share I2C1 and use XSHUT pins so each can be assigned a unique address at boot.

Default mapping:

- `left45`: XSHUT GPIO 25, I2C `0x30`
- `front`: XSHUT GPIO 8, I2C `0x31`
- `right45`: XSHUT GPIO 7, I2C `0x32`
- `back`: XSHUT GPIO 1, I2C `0x33`

Run:

```bash
python3 scripts/vl53l1x_xshut_read.py
```

Override mappings with repeated `--sensor name:xshut_gpio:i2c_address` arguments.

## TOF Mapping Drive Test

`scripts/tof_mapping_drive_test.py` is an early self-navigation proof. It is not full SLAM. It drives in short cautious pulses, estimates pose from wheel encoders, projects TOF readings into a 2D point map, and writes artifacts to `artifacts/maps/latest`.

Run in a clear test area:

```bash
python3 scripts/tof_mapping_drive_test.py --max-steps 120 --max-seconds 60 --speed 0.14 --turn-speed 0.14
```

Outputs:

- `map.json`: metadata, robot path, and projected TOF points.
- `points.csv`: projected obstacle points.
- `path.csv`: robot pose and sensor readings per step.

This test assumes a rough wheel track of `105 mm` and gear ratio `105.6`. Calibrate those values before treating the map as accurate.

## Self Exploration Map Test

`scripts/self_explore_room.py` is a more autonomous room exploration proof. It maintains a simple occupancy grid from TOF rays, records encoder odometry, and stops when it has not discovered new grid cells for a sustained period.

Run in a clear, supervised test area:

```bash
python3 scripts/self_explore_room.py --max-seconds 180 --max-steps 300 --speed 0.13 --turn-speed 0.13
```

Outputs are written to `artifacts/explore/latest`:

- `map.json`: metadata, occupancy grid, robot path, and TOF hit points.
- `map.html`: standalone canvas viewer for the map.

This is still not full SLAM. It is a demonstration of cautious self-navigation, encoder odometry, TOF coverage, and map artifact generation.

## INA219 Current Sensor

The INA219 current sensor is on I2C address `0x40`.

Run:

```bash
python3 scripts/ina219_read.py
```

The script prints bus voltage, shunt voltage, estimated load voltage, current, and power.

Continuous battery guard:

```bash
python3 scripts/battery_guard.py
```

By default the guard powers off the Raspberry Pi after sustained critical voltage. This is intentional battery protection. Use `--dry-run` only when testing thresholds:

```bash
python3 scripts/battery_guard.py --dry-run
```

Recommended service command:

```bash
python3 scripts/battery_guard.py --critical-voltage 6.2 --critical-seconds 30
```

The guard ignores low voltage while the INA219 reports negative current, because that indicates charging/backfeed. The same `BatteryMonitor` subsystem is used by the robot status loop.

Install as a systemd service:

```bash
bash scripts/install_battery_guard_service.sh
```

Useful service commands:

```bash
sudo systemctl status navibot-battery-guard
sudo journalctl -u navibot-battery-guard -f
sudo systemctl restart navibot-battery-guard
```

Observed baseline behavior:

- Normal robot draw is positive current. In one idle/running sample, the bus was about `6.48 V`, current was about `+0.44 A`, and power was about `2.85 W`.
- Charging or backfeed appears as negative current. In one charging sample, the bus was about `7.01 V`, current was about `-1.10 A`, and power magnitude was about `7.8 W`.
- Treat the current sign as directional: positive means battery/output supplying the robot load, negative means current flowing back through the sensor toward the battery/charger side.

## Wheels And Encoders

- Wheel: D-shaped shaft rubber tire accessory model, color option `43MM轮子一个`.
- Wheel diameter: 43 mm.
- Encoder: 7 pulses per pin for one motor-shaft rotation.

The GA12-N20 encoder is mounted on the motor shaft before the gearbox. With x4 quadrature decoding, counts per final wheel rotation are:

```text
wheel_counts_per_rev = 7 pulses/channel * 4 edges/pulse * gear_ratio
```

For a 100:1 gearbox, that is `7 * 4 * 100 = 2800` counts per wheel rotation. Set the actual gearbox ratio in PID distance tests before relying on distance accuracy.

## Motor Smoke Test

The initial motor test script is `scripts/motor_smoke_test.py`. It uses BCM GPIO numbering and the pins recorded in `docs/gpio.md` by default:

- Left motor: PWM GPIO 13, IN1 GPIO 26, IN2 GPIO 19
- Right motor: PWM GPIO 12, IN1 GPIO 20, IN2 GPIO 21
- Standby: GPIO 16

Run it only with the robot lifted off the ground:

```bash
python -m pip install -e ".[rpi]"
python scripts/motor_smoke_test.py
```

## WASD Drive Test

For interactive console control on the Raspberry Pi:

```bash
python3 scripts/wasd_drive_test.py --speed 0.18 --pulse-seconds 0.15
```

Controls are `W` forward, `S` reverse, `A` rotate left, `D` rotate right, space stop, and `Q` quit. Each keypress moves for a short pulse and then stops unless more keys are pressed.

## Encoder Motor Accuracy Test

The encoder-paired movement test is `scripts/encoder_motor_accuracy_test.py`. It drives each wheel forward and reverse until the paired encoder reaches a target count.

Default assumptions:

- Left encoder: A GPIO 23, B GPIO 24
- Right encoder: A GPIO 27, B GPIO 22
- Right encoder direction is inverted in software.
- Target count: 7 A-channel rising pulses per wheel rotation

Run with the robot lifted:

```bash
python scripts/encoder_motor_accuracy_test.py --rotations 1 --speed 0.25
```

For short powered tests:

```bash
python scripts/encoder_motor_accuracy_test.py --rotations 1 --speed 0.20 --timeout-seconds 5 --yes
```

## PID Distance Drive Test

The paired wheel drive test is `scripts/drive_pid_distance_test.py`. It decodes both A and B encoder channels using x4 quadrature, estimates distance from wheel diameter and gear ratio, and drives both motors forward with per-wheel PID speed control plus left/right synchronization.

The script does not have a simple `--speed` option because speed is controlled through PID targets. Example for a short 200 mm test using a 100:1 gearbox assumption:

```bash
python scripts/drive_pid_distance_test.py --distance-mm 200 --gear-ratio 100 --target-speed-mm-s 80 --min-pwm 0.18 --max-pwm 0.45 --yes
```

If one wheel runs backward relative to the other, add `--left-motor-inverted` or `--right-motor-inverted`. If encoder counts have the wrong sign, add the matching `--left-encoder-inverted` or `--right-encoder-inverted`.

The script brakes a wheel when it reaches target. To test motor-driver behavior without active braking, use `--coast-on-stop`.

Current chassis wiring requires the left motor to be inverted for forward motion. `scripts/drive_pid_distance_test.py` defaults to left motor inverted; use `--left-motor-normal` only if the wiring changes.

The PID script lets an ahead wheel coast before target and aborts if left/right encoder counts diverge too far. Tune that guard with `--max-skew-counts`.

Current encoder wiring maps left encoder to GPIO 23/24 and right encoder to GPIO 27/22. The right encoder direction is inverted in software.

The GA12-N20 motors are rated for 6 V max. With a 7.4 V supply, keep commanded drive PWM at or below:

```text
6.0V / 7.4V = 0.811 PWM
```

`scripts/drive_pid_distance_test.py` enforces this with `--supply-voltage` and `--motor-voltage-limit`. For early testing, stay much lower than the absolute cap, such as `--max-pwm 0.28`.

PID tuning notes:

- If the log shows both motors at `min_pwm` most of the run, PID is saturated at the floor. Lower `--min-pwm` until the robot just barely moves reliably, then set it slightly above that.
- If the robot oscillates left/right, lower `--sync-gain`.
- If one side drifts away slowly, raise `--sync-gain` a little.
- If speed overshoots the target speed, lower `--kp` or lower `--min-pwm`.
- Tune with short distances first, such as 100-200 mm, before increasing speed or distance.

## Encoder Performance

`scripts/encoder_count_watch.py` and `scripts/drive_pid_distance_test.py` use direct `lgpio` alert callbacks for encoder checks and PID distance testing. Older diagnostic scripts still using GPIO Zero wrappers should be migrated to the same lower-level counter path before relying on odometry.

Direct `lgpio` callbacks are better than GPIO Zero device callbacks for encoder counting, but Raspberry Pi Linux userspace is still not real-time. At higher wheel speeds, Python callbacks can miss quadrature edges, especially while also running motor control, networking, camera, or web-server work.

For production odometry, prefer one of these approaches:

- A small microcontroller, such as RP2040, Arduino, or ESP32, decodes both wheel encoders and sends count snapshots to the Pi over UART/I2C/SPI.
- A dedicated quadrature counter IC handles encoder counting in hardware.
- A lower-level Pi GPIO backend, such as `lgpio` or `pigpio`, timestamps edges outside the Python control loop. This is better than GPIO Zero wrappers or Python polling, but still does not make Linux real-time.

Keep Python callbacks short: only update counters in the callback, then do PID/control math in the main loop from sampled counter values.

## Docking

Docking is expected to use a camera-visible fiducial marker, likely ArUco or AprilTag. Record camera model, marker size, marker family, and dock geometry before implementing pose estimation.
