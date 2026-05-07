# Hardware Notes

## Planned Sensors

- Front TOF sensor
- Left TOF sensor
- Right TOF sensor
- Rear TOF sensor

Record exact sensor model, I2C addresses, GPIO pins, power budget, and mounting offsets here before implementing drivers.

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

## Encoder Motor Accuracy Test

The encoder-paired movement test is `scripts/encoder_motor_accuracy_test.py`. It drives each wheel forward and reverse until the paired encoder reaches a target count.

Default assumptions:

- Left encoder: A GPIO 27, B GPIO 22
- Right encoder: A GPIO 23, B GPIO 24
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

## Docking

Docking is expected to use a camera-visible fiducial marker, likely ArUco or AprilTag. Record camera model, marker size, marker family, and dock geometry before implementing pose estimation.
