# Hardware Notes

## Planned Sensors

- Front TOF sensor
- Left TOF sensor
- Right TOF sensor
- Rear TOF sensor

Record exact sensor model, I2C addresses, GPIO pins, power budget, and mounting offsets here before implementing drivers.

## Wheels And Encoders

- Wheel: D-shaped shaft rubber tire accessory model, color option `43MM轮子一个`.
- Encoder: 7 pulses per pin for one wheel rotation.

Use 7 pulses per rotation when counting a single encoder channel, such as A-channel rising edges. If later tests count both A and B edges or all quadrature transitions, update the effective pulses-per-rotation value in test commands and calibration docs.

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

## Docking

Docking is expected to use a camera-visible fiducial marker, likely ArUco or AprilTag. Record camera model, marker size, marker family, and dock geometry before implementing pose estimation.
