# Hardware Notes

## Planned Sensors

- Front TOF sensor
- Left TOF sensor
- Right TOF sensor
- Rear TOF sensor

Record exact sensor model, I2C addresses, GPIO pins, power budget, and mounting offsets here before implementing drivers.

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

## Docking

Docking is expected to use a camera-visible fiducial marker, likely ArUco or AprilTag. Record camera model, marker size, marker family, and dock geometry before implementing pose estimation.
