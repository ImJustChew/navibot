# Architecture

Navibot is organized around replaceable Python components so robot logic can be tested away from Raspberry Pi hardware.

## Major Subsystems

- Robot runtime: owns lifecycle, scheduling, and hardware composition.
- Sensors: exposes normalized readings from four TOF sensors.
- Navigation: converts sensor state into movement commands.
- Docking: detects a fiducial marker and generates alignment commands.
- Control: defines command objects and control services.
- Crypto: owns pairing, secure sessions, and encrypted browser-to-robot messaging.
- Server: serves the portal and exposes API/websocket endpoints.

## Hardware Driver Modules

- `navibot.robot.motors`: motor pins, motor driver, differential drive helper, and PWM voltage guard.
- `navibot.robot.encoders`: direct `lgpio` quadrature encoder callbacks and signed count samples.
- `navibot.sensors.vl53l1x_array`: XSHUT-based VL53L1X address assignment and array readings.
- `navibot.sensors.ina219`: INA219 power/current readings and charging direction helper.

Hardware scripts in `scripts/` should stay thin wrappers around these modules so the future robot runtime and web server use the same tested driver code.

## Encryption Direction

The server should treat browser control payloads as encrypted session messages after pairing. Long term, the browser and robot should negotiate keys through the pairing flow, then exchange encrypted commands and telemetry over websocket or WebRTC data channel.
