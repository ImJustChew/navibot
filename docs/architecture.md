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

## Remote Access Architecture

The web UI will be a Vite + React + shadcn application, likely hosted on Vercel. The robot cannot assume the browser is on the same LAN, and it must support realtime video, realtime control, and persistent status.

Use two channels:

- WebRTC for realtime encrypted sessions: camera video, low-latency drive commands, and live telemetry.
- Cloud API/database for durable state: online/offline status, battery history, map snapshots, settings, alarms, display/music configuration, and event logs.

Recommended high-level topology:

```text
Vercel React UI
   |                     |
   | HTTPS API           | WebRTC DTLS-SRTP/DataChannel
   v                     v
Cloud state DB <--> Signaling/TURN <--> Robot Pi agent
```

The signaling server handles authentication, robot presence, SDP/ICE exchange, and pairing. It should not be the long-term storage backend. The robot and browser exchange video/control/telemetry over WebRTC, which encrypts media and data channels by default. A TURN server such as `coturn` is required for reliable access through restrictive NATs; TURN relays encrypted WebRTC packets and should not see decoded video/control payloads.

Persistent state should be lightweight. Supabase is a good fit because Postgres works well for telemetry/history, Supabase Storage can hold map snapshots, and Vercel integration is straightforward.

Suggested persistent data:

- `robots`: name, online state, last seen time, battery/current, charging state, mode, firmware version.
- `telemetry_samples`: timestamped battery, power, TOF, encoder, and pose samples.
- `maps`: map version, storage path, resolution, origin, and metadata.
- `robot_settings`: safety distance, max PWM, display brightness, speaker volume, timezone.
- `alarms`: time, recurrence, sound, enabled state.
- `commands`: non-realtime command queue for alarms, display, music, mapping, docking, and configuration.

Manual driving should never depend on cloud database writes. Use WebRTC data channels for realtime control and keep safety local on the robot:

- command heartbeat / deadman timeout
- max PWM and voltage caps
- TOF obstacle stop
- encoder stall/skew stop
- low battery behavior
- emergency stop

Non-urgent actions, such as setting alarms or display brightness, may go through the cloud command queue and be acknowledged by the robot agent.
