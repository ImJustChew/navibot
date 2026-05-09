# Remote Control Stack

This stack is designed for a Vercel-hosted web UI and a Raspberry Pi robot that can be controlled from anywhere without port forwarding.

```text
Vercel React app
  | HTTPS + WSS
  v
Hono backend relay/signaling service
  | Neon Postgres via Drizzle
  ^
  | outbound WSS
Raspberry Pi robot agent
```

## Packages

- `apps/web`: Vite + React control portal for Vercel.
- `apps/backend`: Hono backend with websocket relay endpoints and Drizzle schema for Neon Postgres.
- `apps/robot`: Python robot agent entrypoint. It connects outbound to the backend and executes realtime commands locally.

## Environment

Set these in the backend host:

```bash
NAVIBOT_ROBOT_TOKEN=replace-with-long-random-robot-token
NAVIBOT_OPERATOR_TOKEN=replace-with-long-random-operator-token
NAVIBOT_WEB_ORIGIN=https://your-vercel-app.vercel.app
DATABASE_URL=postgresql://...neon.tech/navibot?sslmode=require
PORT=8787
```

Set these in Vercel for `apps/web`:

```bash
VITE_BACKEND_HTTP_URL=https://your-backend.example.com
VITE_BACKEND_WS_URL=wss://your-backend.example.com
VITE_ROBOT_ID=devbot
```

Set these on the Pi:

```bash
NAVIBOT_BACKEND_URL=wss://your-backend.example.com
NAVIBOT_ROBOT_ID=devbot
NAVIBOT_ROBOT_TOKEN=replace-with-long-random-robot-token
```

## Local Development

Install Bun packages:

```bash
bun install
```

Run the backend:

```bash
bun run dev:backend
```

Run the web app:

```bash
bun run dev:web
```

Run the robot agent in mock mode:

```bash
python -m apps.robot --mock --backend-url ws://localhost:8787
```

Run on the Pi with hardware:

```bash
python -m apps.robot
```

## Database

The Drizzle schema lives at `apps/backend/src/db/schema.ts`.

Generate migrations:

```bash
bun run db:generate
```

Apply migrations to Neon:

```bash
bun run db:migrate
```

The backend still runs without `DATABASE_URL`; it will relay commands and telemetry in memory only. Configure Neon before expecting durable status, command history, telemetry history, or map snapshots.

## Security Notes

The first pass uses separate bearer tokens over HTTPS/WSS:

- `NAVIBOT_ROBOT_TOKEN`: private to the Pi and backend.
- `NAVIBOT_OPERATOR_TOKEN`: entered by the operator in the browser at runtime.

Do not put either token in `VITE_*` variables. Vite variables are public browser bundle configuration. WebSocket TLS protects traffic in transit. For camera video and lower-latency manual control, the next step is to use this Hono backend as WebRTC signaling and move live video/control onto WebRTC media/data channels.
