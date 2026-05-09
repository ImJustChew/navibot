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
NAVIBOT_CAMERA_ENABLED=1
NAVIBOT_CAMERA_WIDTH=320
NAVIBOT_CAMERA_HEIGHT=240
NAVIBOT_CAMERA_FPS=2
NAVIBOT_CAMERA_QUALITY=70
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

## Camera Stream

The robot agent can publish Raspberry Pi Camera frames over the same outbound WSS connection used for telemetry and commands. This is encrypted in transit by TLS and works through Cloud Run without exposing the Pi on the internet.

The first-pass stream is intentionally low bandwidth: JPEG frames, default `320x240` at `2 FPS`. The backend relays each frame to connected operator websocket clients as:

```json
{
  "kind": "video_frame",
  "robotId": "devbot",
  "frame": {
    "contentType": "image/jpeg",
    "data": "base64-jpeg",
    "width": 320,
    "height": 240,
    "sequence": 1,
    "capturedAt": "2026-05-09T10:25:54.000Z"
  },
  "at": "2026-05-09T10:25:54.000Z"
}
```

On Raspberry Pi OS, `picamera2` is installed through apt as `python3-picamera2`. The service installer creates the robot venv with system site packages so that apt-provided camera libraries are visible inside the agent runtime.

This relay is good enough for a live preview and debugging. The long-term path for lower latency and better video quality is still WebRTC, with this Hono backend acting as signaling and presence.

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

## Cloud Run Deployment

Recommended GCP resources:

- Cloud Run service: `navibot-backend`
- Artifact Registry Docker repository: `navibot`
- Secret Manager secrets:
  - `navibot-database-url`
  - `navibot-robot-token`
  - `navibot-operator-token`

Manual Cloud Build deploy:

```bash
gcloud builds submit . \
  --config deploy/cloudbuild-backend-deploy.yaml \
  --substitutions="_REGION=asia-southeast1,_REPOSITORY=navibot,_IMAGE=backend,_SERVICE=navibot-backend,_NAVIBOT_WEB_ORIGIN=https://your-vercel-app.vercel.app"
```

Build-only Cloud Build:

```bash
gcloud builds submit . \
  --config deploy/cloudbuild-backend.yaml \
  --substitutions="_REGION=asia-southeast1,_REPOSITORY=navibot,_IMAGE=backend"
```

GitHub Actions:

- `.github/workflows/ci.yml` runs Python tests, Bun typecheck/build, and Docker image build.
- `.github/workflows/deploy-backend-cloud-run.yml` is a manual workflow for Cloud Run deploys using Workload Identity Federation.

## Security Notes

The first pass uses separate bearer tokens over HTTPS/WSS:

- `NAVIBOT_ROBOT_TOKEN`: private to the Pi and backend.
- `NAVIBOT_OPERATOR_TOKEN`: entered by the operator in the browser at runtime.

Do not put either token in `VITE_*` variables. Vite variables are public browser bundle configuration. REST calls may use the `X-Navibot-Token` header; websocket connections pass the token as a query parameter during the upgrade. WebSocket TLS protects traffic in transit. For lower-latency manual control and production-grade camera video, the next step is to use this Hono backend as WebRTC signaling and move live video/control onto WebRTC media/data channels.
