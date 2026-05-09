# Vercel Web Deployment

Deploy `apps/web` as the Vercel project root.

Build settings:

- Framework preset: Vite
- Install command: `bun install --frozen-lockfile`
- Build command: `bun run build`
- Output directory: `dist`

Environment variables:

```bash
VITE_BACKEND_HTTP_URL=https://navibot-backend-xxxxx-as.a.run.app
VITE_BACKEND_WS_URL=wss://navibot-backend-xxxxx-as.a.run.app
VITE_ROBOT_ID=devbot
```

Do not set operator or robot tokens as `VITE_*` variables. The operator token is entered in the browser at runtime.
