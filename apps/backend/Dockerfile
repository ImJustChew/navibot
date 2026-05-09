FROM oven/bun:1.3.4-alpine AS deps
WORKDIR /app
COPY package.json bun.lock* ./
COPY apps/backend/package.json apps/backend/package.json
RUN bun install --frozen-lockfile

FROM oven/bun:1.3.4-alpine AS build
WORKDIR /app
COPY --from=deps /app/node_modules node_modules
COPY package.json tsconfig.base.json ./
COPY apps/backend apps/backend
RUN bun run --cwd apps/backend build

FROM oven/bun:1.3.4-alpine AS runner
WORKDIR /app
ENV NODE_ENV=production
COPY --from=build /app/apps/backend/dist dist
COPY --from=build /app/apps/backend/package.json package.json
COPY --from=deps /app/node_modules node_modules
CMD ["bun", "dist/index.js"]
