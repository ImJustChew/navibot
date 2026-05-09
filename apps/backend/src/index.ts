import { serve } from "@hono/node-server";
import { createNodeWebSocket } from "@hono/node-ws";
import { desc, eq } from "drizzle-orm";
import { Hono } from "hono";
import { cors } from "hono/cors";
import { nanoid } from "nanoid";

import { requireOperatorToken, tokenFromRequest, getOperatorToken, getRobotToken } from "./auth";
import { createDb } from "./db/client";
import { mapSnapshots, robotCommands, robots, telemetrySamples } from "./db/schema";
import { RobotHub } from "./hub";
import {
  commandMessageSchema,
  drivePayloadSchema,
  mapSnapshotSchema,
  robotHelloSchema,
  telemetryMessageSchema,
  type CommandMessage,
} from "./protocol";

const app = new Hono();
const { injectWebSocket, upgradeWebSocket } = createNodeWebSocket({ app });
const hub = new RobotHub();
const db = createDb();

function requireRobotId(value: string | undefined): string {
  if (!value) {
    throw new Error("missing robot id");
  }
  return value;
}

async function ensureRobot(robotId: string, name = robotId) {
  if (!db) {
    return;
  }
  await db
    .insert(robots)
    .values({
      id: robotId,
      name,
      online: false,
      updatedAt: new Date(),
    })
    .onConflictDoNothing();
}

async function markRobotOnline(robotId: string, name = robotId) {
  if (!db) {
    return;
  }
  await db
    .insert(robots)
    .values({
      id: robotId,
      name,
      online: true,
      lastSeenAt: new Date(),
      updatedAt: new Date(),
    })
    .onConflictDoUpdate({
      target: robots.id,
      set: { name, online: true, lastSeenAt: new Date(), updatedAt: new Date() },
    });
}

async function flushQueuedCommands(robotId: string) {
  if (!db) {
    return;
  }
  const queued = await db
    .select()
    .from(robotCommands)
    .where(eq(robotCommands.robotId, robotId))
    .orderBy(desc(robotCommands.createdAt))
    .limit(20);

  for (const row of queued.reverse()) {
    if (row.status !== "queued") {
      continue;
    }
    const command = commandMessageSchema.parse({
      kind: "command",
      id: row.id,
      type: row.type,
      payload: row.payload,
      createdAt: row.createdAt.toISOString(),
    });
    if (hub.sendCommand(robotId, command)) {
      await db
        .update(robotCommands)
        .set({ status: "sent" })
        .where(eq(robotCommands.id, row.id));
    }
  }
}

app.use(
  "*",
  cors({
    origin: process.env.NAVIBOT_WEB_ORIGIN ?? "*",
    allowHeaders: ["Authorization", "Content-Type", "X-Navibot-Token"],
    allowMethods: ["GET", "POST", "OPTIONS"],
  }),
);

app.get("/health", (c) =>
  c.json({
    status: "ok",
    service: "navibot-backend",
    db: db ? "configured" : "disabled",
  }),
);

app.get("/api/robots", requireOperatorToken, async (c) => {
  if (!db) {
    return c.json({ robots: hub.listRobots() });
  }
  const rows = await db.select().from(robots).orderBy(desc(robots.updatedAt));
  return c.json({ robots: rows });
});

app.get("/api/robots/:robotId/status", requireOperatorToken, async (c) => {
  const robotId = requireRobotId(c.req.param("robotId"));
  if (!db) {
    return c.json({ robot: hub.listRobots().find((robot) => robot.id === robotId) ?? null });
  }
  const [robot] = await db.select().from(robots).where(eq(robots.id, robotId)).limit(1);
  const latestTelemetry = await db
    .select()
    .from(telemetrySamples)
    .where(eq(telemetrySamples.robotId, robotId))
    .orderBy(desc(telemetrySamples.createdAt))
    .limit(1);
  return c.json({ robot: robot ?? null, latestTelemetry: latestTelemetry[0] ?? null });
});

app.post("/api/robots/:robotId/commands", requireOperatorToken, async (c) => {
  const robotId = requireRobotId(c.req.param("robotId"));
  const body = await c.req.json();
  const parsedPayload =
    body.type === "drive" ? drivePayloadSchema.parse(body.payload ?? {}) : (body.payload ?? {});
  const command: CommandMessage = commandMessageSchema.parse({
    kind: "command",
    id: body.id ?? nanoid(),
    type: body.type,
    payload: parsedPayload,
    createdAt: new Date().toISOString(),
  });

  if (db) {
    await ensureRobot(robotId);
    await db.insert(robotCommands).values({
      id: command.id,
      robotId,
      type: command.type,
      payload: command.payload,
      status: hub.sendCommand(robotId, command) ? "sent" : "queued",
      createdBy: "api",
    });
  } else {
    hub.sendCommand(robotId, command);
  }

  return c.json({ command });
});

app.get(
  "/ws/robot/:robotId",
  upgradeWebSocket((c) => {
    const robotId = requireRobotId(c.req.param("robotId"));
    const token = tokenFromRequest(c);
    if (token !== getRobotToken()) {
      return {
        onOpen(_event, ws) {
          ws.close(1008, "unauthorized");
        },
      };
    }

    return {
      async onOpen(_event, ws) {
        hub.connectRobot(robotId, robotId, ws);
        await markRobotOnline(robotId);
        await flushQueuedCommands(robotId);
      },
      async onMessage(event, ws) {
        try {
          const raw = typeof event.data === "string" ? event.data : event.data.toString();
          const message = JSON.parse(raw);

          if (message.kind === "hello") {
          const hello = robotHelloSchema.parse(message);
          hub.connectRobot(robotId, hello.name ?? robotId, ws);
          await markRobotOnline(robotId, hello.name ?? robotId);
          await flushQueuedCommands(robotId);
          return;
          }

          if (message.kind === "telemetry") {
          const telemetry = telemetryMessageSchema.parse(message);
          const state = telemetry.state;
          const power = (state.power ?? {}) as Record<string, unknown>;
          const battery = (state.battery ?? {}) as Record<string, unknown>;
          const pose = (state.pose ?? {}) as Record<string, unknown>;
          const safety = (state.safety ?? {}) as Record<string, unknown>;

          if (db) {
            await db
              .insert(robots)
              .values({
                id: robotId,
                name: robotId,
                online: true,
                batteryVoltage:
                  typeof power.bus_voltage_v === "number" ? power.bus_voltage_v : null,
                batteryPercent: typeof battery.percent === "number" ? battery.percent : null,
                charging: Boolean(battery.is_charging),
                lastSeenAt: new Date(),
                updatedAt: new Date(),
              })
              .onConflictDoUpdate({
                target: robots.id,
                set: {
                  online: true,
                  batteryVoltage:
                    typeof power.bus_voltage_v === "number" ? power.bus_voltage_v : null,
                  batteryPercent: typeof battery.percent === "number" ? battery.percent : null,
                  charging: Boolean(battery.is_charging),
                  lastSeenAt: new Date(),
                  updatedAt: new Date(),
                },
              });
            await db.insert(telemetrySamples).values({
              id: nanoid(),
              robotId,
              tRobotSeconds: typeof state.t_s === "number" ? state.t_s : null,
              batteryVoltage:
                typeof power.bus_voltage_v === "number" ? power.bus_voltage_v : null,
              currentMa: typeof power.current_ma === "number" ? power.current_ma : null,
              powerW: typeof power.power_w === "number" ? power.power_w : null,
              pose,
              tofMm: (state.tof_mm ?? {}) as Record<string, number | null>,
              safety,
              raw: state,
            });
          }

          hub.broadcast(robotId, {
            kind: "telemetry",
            robotId,
            state,
            at: telemetry.createdAt ?? new Date().toISOString(),
          });
          console.log(`telemetry stored for robot ${robotId}`);
          return;
          }

          if (message.kind === "command_ack") {
          const commandId = String(message.commandId);
          if (db) {
            await db
              .update(robotCommands)
              .set({ status: "acknowledged", acknowledgedAt: new Date() })
              .where(eq(robotCommands.id, commandId));
          }
          hub.broadcast(robotId, {
            kind: "command_ack",
            robotId,
            commandId,
            at: new Date().toISOString(),
          });
          return;
          }

          if (message.kind === "map_snapshot") {
          const snapshot = mapSnapshotSchema.parse(message);
          if (db) {
            await db.insert(mapSnapshots).values({
              id: nanoid(),
              robotId,
              label: snapshot.label,
              resolutionMm: snapshot.resolutionMm,
              payload: snapshot.payload,
            });
          }
          }
        } catch (error) {
          console.error("robot websocket message failed", error);
          ws.send(JSON.stringify({ kind: "error", message: "robot message rejected" }));
        }
      },
      async onClose(_event, ws) {
        hub.disconnectRobot(robotId, ws);
        if (db) {
          await db
            .update(robots)
            .set({ online: false, updatedAt: new Date() })
            .where(eq(robots.id, robotId));
        }
      },
      onError() {
        hub.disconnectRobot(robotId, {} as never);
      },
    };
  }),
);

app.get(
  "/ws/client/:robotId",
  upgradeWebSocket((c) => {
    const robotId = requireRobotId(c.req.param("robotId"));
    const token = tokenFromRequest(c);
    if (token !== getOperatorToken()) {
      return {
        onOpen(_event, ws) {
          ws.close(1008, "unauthorized");
        },
      };
    }

    return {
      onOpen(_event, ws) {
        hub.connectClient(robotId, ws);
      },
      async onMessage(event, ws) {
        const raw = typeof event.data === "string" ? event.data : event.data.toString();
        const body = JSON.parse(raw);
        const payload = body.type === "drive" ? drivePayloadSchema.parse(body.payload ?? {}) : body.payload ?? {};
        const command = commandMessageSchema.parse({
          kind: "command",
          id: body.id ?? nanoid(),
          type: body.type,
          payload,
          createdAt: new Date().toISOString(),
        });
        const sent = hub.sendCommand(robotId, command);
        if (db) {
          await ensureRobot(robotId);
          await db.insert(robotCommands).values({
            id: command.id,
            robotId,
            type: command.type,
            payload: command.payload,
            status: sent ? "sent" : "queued",
            createdBy: "web",
          });
        }
        ws.send(JSON.stringify({ kind: "command_sent", commandId: command.id, sent }));
      },
      onClose(_event, ws) {
        hub.disconnectClient(robotId, ws);
      },
    };
  }),
);

const port = Number(process.env.PORT ?? process.env.NAVIBOT_PORT ?? 8787);
const server = serve({ fetch: app.fetch, port });
injectWebSocket(server);

console.log(`Navibot Hono backend listening on :${port}`);
