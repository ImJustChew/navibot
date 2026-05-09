import { boolean, index, integer, jsonb, pgTable, real, text, timestamp } from "drizzle-orm/pg-core";

export const robots = pgTable("robots", {
  id: text("id").primaryKey(),
  name: text("name").notNull(),
  online: boolean("online").notNull().default(false),
  mode: text("mode").notNull().default("idle"),
  batteryVoltage: real("battery_voltage"),
  batteryPercent: real("battery_percent"),
  charging: boolean("charging").notNull().default(false),
  lastSeenAt: timestamp("last_seen_at", { withTimezone: true }),
  createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
  updatedAt: timestamp("updated_at", { withTimezone: true }).notNull().defaultNow(),
});

export const telemetrySamples = pgTable(
  "telemetry_samples",
  {
    id: text("id").primaryKey(),
    robotId: text("robot_id")
      .notNull()
      .references(() => robots.id, { onDelete: "cascade" }),
    tRobotSeconds: real("t_robot_seconds"),
    batteryVoltage: real("battery_voltage"),
    currentMa: real("current_ma"),
    powerW: real("power_w"),
    pose: jsonb("pose").$type<Record<string, unknown>>(),
    tofMm: jsonb("tof_mm").$type<Record<string, number | null>>(),
    safety: jsonb("safety").$type<Record<string, unknown>>(),
    raw: jsonb("raw").$type<Record<string, unknown>>().notNull(),
    createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
  },
  (table) => ({
    robotCreatedAtIdx: index("telemetry_robot_created_at_idx").on(table.robotId, table.createdAt),
  }),
);

export const robotCommands = pgTable(
  "robot_commands",
  {
    id: text("id").primaryKey(),
    robotId: text("robot_id")
      .notNull()
      .references(() => robots.id, { onDelete: "cascade" }),
    type: text("type").notNull(),
    payload: jsonb("payload").$type<Record<string, unknown>>().notNull(),
    status: text("status").notNull().default("queued"),
    createdBy: text("created_by").notNull().default("web"),
    createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
    acknowledgedAt: timestamp("acknowledged_at", { withTimezone: true }),
  },
  (table) => ({
    robotStatusIdx: index("robot_commands_robot_status_idx").on(table.robotId, table.status),
  }),
);

export const mapSnapshots = pgTable(
  "map_snapshots",
  {
    id: text("id").primaryKey(),
    robotId: text("robot_id")
      .notNull()
      .references(() => robots.id, { onDelete: "cascade" }),
    label: text("label").notNull(),
    resolutionMm: integer("resolution_mm").notNull(),
    payload: jsonb("payload").$type<Record<string, unknown>>().notNull(),
    createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
  },
  (table) => ({
    robotMapCreatedAtIdx: index("map_robot_created_at_idx").on(table.robotId, table.createdAt),
  }),
);
