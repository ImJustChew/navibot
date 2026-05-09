import { z } from "zod";

export const drivePayloadSchema = z.object({
  linear: z.number().min(-1).max(1),
  angular: z.number().min(-1).max(1),
  durationMs: z.number().int().min(50).max(5000).default(250),
});

export const commandMessageSchema = z.object({
  kind: z.literal("command"),
  id: z.string(),
  type: z.enum(["drive", "stop", "dock", "map_start", "map_stop", "display", "alarm", "music"]),
  payload: z.record(z.unknown()).default({}),
  createdAt: z.string(),
});

export const telemetryMessageSchema = z.object({
  kind: z.literal("telemetry"),
  robotId: z.string(),
  state: z.record(z.unknown()),
  createdAt: z.string().optional(),
});

export const robotHelloSchema = z.object({
  kind: z.literal("hello"),
  robotId: z.string(),
  name: z.string().optional(),
  capabilities: z.array(z.string()).default([]),
});

export const mapSnapshotSchema = z.object({
  kind: z.literal("map_snapshot"),
  robotId: z.string(),
  label: z.string().default("latest"),
  resolutionMm: z.number().int().positive().default(50),
  payload: z.record(z.unknown()),
});

export const videoFrameSchema = z.object({
  kind: z.literal("video_frame"),
  robotId: z.string(),
  contentType: z.literal("image/jpeg").default("image/jpeg"),
  data: z.string(),
  width: z.number().int().positive(),
  height: z.number().int().positive(),
  sequence: z.number().int().nonnegative(),
  capturedAt: z.string(),
});

export type CommandMessage = z.infer<typeof commandMessageSchema>;
export type TelemetryMessage = z.infer<typeof telemetryMessageSchema>;
export type RobotHello = z.infer<typeof robotHelloSchema>;
export type MapSnapshotMessage = z.infer<typeof mapSnapshotSchema>;
export type VideoFrameMessage = z.infer<typeof videoFrameSchema>;

export type ClientEvent =
  | { kind: "robot_status"; robotId: string; online: boolean; at: string }
  | { kind: "telemetry"; robotId: string; state: Record<string, unknown>; at: string }
  | {
      kind: "video_frame";
      robotId: string;
      frame: Omit<VideoFrameMessage, "kind" | "robotId">;
      at: string;
    }
  | { kind: "command_ack"; robotId: string; commandId: string; at: string }
  | { kind: "error"; message: string };
