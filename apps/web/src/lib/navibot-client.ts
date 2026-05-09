import { config } from "./config";

export type TelemetryState = {
  t_s?: number;
  power?: {
    bus_voltage_v?: number;
    current_ma?: number;
    power_w?: number;
  };
  battery?: {
    is_charging?: boolean;
    level?: string;
  };
  tof_mm?: Record<string, number | null>;
  pose?: {
    x_mm?: number;
    y_mm?: number;
    theta_deg?: number;
  };
  safety?: Record<string, unknown>;
};

export type ClientEvent =
  | { kind: "robot_status"; robotId: string; online: boolean; at: string }
  | { kind: "telemetry"; robotId: string; state: TelemetryState; at: string }
  | { kind: "command_ack"; robotId: string; commandId: string; at: string }
  | { kind: "command_sent"; commandId: string; sent: boolean }
  | { kind: "error"; message: string };

export type DriveCommand = {
  linear: number;
  angular: number;
  durationMs?: number;
};

export function connectRobotEvents(
  onEvent: (event: ClientEvent) => void,
  onConnectionState: (state: "connecting" | "open" | "closed") => void,
) {
  const url = new URL(`/ws/client/${config.robotId}`, config.backendWsUrl);
  url.searchParams.set("token", config.sharedSecret);
  const socket = new WebSocket(url);

  onConnectionState("connecting");

  socket.addEventListener("open", () => onConnectionState("open"));
  socket.addEventListener("close", () => onConnectionState("closed"));
  socket.addEventListener("message", (event) => {
    onEvent(JSON.parse(event.data) as ClientEvent);
  });

  return {
    sendCommand(type: string, payload: Record<string, unknown> = {}) {
      if (socket.readyState !== WebSocket.OPEN) {
        return false;
      }
      socket.send(JSON.stringify({ type, payload }));
      return true;
    },
    close() {
      socket.close();
    },
  };
}

export async function sendRestCommand(type: string, payload: Record<string, unknown> = {}) {
  const response = await fetch(`${config.backendHttpUrl}/api/robots/${config.robotId}/commands`, {
    method: "POST",
    headers: {
      Authorization: `Bearer ${config.sharedSecret}`,
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ type, payload }),
  });
  if (!response.ok) {
    throw new Error(`command failed: ${response.status}`);
  }
  return response.json() as Promise<{ command: unknown }>;
}
