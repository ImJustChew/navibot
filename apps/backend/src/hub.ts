import type { WSContext } from "hono/ws";

import type { ClientEvent, CommandMessage } from "./protocol";

type ClientSocket = WSContext<WebSocket>;
type RobotSocket = WSContext<WebSocket>;

type RobotConnection = {
  socket: RobotSocket;
  name: string;
  connectedAt: string;
};

export class RobotHub {
  private robots = new Map<string, RobotConnection>();
  private clients = new Map<string, Set<ClientSocket>>();

  connectRobot(robotId: string, name: string, socket: RobotSocket): void {
    this.robots.set(robotId, { socket, name, connectedAt: new Date().toISOString() });
    this.broadcast(robotId, {
      kind: "robot_status",
      robotId,
      online: true,
      at: new Date().toISOString(),
    });
  }

  disconnectRobot(robotId: string, socket: RobotSocket): void {
    const current = this.robots.get(robotId);
    if (current?.socket !== socket) {
      return;
    }
    this.robots.delete(robotId);
    this.broadcast(robotId, {
      kind: "robot_status",
      robotId,
      online: false,
      at: new Date().toISOString(),
    });
  }

  connectClient(robotId: string, socket: ClientSocket): void {
    const sockets = this.clients.get(robotId) ?? new Set<ClientSocket>();
    sockets.add(socket);
    this.clients.set(robotId, sockets);
    socket.send(
      JSON.stringify({
        kind: "robot_status",
        robotId,
        online: this.robots.has(robotId),
        at: new Date().toISOString(),
      } satisfies ClientEvent),
    );
  }

  disconnectClient(robotId: string, socket: ClientSocket): void {
    const sockets = this.clients.get(robotId);
    sockets?.delete(socket);
    if (sockets?.size === 0) {
      this.clients.delete(robotId);
    }
  }

  sendCommand(robotId: string, command: CommandMessage): boolean {
    const robot = this.robots.get(robotId);
    if (!robot) {
      return false;
    }
    robot.socket.send(JSON.stringify(command));
    return true;
  }

  broadcast(robotId: string, event: ClientEvent): void {
    const payload = JSON.stringify(event);
    for (const socket of this.clients.get(robotId) ?? []) {
      socket.send(payload);
    }
  }

  listRobots(): Array<{ id: string; name: string; online: boolean; connectedAt: string }> {
    return Array.from(this.robots.entries()).map(([id, robot]) => ({
      id,
      name: robot.name,
      online: true,
      connectedAt: robot.connectedAt,
    }));
  }
}
