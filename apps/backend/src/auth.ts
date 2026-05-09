import type { Context, Next } from "hono";

export function getRobotToken(): string {
  return process.env.NAVIBOT_ROBOT_TOKEN ?? process.env.NAVIBOT_SHARED_SECRET ?? "change-me";
}

export function getOperatorToken(): string {
  return process.env.NAVIBOT_OPERATOR_TOKEN ?? process.env.NAVIBOT_SHARED_SECRET ?? "change-me";
}

export function tokenFromRequest(c: Context): string | null {
  const header = c.req.header("authorization");
  if (header?.startsWith("Bearer ")) {
    return header.slice("Bearer ".length);
  }
  return c.req.query("token") ?? null;
}

export async function requireOperatorToken(c: Context, next: Next) {
  const expected = getOperatorToken();
  const provided = tokenFromRequest(c);
  if (!provided || provided !== expected) {
    return c.json({ error: "unauthorized" }, 401);
  }
  await next();
}
