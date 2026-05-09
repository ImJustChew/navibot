import type { Context, Next } from "hono";

export function getSharedSecret(): string {
  return process.env.NAVIBOT_SHARED_SECRET ?? "change-me";
}

export function tokenFromRequest(c: Context): string | null {
  const header = c.req.header("authorization");
  if (header?.startsWith("Bearer ")) {
    return header.slice("Bearer ".length);
  }
  return c.req.query("token") ?? null;
}

export async function requireSharedSecret(c: Context, next: Next) {
  const expected = getSharedSecret();
  const provided = tokenFromRequest(c);
  if (!provided || provided !== expected) {
    return c.json({ error: "unauthorized" }, 401);
  }
  await next();
}
