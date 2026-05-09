import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";

import * as schema from "./schema";

export function createDb() {
  const databaseUrl = process.env.DATABASE_URL;
  if (!databaseUrl) {
    return null;
  }

  return drizzle(neon(databaseUrl), { schema });
}

export type Db = NonNullable<ReturnType<typeof createDb>>;
