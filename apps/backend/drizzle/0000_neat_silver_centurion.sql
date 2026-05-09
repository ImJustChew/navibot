CREATE TABLE "map_snapshots" (
	"id" text PRIMARY KEY NOT NULL,
	"robot_id" text NOT NULL,
	"label" text NOT NULL,
	"resolution_mm" integer NOT NULL,
	"payload" jsonb NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "robot_commands" (
	"id" text PRIMARY KEY NOT NULL,
	"robot_id" text NOT NULL,
	"type" text NOT NULL,
	"payload" jsonb NOT NULL,
	"status" text DEFAULT 'queued' NOT NULL,
	"created_by" text DEFAULT 'web' NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"acknowledged_at" timestamp with time zone
);
--> statement-breakpoint
CREATE TABLE "robots" (
	"id" text PRIMARY KEY NOT NULL,
	"name" text NOT NULL,
	"online" boolean DEFAULT false NOT NULL,
	"mode" text DEFAULT 'idle' NOT NULL,
	"battery_voltage" real,
	"battery_percent" real,
	"charging" boolean DEFAULT false NOT NULL,
	"last_seen_at" timestamp with time zone,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL,
	"updated_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "telemetry_samples" (
	"id" text PRIMARY KEY NOT NULL,
	"robot_id" text NOT NULL,
	"t_robot_seconds" real,
	"battery_voltage" real,
	"current_ma" real,
	"power_w" real,
	"pose" jsonb,
	"tof_mm" jsonb,
	"safety" jsonb,
	"raw" jsonb NOT NULL,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
ALTER TABLE "map_snapshots" ADD CONSTRAINT "map_snapshots_robot_id_robots_id_fk" FOREIGN KEY ("robot_id") REFERENCES "public"."robots"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "robot_commands" ADD CONSTRAINT "robot_commands_robot_id_robots_id_fk" FOREIGN KEY ("robot_id") REFERENCES "public"."robots"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "telemetry_samples" ADD CONSTRAINT "telemetry_samples_robot_id_robots_id_fk" FOREIGN KEY ("robot_id") REFERENCES "public"."robots"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
CREATE INDEX "map_robot_created_at_idx" ON "map_snapshots" USING btree ("robot_id","created_at");--> statement-breakpoint
CREATE INDEX "robot_commands_robot_status_idx" ON "robot_commands" USING btree ("robot_id","status");--> statement-breakpoint
CREATE INDEX "telemetry_robot_created_at_idx" ON "telemetry_samples" USING btree ("robot_id","created_at");