import { createRoot } from "react-dom/client";
import {
  ArrowDown,
  ArrowUp,
  Camera,
  CircleStop,
  Dock,
  Map,
  RotateCcw,
  RotateCw,
} from "lucide-react";
import { useEffect, useRef, useState } from "react";

import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { cn } from "@/lib/utils";
import { config } from "./lib/config";
import { connectRobotEvents, type TelemetryState, type VideoFrame } from "./lib/navibot-client";
import "./styles.css";

// ─── Helpers ─────────────────────────────────────────────────────────────────

function fmt(v: number | undefined, digits = 1) {
  return typeof v === "number" && !Number.isNaN(v) ? v.toFixed(digits) : "—";
}

function tofColor(mm: number | null | undefined) {
  if (mm == null) return "#d1d5db";
  if (mm > 250) return "#3b82f6";
  if (mm > 100) return "#f59e0b";
  return "#ef4444";
}

function tofLen(mm: number | null | undefined, max = 190) {
  if (mm == null) return 0;
  return Math.max(10, Math.min(max, (mm / 500) * max));
}

// ─── Obstacle beam with physical wall ────────────────────────────────────────

const CX = 300, CY = 290, RW = 100, RH = 66;

const SENSORS = [
  { key: "front",   ox: CX,          oy: CY - RH / 2, dx:  0,      dy: -1,     ta: "middle" as const },
  { key: "back",    ox: CX,          oy: CY + RH / 2, dx:  0,      dy:  1,     ta: "middle" as const },
  { key: "left45",  ox: CX - RW / 2, oy: CY - RH / 2, dx: -0.707,  dy: -0.707, ta: "end"    as const },
  { key: "right45", ox: CX + RW / 2, oy: CY - RH / 2, dx:  0.707,  dy: -0.707, ta: "start"  as const },
] as const;

type Sensor = typeof SENSORS[number];

function ObstacleBeam({ s, mm }: { s: Sensor; mm: number | null }) {
  const maxLen = 190;

  if (mm == null) {
    // No data: faint max-range dotted ray
    return (
      <line
        x1={s.ox} y1={s.oy}
        x2={s.ox + s.dx * maxLen} y2={s.oy + s.dy * maxLen}
        stroke="#e5e7eb" strokeWidth="1.5" strokeDasharray="3 7" opacity="0.6"
      />
    );
  }

  const color  = tofColor(mm);
  const len    = tofLen(mm);
  const ex     = s.ox + s.dx * len;
  const ey     = s.oy + s.dy * len;

  // Perpendicular wall direction
  const perpDx = -s.dy as number;
  const perpDy =  s.dx as number;

  // Wall grows larger as obstacle gets closer
  const wallHalf  = mm < 100 ? 48 : mm < 250 ? 40 : 32;
  const wallStroke = mm < 100 ? 6  : mm < 250 ? 4.5 : 3;
  const zoneAlpha = mm < 100 ? 0.13 : mm < 250 ? 0.06 : 0.03;

  // Trapezoid zone from robot edge → wall
  const rHalf = 14;
  const zonePoints = [
    `${s.ox - perpDx * rHalf},${s.oy - perpDy * rHalf}`,
    `${s.ox + perpDx * rHalf},${s.oy + perpDy * rHalf}`,
    `${ex   + perpDx * wallHalf},${ey   + perpDy * wallHalf}`,
    `${ex   - perpDx * wallHalf},${ey   - perpDy * wallHalf}`,
  ].join(" ");

  // Label sits past the wall in the ray direction
  const lx = ex + s.dx * 20;
  const ly = ey + s.dy * 20;

  return (
    <g>
      {/* Zone fill */}
      <polygon points={zonePoints} fill={color} opacity={zoneAlpha} />

      {/* Dashed ray */}
      <line
        x1={s.ox} y1={s.oy} x2={ex} y2={ey}
        stroke={color} strokeWidth="1.5" strokeDasharray="5 5" opacity="0.55"
      />

      {/* Physical wall */}
      <line
        x1={ex - perpDx * wallHalf} y1={ey - perpDy * wallHalf}
        x2={ex + perpDx * wallHalf} y2={ey + perpDy * wallHalf}
        stroke={color} strokeWidth={wallStroke} strokeLinecap="round" opacity="0.95"
      />

      {/* Wall end dots */}
      <circle cx={ex - perpDx * wallHalf} cy={ey - perpDy * wallHalf}
        r={wallStroke / 2 + 1} fill={color} />
      <circle cx={ex + perpDx * wallHalf} cy={ey + perpDy * wallHalf}
        r={wallStroke / 2 + 1} fill={color} />

      {/* Distance label */}
      <text x={lx} y={ly - 6} textAnchor={s.ta} dominantBaseline="middle"
        fill={color} fontSize="12" fontFamily="Inter, system-ui, sans-serif" fontWeight="700">
        {mm}
      </text>
      <text x={lx} y={ly + 8} textAnchor={s.ta} dominantBaseline="middle"
        fill={color} fontSize="9" fontFamily="Inter, system-ui, sans-serif" opacity="0.65">
        mm
      </text>
    </g>
  );
}

// ─── Robot SVG view ───────────────────────────────────────────────────────────

function RobotView({ tofMm }: { tofMm: Record<string, number | null> }) {
  return (
    <svg viewBox="0 0 600 530" className="w-full h-full" preserveAspectRatio="xMidYMid meet">
      <defs>
        <pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse">
          <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#ebebef" strokeWidth="0.7" />
        </pattern>
      </defs>

      {/* Background */}
      <rect width="600" height="530" fill="#f5f5f7" />
      <rect width="600" height="530" fill="url(#grid)" />

      {/* Subtle range rings */}
      <circle cx={CX} cy={CY} r="100" fill="none" stroke="#e5e7eb" strokeWidth="1"
        strokeDasharray="3 7" />
      <circle cx={CX} cy={CY} r="190" fill="none" stroke="#e5e7eb" strokeWidth="0.5"
        strokeDasharray="2 7" />

      {/* Sensor beams with obstacle walls */}
      {SENSORS.map((s) => (
        <ObstacleBeam key={s.key} s={s} mm={tofMm[s.key] ?? null} />
      ))}

      {/* Wheels */}
      {([-RW / 2 - 8, RW / 2 + 2] as const).map((wx, wi) =>
        ([-RH / 2 + 5, RH / 2 - 21] as const).map((wy, hi) => (
          <rect key={`${wi}${hi}`}
            x={CX + wx} y={CY + wy} width="6" height="16" rx="2" fill="#374151" />
        )),
      )}

      {/* Robot body */}
      <rect x={CX - RW / 2} y={CY - RH / 2} width={RW} height={RH} rx="7" fill="#1f2937" />

      {/* Inner recess */}
      <rect x={CX - RW / 2 + 10} y={CY - RH / 2 + 12} width={RW - 20} height={RH - 24}
        rx="4" fill="#374151" />

      {/* Direction nose */}
      <polygon
        points={`${CX - 8},${CY - RH / 2} ${CX + 8},${CY - RH / 2} ${CX},${CY - RH / 2 - 13}`}
        fill="#3b82f6"
      />

      {/* Core dot */}
      <circle cx={CX} cy={CY} r="5" fill="#3b82f6" />

      {/* Robot label */}
      <text x={CX} y={CY + 6} textAnchor="middle" dominantBaseline="middle"
        fill="#4b5563" fontSize="7" fontFamily="Inter, system-ui, sans-serif"
        fontWeight="700" letterSpacing="2.5">
        NAVIBOT
      </text>

      {/* Compass */}
      {([
        { l: "N", x: 300, y: 16 },
        { l: "S", x: 300, y: 518 },
        { l: "W", x: 18,  y: 268 },
        { l: "E", x: 584, y: 268 },
      ] as const).map((p) => (
        <text key={p.l} x={p.x} y={p.y} textAnchor="middle" dominantBaseline="middle"
          fill="#d1d5db" fontSize="11" fontFamily="Inter, system-ui, sans-serif" fontWeight="600">
          {p.l}
        </text>
      ))}
    </svg>
  );
}

// ─── Mini metric ──────────────────────────────────────────────────────────────

function MiniMetric({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div>
      <div className="text-[9px] uppercase tracking-[0.15em] text-[#9ca3af] mb-1">{label}</div>
      <div className="flex items-baseline gap-1">
        <span className="text-2xl font-light tabular-nums text-[#111827]">{value}</span>
        {unit && <span className="text-xs text-[#9ca3af]">{unit}</span>}
      </div>
    </div>
  );
}

function CameraView({ frame, online }: { frame: VideoFrame | null; online: boolean }) {
  return (
    <Card className="h-[260px] rounded-2xl border-[#e8e8ed] overflow-hidden">
      <CardHeader className="h-11 flex-row items-center justify-between space-y-0 border-b border-[#e8e8ed]">
        <CardTitle className="flex items-center gap-2">
          <Camera size={14} className="text-[#9ca3af]" />
          Camera
        </CardTitle>
        <Badge variant={frame ? "online" : "offline"} className="text-[9px]">
          {frame ? `${frame.width}x${frame.height}` : online ? "Waiting" : "Offline"}
        </Badge>
      </CardHeader>
      <CardContent className="h-[calc(100%-44px)] p-0">
        {frame ? (
          <div className="relative h-full bg-[#111827]">
            <img
              src={`data:${frame.contentType};base64,${frame.data}`}
              alt="Robot camera stream"
              className="h-full w-full object-contain"
              draggable={false}
            />
            <div className="absolute bottom-2 left-2 rounded-full bg-black/60 px-2 py-1 text-[10px] font-mono text-white/80">
              #{frame.sequence} · {new Date(frame.capturedAt).toLocaleTimeString()}
            </div>
          </div>
        ) : (
          <div className="flex h-full flex-col items-center justify-center gap-2 bg-[#f5f5f7] text-center">
            <Camera size={28} className="text-[#d1d5db]" />
            <div className="text-xs font-medium text-[#6b7280]">
              {online ? "Waiting for camera frames" : "Robot camera offline"}
            </div>
            <div className="max-w-[220px] text-[10px] leading-4 text-[#9ca3af]">
              The Pi publishes encrypted JPEG frames over the robot websocket.
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  );
}

// ─── App ──────────────────────────────────────────────────────────────────────

function App() {
  const [operatorToken, setOperatorToken] = useState(
    () => sessionStorage.getItem("navibot_operator_token") ?? "",
  );
  const [tokenDraft, setTokenDraft] = useState(operatorToken);
  const [connection, setConnection] = useState<"connecting" | "open" | "closed">("closed");
  const [online, setOnline] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryState>({});
  const [videoFrame, setVideoFrame] = useState<VideoFrame | null>(null);
  const [driveSpeed, setDriveSpeed] = useState(0.6);
  const [events, setEvents] = useState<string[]>([]);
  const [sendCommand, setSendCommand] = useState<
    (type: string, payload?: Record<string, unknown>) => boolean
  >(() => () => false);
  const sendCommandRef = useRef(sendCommand);
  const pressedKeysRef = useRef(new Set<string>());
  const driveSpeedRef = useRef(driveSpeed);

  useEffect(() => { sendCommandRef.current = sendCommand; }, [sendCommand]);
  useEffect(() => { driveSpeedRef.current = driveSpeed; }, [driveSpeed]);

  useEffect(() => {
    if (!operatorToken) {
      setConnection("closed");
      setOnline(false);
      setVideoFrame(null);
      setSendCommand(() => () => false);
      return;
    }
    sessionStorage.setItem("navibot_operator_token", operatorToken);
    const client = connectRobotEvents(
      operatorToken,
      (event) => {
        setEvents((cur) =>
          [`${new Date().toLocaleTimeString()} ${event.kind}`, ...cur].slice(0, 100),
        );
        if (event.kind === "robot_status") setOnline(event.online);
        if (event.kind === "telemetry") { setTelemetry(event.state); setOnline(true); }
        if (event.kind === "video_frame") { setVideoFrame(event.frame); setOnline(true); }
      },
      setConnection,
    );
    setSendCommand(() => client.sendCommand);
    return () => client.close();
  }, [operatorToken]);

  useEffect(() => {
    const movKeys = new Set(["w", "a", "s", "d", "arrowup", "arrowleft", "arrowdown", "arrowright"]);

    function getVec() {
      const k = pressedKeysRef.current;
      return {
        linear:  (k.has("w") || k.has("arrowup")    ? 1 : 0) + (k.has("s") || k.has("arrowdown")  ? -1 : 0),
        angular: (k.has("a") || k.has("arrowleft")  ? 1 : 0) + (k.has("d") || k.has("arrowright") ? -1 : 0),
      };
    }

    function sendDrive() {
      const { linear, angular } = getVec();
      if (linear === 0 && angular === 0) return;
      sendCommandRef.current("drive", {
        linear: linear * driveSpeedRef.current,
        angular: angular * driveSpeedRef.current,
        durationMs: 350,
      });
    }

    function onDown(e: KeyboardEvent) {
      const t = e.target as HTMLElement | null;
      if (t?.tagName === "INPUT" || t?.tagName === "TEXTAREA" || t?.isContentEditable) return;
      const key = e.key.toLowerCase();
      if (!movKeys.has(key)) return;
      e.preventDefault();
      pressedKeysRef.current.add(key);
      sendDrive();
    }

    function onUp(e: KeyboardEvent) {
      const key = e.key.toLowerCase();
      if (!movKeys.has(key)) return;
      e.preventDefault();
      pressedKeysRef.current.delete(key);
      const { linear, angular } = getVec();
      if (linear === 0 && angular === 0) sendCommandRef.current("stop");
      else sendDrive();
    }

    function onBlur() {
      if (pressedKeysRef.current.size > 0) {
        pressedKeysRef.current.clear();
        sendCommandRef.current("stop");
      }
    }

    const interval = window.setInterval(sendDrive, 150);
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    window.addEventListener("blur", onBlur);
    return () => {
      window.clearInterval(interval);
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
      window.removeEventListener("blur", onBlur);
    };
  }, []);

  function drive(linear: number, angular: number) {
    sendCommand("drive", {
      linear: linear * driveSpeed,
      angular: angular * driveSpeed,
      durationMs: 350,
    });
  }

  const tofMm = telemetry.tof_mm ?? {};

  const isConnected = connection === "open";

  return (
    <div
      className="h-screen bg-[#f5f5f7] text-[#111827] flex flex-col overflow-hidden"
      style={{ fontFamily: "Inter, ui-sans-serif, system-ui, -apple-system, sans-serif" }}
    >
      {/* ── Header ────────────────────────────────────────────────────────── */}
      <header className="h-11 bg-white border-b border-[#e8e8ed] flex items-center px-5 gap-3 shrink-0">
        <span className="font-bold text-[11px] tracking-[0.28em] uppercase text-[#111827]">NAVIBOT</span>
        <div className="w-px h-3.5 bg-[#e8e8ed]" />
        <span className="text-[11px] text-[#9ca3af]">Remote Operations</span>
        <div className="ml-auto flex items-center gap-3">
          <Badge variant={online ? "online" : "offline"}>
            <span className={cn("w-1.5 h-1.5 rounded-full", online ? "bg-green-500 animate-pulse" : "bg-[#d1d5db]")} />
            {online ? "Online" : "Offline"}
          </Badge>
          <span className="text-[11px] text-[#d1d5db] capitalize">{connection}</span>
        </div>
      </header>

      {/* ── Body ──────────────────────────────────────────────────────────── */}
      <div className="flex-1 flex min-h-0">

        {/* ── Left sidebar: controls ──────────────────────────────────────── */}
        <aside className="w-[272px] shrink-0 bg-white border-r border-[#e8e8ed] flex flex-col overflow-y-auto">

          {/* Drive */}
          <Card className="m-4 mb-0 rounded-2xl border-[#e8e8ed]">
            <CardHeader><CardTitle>Drive Control</CardTitle></CardHeader>
            <CardContent className="pt-0">
              <div className="flex flex-col items-center gap-2.5">
                <Button variant="outline" size="icon" onClick={() => drive(1, 0)}>
                  <ArrowUp size={20} />
                </Button>
                <div className="flex items-center gap-2.5">
                  <Button variant="outline" size="icon" onClick={() => drive(0, 1)}>
                    <RotateCcw size={18} />
                  </Button>
                  <Button variant="destructive" size="icon" onClick={() => sendCommand("stop")}>
                    <CircleStop size={20} />
                  </Button>
                  <Button variant="outline" size="icon" onClick={() => drive(0, -1)}>
                    <RotateCw size={18} />
                  </Button>
                </div>
                <Button variant="outline" size="icon" onClick={() => drive(-1, 0)}>
                  <ArrowDown size={20} />
                </Button>
              </div>
              <p className="text-center text-[9px] uppercase tracking-widest text-[#d1d5db] mt-3">
                WASD / Arrow Keys
              </p>
              <div className="mt-5 rounded-xl border border-[#e8e8ed] bg-[#f9fafb] p-3">
                <div className="mb-2 flex items-center justify-between">
                  <span className="text-[9px] uppercase tracking-[0.15em] text-[#9ca3af]">
                    Speed
                  </span>
                  <span className="font-mono text-xs text-[#4b5563]">
                    {Math.round(driveSpeed * 100)}%
                  </span>
                </div>
                <input
                  type="range"
                  min="20"
                  max="100"
                  step="5"
                  value={Math.round(driveSpeed * 100)}
                  onChange={(event) => setDriveSpeed(Number(event.currentTarget.value) / 100)}
                  className="h-2 w-full accent-blue-600"
                  aria-label="Drive speed"
                />
                <div className="mt-2 flex justify-between font-mono text-[9px] text-[#d1d5db]">
                  <span>20</span>
                  <span>60</span>
                  <span>100</span>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Tasks */}
          <Card className="m-4 mb-0 rounded-2xl border-[#e8e8ed]">
            <CardHeader><CardTitle>Tasks</CardTitle></CardHeader>
            <CardContent className="pt-0 flex flex-col gap-2">
              <Button variant="secondary" className="w-full justify-start gap-2 text-xs"
                onClick={() => sendCommand("dock")}>
                <Dock size={14} className="text-[#9ca3af]" /> Start Docking
              </Button>
              <Button variant="secondary" className="w-full justify-start gap-2 text-xs"
                onClick={() => sendCommand("map_start")}>
                <Map size={14} className="text-[#9ca3af]" /> Start Mapping
              </Button>
              <Button variant="secondary" className="w-full justify-start gap-2 text-xs"
                onClick={() => sendCommand("map_stop")}>
                <Map size={14} className="text-[#9ca3af]" /> Stop Mapping
              </Button>
              <Button variant="secondary" className="w-full justify-start text-xs"
                onClick={() => sendCommand("display", { mode: "status" })}>
                Display Status
              </Button>
            </CardContent>
          </Card>

          {/* Connection / token */}
          <Card className="m-4 mb-0 rounded-2xl border-[#e8e8ed]">
            <CardHeader><CardTitle>Connection</CardTitle></CardHeader>
            <CardContent className="pt-0 flex flex-col gap-3">
              <div className="flex items-center justify-between text-xs">
                <span className="text-[#9ca3af]">{config.robotId}</span>
                <Badge variant={isConnected ? "online" : "offline"} className="text-[9px]">
                  {connection}
                </Badge>
              </div>
              <form
                className="flex flex-col gap-2"
                onSubmit={(e) => {
                  e.preventDefault();
                  setOperatorToken(tokenDraft.trim());
                }}
              >
                <Input
                  type="password"
                  placeholder="Operator token"
                  autoComplete="current-password"
                  value={tokenDraft}
                  onChange={(e) => setTokenDraft(e.target.value)}
                  className="font-mono text-xs h-8"
                />
                <Button type="submit" size="sm" className="w-full text-xs">
                  {operatorToken ? "Reconnect" : "Connect"}
                </Button>
              </form>
              {operatorToken && (
                <button
                  className="text-[10px] text-[#9ca3af] hover:text-[#6b7280] text-center transition-colors"
                  onClick={() => { setOperatorToken(""); setTokenDraft(""); }}
                >
                  Disconnect
                </button>
              )}
            </CardContent>
          </Card>

          {/* Telemetry */}
          <Card className="m-4 rounded-2xl border-[#e8e8ed]">
            <CardHeader><CardTitle>Telemetry</CardTitle></CardHeader>
            <CardContent className="pt-0 grid grid-cols-2 gap-x-4 gap-y-4">
              <MiniMetric label="Voltage" value={fmt(telemetry.power?.bus_voltage_v, 2)} unit="V" />
              <MiniMetric label="Current" value={fmt(telemetry.power?.current_ma, 0)} unit="mA" />
              <MiniMetric label="Power" value={fmt(telemetry.power?.power_w, 2)} unit="W" />
              <MiniMetric label="Heading" value={fmt(telemetry.pose?.theta_deg, 0)} unit="°" />
            </CardContent>
          </Card>
        </aside>

        {/* ── Right: camera and robot visualization ─────────────────────── */}
        <main className="flex-1 flex flex-col min-w-0 overflow-hidden">
          <div className="flex-1 grid grid-cols-[minmax(0,1fr)_360px] gap-4 p-6 overflow-hidden">
            <div className="min-h-0 flex items-center justify-center overflow-hidden">
              <RobotView tofMm={tofMm} />
            </div>
            <div className="min-h-0 flex flex-col gap-4">
              <CameraView frame={videoFrame} online={online} />
              <Card className="min-h-0 flex-1 rounded-2xl border-[#e8e8ed]">
                <CardHeader><CardTitle>Backend Log</CardTitle></CardHeader>
                <CardContent className="pt-0">
                  <div className="max-h-[calc(100vh-420px)] overflow-y-auto pr-2 font-mono text-[10px] leading-5 text-[#9ca3af]">
                    {events.map((event, index) => (
                      <div key={`${event}-${index}`} className={cn(index === 0 && "text-[#4b5563]")}>
                        {event}
                      </div>
                    ))}
                  </div>
                </CardContent>
              </Card>
            </div>
          </div>

          {/* Sensor legend */}
          <div className="shrink-0 border-t border-[#e8e8ed] bg-white px-6 py-3 flex items-center gap-6">
            <span className="text-[9px] uppercase tracking-[0.2em] text-[#d1d5db]">Proximity</span>
            {[
              { color: "#3b82f6", label: "> 250 mm  Clear" },
              { color: "#f59e0b", label: "100–250 mm  Caution" },
              { color: "#ef4444", label: "< 100 mm  Danger" },
            ].map((item) => (
              <div key={item.label} className="flex items-center gap-2">
                <div className="w-3 h-1 rounded-full" style={{ background: item.color }} />
                <span className="text-[10px] text-[#9ca3af]">{item.label}</span>
              </div>
            ))}
            <div className="ml-auto flex gap-4 text-[10px] font-mono text-[#d1d5db] overflow-hidden">
              {videoFrame ? (
                <span className="shrink-0 text-[#9ca3af]">camera #{videoFrame.sequence}</span>
              ) : null}
              {events.slice(0, 3).map((e, i) => (
                <span key={i} className={cn("shrink-0", i === 0 && "text-[#9ca3af]")}>{e}</span>
              ))}
            </div>
          </div>
        </main>
      </div>
    </div>
  );
}

createRoot(document.getElementById("root")!).render(<App />);
