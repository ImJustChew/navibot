import { createRoot } from "react-dom/client";
import { Activity, BatteryCharging, CircleStop, Dock, Gauge, Map, RotateCcw, RotateCw } from "lucide-react";
import type { CSSProperties } from "react";
import { useEffect, useRef, useState } from "react";

import { config } from "./lib/config";
import { connectRobotEvents, type TelemetryState } from "./lib/navibot-client";
import "./styles.css";

function formatNumber(value: number | undefined, suffix = "", digits = 1) {
  if (typeof value !== "number" || Number.isNaN(value)) {
    return "n/a";
  }
  return `${value.toFixed(digits)}${suffix}`;
}

function App() {
  const [operatorToken, setOperatorToken] = useState(() => sessionStorage.getItem("navibot_operator_token") ?? "");
  const [connection, setConnection] = useState<"connecting" | "open" | "closed">("closed");
  const [online, setOnline] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryState>({});
  const [events, setEvents] = useState<string[]>([]);
  const [sendCommand, setSendCommand] = useState<(type: string, payload?: Record<string, unknown>) => boolean>(() => () => false);
  const sendCommandRef = useRef(sendCommand);
  const pressedKeysRef = useRef(new Set<string>());

  useEffect(() => {
    sendCommandRef.current = sendCommand;
  }, [sendCommand]);

  useEffect(() => {
    if (!operatorToken) {
      setConnection("closed");
      setOnline(false);
      setSendCommand(() => () => false);
      return;
    }
    sessionStorage.setItem("navibot_operator_token", operatorToken);
    const client = connectRobotEvents(
      operatorToken,
      (event) => {
        setEvents((current) => [`${new Date().toLocaleTimeString()} ${event.kind}`, ...current].slice(0, 100));
        if (event.kind === "robot_status") {
          setOnline(event.online);
        }
        if (event.kind === "telemetry") {
          setTelemetry(event.state);
          setOnline(true);
        }
      },
      setConnection,
    );
    setSendCommand(() => client.sendCommand);
    return () => client.close();
  }, [operatorToken]);

  useEffect(() => {
    const movementKeys = new Set(["w", "a", "s", "d", "arrowup", "arrowleft", "arrowdown", "arrowright"]);

    function getDriveVector() {
      const keys = pressedKeysRef.current;
      const forward = keys.has("w") || keys.has("arrowup");
      const reverse = keys.has("s") || keys.has("arrowdown");
      const left = keys.has("a") || keys.has("arrowleft");
      const right = keys.has("d") || keys.has("arrowright");
      return {
        linear: (forward ? 1 : 0) + (reverse ? -1 : 0),
        angular: (left ? 1 : 0) + (right ? -1 : 0),
      };
    }

    function sendDriveFromKeys() {
      const { linear, angular } = getDriveVector();
      if (linear === 0 && angular === 0) {
        return;
      }
      sendCommandRef.current("drive", { linear, angular, durationMs: 350 });
    }

    function onKeyDown(event: KeyboardEvent) {
      const target = event.target as HTMLElement | null;
      if (target?.tagName === "INPUT" || target?.tagName === "TEXTAREA" || target?.isContentEditable) {
        return;
      }
      const key = event.key.toLowerCase();
      if (!movementKeys.has(key)) {
        return;
      }
      event.preventDefault();
      pressedKeysRef.current.add(key);
      sendDriveFromKeys();
    }

    function onKeyUp(event: KeyboardEvent) {
      const key = event.key.toLowerCase();
      if (!movementKeys.has(key)) {
        return;
      }
      event.preventDefault();
      pressedKeysRef.current.delete(key);
      const { linear, angular } = getDriveVector();
      if (linear === 0 && angular === 0) {
        sendCommandRef.current("stop");
      } else {
        sendDriveFromKeys();
      }
    }

    function onBlur() {
      if (pressedKeysRef.current.size > 0) {
        pressedKeysRef.current.clear();
        sendCommandRef.current("stop");
      }
    }

    const interval = window.setInterval(sendDriveFromKeys, 150);
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);
    return () => {
      window.clearInterval(interval);
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
    };
  }, []);

  function drive(linear: number, angular: number) {
    sendCommand("drive", { linear, angular, durationMs: 350 });
  }

  return (
    <main className="app">
      <header className="topbar">
        <div>
          <p className="eyebrow">Remote Operations</p>
          <h1>Navibot</h1>
        </div>
        <div className={`status ${online ? "online" : "offline"}`}>
          <span />
          {online ? "Robot online" : "Robot offline"} · {connection}
        </div>
      </header>

      {!operatorToken ? (
        <section className="login panel">
          <div className="panel-title">Operator Access</div>
          <form
            onSubmit={(event) => {
              event.preventDefault();
              const form = new FormData(event.currentTarget);
              setOperatorToken(String(form.get("token") ?? "").trim());
            }}
          >
            <input name="token" type="password" placeholder="Operator token" autoComplete="current-password" />
            <button type="submit">Connect</button>
          </form>
        </section>
      ) : null}

      <section className="grid">
        <section className="panel telemetry">
          <div className="panel-title">
            <Activity size={18} />
            Live Telemetry
          </div>
          <div className="metrics">
            <Metric label="Bus Voltage" value={formatNumber(telemetry.power?.bus_voltage_v, "V", 2)} />
            <Metric label="Current" value={formatNumber(telemetry.power?.current_ma, "mA", 0)} />
            <Metric label="Power" value={formatNumber(telemetry.power?.power_w, "W", 2)} />
            <Metric label="Heading" value={formatNumber(telemetry.pose?.theta_deg, "deg", 0)} />
          </div>
          <RobotFootprint tofMm={telemetry.tof_mm ?? {}} />
        </section>

        <section className="panel drive">
          <div className="panel-title">
            <Gauge size={18} />
            Drive
          </div>
          <div className="pad">
            <button onClick={() => drive(1, 0)}>Forward</button>
            <button onClick={() => drive(0, 1)}><RotateCcw size={18} />Left</button>
            <button className="stop" onClick={() => sendCommand("stop")}><CircleStop size={20} />Stop</button>
            <button onClick={() => drive(0, -1)}><RotateCw size={18} />Right</button>
            <button onClick={() => drive(-1, 0)}>Reverse</button>
          </div>
        </section>

        <section className="panel actions">
          <div className="panel-title">
            <Dock size={18} />
            Tasks
          </div>
          <button onClick={() => sendCommand("dock")}>Start Docking</button>
          <button onClick={() => sendCommand("map_start")}><Map size={18} />Start Mapping</button>
          <button onClick={() => sendCommand("map_stop")}>Stop Mapping</button>
          <button onClick={() => sendCommand("display", { mode: "status" })}>Display Status</button>
        </section>

        <section className="panel events">
          <div className="panel-title">
            <BatteryCharging size={18} />
            Backend
          </div>
          <dl>
            <dt>Robot</dt>
            <dd>{config.robotId}</dd>
            <dt>Backend</dt>
            <dd>{config.backendHttpUrl}</dd>
          </dl>
          <div className="event-list">
            {events.map((event) => <div key={event}>{event}</div>)}
          </div>
        </section>
      </section>
    </main>
  );
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function formatTof(value: number | null | undefined) {
  return typeof value === "number" ? `${value} mm` : "n/a";
}

function clampTof(value: number | null | undefined) {
  if (typeof value !== "number") {
    return 0;
  }
  return Math.max(8, Math.min(92, (value / 400) * 92));
}

function RobotFootprint({ tofMm }: { tofMm: Record<string, number | null> }) {
  return (
    <div className="footprint" aria-label="Robot footprint and TOF clearances">
      <div className="sensor-readout front" style={{ "--ray": `${clampTof(tofMm.front)}px` } as CSSProperties}>
        <span>front</span>
        <strong>{formatTof(tofMm.front)}</strong>
      </div>
      <div className="sensor-readout left45" style={{ "--ray": `${clampTof(tofMm.left45)}px` } as CSSProperties}>
        <span>left 45</span>
        <strong>{formatTof(tofMm.left45)}</strong>
      </div>
      <div className="robot-body">
        <div className="robot-nose" />
        <span>120 x 80 mm</span>
      </div>
      <div className="sensor-readout right45" style={{ "--ray": `${clampTof(tofMm.right45)}px` } as CSSProperties}>
        <span>right 45</span>
        <strong>{formatTof(tofMm.right45)}</strong>
      </div>
      <div className="sensor-readout back" style={{ "--ray": `${clampTof(tofMm.back)}px` } as CSSProperties}>
        <span>back</span>
        <strong>{formatTof(tofMm.back)}</strong>
      </div>
    </div>
  );
}

createRoot(document.getElementById("root")!).render(<App />);
