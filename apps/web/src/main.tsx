import { createRoot } from "react-dom/client";
import { Activity, BatteryCharging, CircleStop, Dock, Gauge, Map, RotateCcw, RotateCw } from "lucide-react";
import { useEffect, useMemo, useState } from "react";

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
        setEvents((current) => [`${new Date().toLocaleTimeString()} ${event.kind}`, ...current].slice(0, 10));
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

  const tofRows = useMemo(() => Object.entries(telemetry.tof_mm ?? {}), [telemetry.tof_mm]);

  function drive(linear: number, angular: number) {
    sendCommand("drive", { linear, angular, durationMs: 300 });
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
          <div className="tof">
            {tofRows.length === 0 ? <p>No TOF readings yet.</p> : null}
            {tofRows.map(([name, value]) => (
              <div key={name}>
                <span>{name}</span>
                <strong>{value ?? "n/a"} mm</strong>
              </div>
            ))}
          </div>
        </section>

        <section className="panel drive">
          <div className="panel-title">
            <Gauge size={18} />
            Drive
          </div>
          <div className="pad">
            <button onClick={() => drive(1, 0)}>Forward</button>
            <button onClick={() => drive(0, -1)}><RotateCcw size={18} />Left</button>
            <button className="stop" onClick={() => sendCommand("stop")}><CircleStop size={20} />Stop</button>
            <button onClick={() => drive(0, 1)}><RotateCw size={18} />Right</button>
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

createRoot(document.getElementById("root")!).render(<App />);
