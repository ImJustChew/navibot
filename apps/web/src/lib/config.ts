export const config = {
  backendHttpUrl: import.meta.env.VITE_BACKEND_HTTP_URL ?? "http://localhost:8787",
  backendWsUrl: import.meta.env.VITE_BACKEND_WS_URL ?? "ws://localhost:8787",
  robotId: import.meta.env.VITE_ROBOT_ID ?? "devbot",
};
