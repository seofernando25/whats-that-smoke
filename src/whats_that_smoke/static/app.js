const keys = new Set();
let socket;
let heartbeat;
let driveRefresh;
let reconnectTimer;
let state = {};

const $ = (selector) => document.querySelector(selector);
const connection = $("#connection");
const speed = $("#speed");
const speedValue = $("#speed-value");
const arucoCanvas = $("#aruco-overlay");
const arucoContext = arucoCanvas.getContext("2d");

function drawAruco(next) {
  const ratio = devicePixelRatio || 1;
  const width = arucoCanvas.clientWidth, height = arucoCanvas.clientHeight;
  if (arucoCanvas.width !== Math.round(width * ratio) || arucoCanvas.height !== Math.round(height * ratio)) {
    arucoCanvas.width = Math.round(width * ratio); arucoCanvas.height = Math.round(height * ratio);
  }
  arucoContext.setTransform(ratio, 0, 0, ratio, 0, 0);
  arucoContext.clearRect(0, 0, width, height);
  if (!next.aruco_visible || next.aruco_corners.length !== 4) return;
  const image = $("#camera-feed"), naturalWidth = image.naturalWidth || 640, naturalHeight = image.naturalHeight || 480;
  const scale = Math.min(width / naturalWidth, height / naturalHeight);
  const x0 = (width - naturalWidth * scale) / 2, y0 = (height - naturalHeight * scale) / 2;
  const points = next.aruco_corners.map(([x, y]) => [x0 + x * scale, y0 + y * scale]);
  arucoContext.beginPath(); arucoContext.moveTo(...points[0]); points.slice(1).forEach(point => arucoContext.lineTo(...point)); arucoContext.closePath();
  arucoContext.lineWidth = 3; arucoContext.strokeStyle = "#e11d48"; arucoContext.stroke();
  arucoContext.fillStyle = "#e11d48"; arucoContext.font = "700 13px system-ui";
  arucoContext.fillText(`ARUCO ${next.aruco_id} · ${next.aruco_distance_m.toFixed(2)} m`, points[0][0], points[0][1] - 8);
}

function vector() {
  return {
    forward: (keys.has("KeyW") ? 1 : 0) - (keys.has("KeyS") ? 1 : 0),
    turn: (keys.has("KeyA") ? 1 : 0) - (keys.has("KeyD") ? 1 : 0),
  };
}

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function drive() {
  const next = vector();
  document.querySelectorAll("[data-key]").forEach((button) => button.classList.toggle("active", keys.has(button.dataset.key)));
  if (state.armed) send({ type: "drive", ...next, speed_limit: Number(speed.value) });
}

function stop(reason = "client-stop") {
  keys.clear();
  document.querySelectorAll("[data-key]").forEach((button) => button.classList.remove("active"));
  send({ type: "stop", reason });
}

function render(next) {
  state = next;
  $("#motion").textContent = next.stopped ? "STOPPED" : "MOVING";
  $("#motion").classList.toggle("moving", !next.stopped);
  $("#reason").textContent = next.reason;
  $("#ownership").textContent = next.you_are_owner ? "controller" : next.owner ? "busy" : "available";
  $("#arm").textContent = next.armed && next.you_are_owner ? "DISARM / STOP" : "ARM CONTROLS";
  $("#arm").classList.toggle("armed", next.armed && next.you_are_owner);
  $("#clients").textContent = next.clients;
  $("#forward").textContent = next.forward.toFixed(2);
  $("#turn").textContent = next.turn.toFixed(2);
  $("#watchdog").textContent = `${next.watchdog_ms} ms`;
  $("#pan").textContent = `${next.pan_us} µs`;
  $("#tilt").textContent = `${next.tilt_us} µs`;
  $("#aruco-toggle").textContent = next.aruco_enabled ? "ARUCO ON" : "ARUCO OFF";
  $("#aruco-toggle").setAttribute("aria-pressed", String(next.aruco_enabled));
  $("#follow-toggle").textContent = next.aruco_follow ? "FOLLOW ON" : "FOLLOW OFF";
  $("#follow-toggle").setAttribute("aria-pressed", String(next.aruco_follow));
  $("#aruco-readout").textContent = next.aruco_visible ? `ID ${next.aruco_id} · ${next.aruco_distance_m.toFixed(2)} m` : next.aruco_status;
  drawAruco(next);
  Object.entries(next.wheels).forEach(([name, duty]) => document.querySelector(`[data-wheel="${name}"]`).textContent = duty);
}

function connect() {
  clearTimeout(reconnectTimer);
  socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  socket.addEventListener("open", () => {
    connection.className = "connection online";
    connection.lastChild.textContent = " Online";
    heartbeat = setInterval(() => send({ type: "heartbeat" }), 200);
    driveRefresh = setInterval(() => { if (keys.size && state.armed && state.you_are_owner) drive(); }, 150);
  });
  socket.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "state") render(message.state);
  });
  socket.addEventListener("close", () => {
    clearInterval(heartbeat);
    clearInterval(driveRefresh);
    connection.className = "connection offline";
    connection.lastChild.textContent = " Offline";
    reconnectTimer = setTimeout(connect, 1000);
  });
}

document.addEventListener("keydown", (event) => {
  if (event.code === "Space") { event.preventDefault(); stop("space-stop"); return; }
  if (event.code.startsWith("Arrow")) {
    event.preventDefault();
    document.querySelector(`[data-key="${event.code}"]`)?.classList.add("active");
    if (!state.armed || !state.you_are_owner) return;
    const moves = { ArrowLeft: ["pan", 25], ArrowRight: ["pan", -25], ArrowUp: ["tilt", -25], ArrowDown: ["tilt", 25] };
    const [axis, delta] = moves[event.code]; send({ type: "camera", axis, delta }); return;
  }
  if (!["KeyW", "KeyA", "KeyS", "KeyD"].includes(event.code) || event.repeat) return;
  event.preventDefault(); keys.add(event.code); drive();
});
document.addEventListener("keyup", (event) => {
  if (event.code.startsWith("Arrow")) {
    event.preventDefault(); document.querySelector(`[data-key="${event.code}"]`)?.classList.remove("active"); return;
  }
  if (!["KeyW", "KeyA", "KeyS", "KeyD"].includes(event.code)) return;
  event.preventDefault(); keys.delete(event.code); drive();
});
$("#stop").addEventListener("click", () => stop("button-stop"));
$("#arm").addEventListener("click", () => state.armed && state.you_are_owner ? stop("disarm") : send({ type: "arm" }));
$("#aruco-toggle").addEventListener("click", () => send({ type: "aruco", enabled: !state.aruco_enabled, follow: state.aruco_follow ? false : null }));
$("#follow-toggle").addEventListener("click", () => send({ type: "aruco", enabled: true, follow: !state.aruco_follow }));
speed.addEventListener("input", () => {
  speedValue.textContent = speed.value;
  $("#speed-meter").style.width = `${((speed.value - 500) / 1300) * 100}%`;
  if (keys.size) drive();
});
window.addEventListener("blur", () => stop("window-blur"));
document.addEventListener("visibilitychange", () => { if (document.hidden) stop("hidden"); });
window.addEventListener("pagehide", () => stop("pagehide"));

connect();
