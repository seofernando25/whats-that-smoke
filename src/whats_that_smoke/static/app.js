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
  if (!next.aruco_visible) return;
  const image = $("#camera-feed"), naturalWidth = image.naturalWidth || 640, naturalHeight = image.naturalHeight || 480;
  const scale = Math.min(width / naturalWidth, height / naturalHeight);
  const x0 = (width - naturalWidth * scale) / 2, y0 = (height - naturalHeight * scale) / 2;
  const markers = next.aruco_markers?.length ? next.aruco_markers : [{ id: next.aruco_id, distance_m: next.aruco_distance_m, corners: next.aruco_corners, target: true }];
  markers.forEach(marker => {
    if (marker.corners.length !== 4) return;
    const points = marker.corners.map(([x, y]) => [x0 + x * scale, y0 + y * scale]);
    const color = marker.target ? "#e11d48" : "#2563eb";
    arucoContext.setLineDash(marker.tracked ? [6, 4] : []);
    arucoContext.beginPath(); arucoContext.moveTo(...points[0]); points.slice(1).forEach(point => arucoContext.lineTo(...point)); arucoContext.closePath();
    arucoContext.lineWidth = marker.target ? 3 : 2; arucoContext.strokeStyle = color; arucoContext.stroke();
    arucoContext.fillStyle = color; arucoContext.font = "700 13px system-ui";
    const source = marker.source === "decode-fallback"
      ? ` · ${marker.confidence === 1 ? "EXACT CODE · RESAMPLED" : "CORRECTED CODE · RESAMPLED"}`
      : marker.tracked ? ` · ${marker.source.toUpperCase()} · ${marker.age_ms} ms` : "";
    arucoContext.fillText(`ARUCO ${marker.id}${source} · ${marker.distance_m.toFixed(2)} m`, points[0][0], points[0][1] - 8);
  });
  arucoContext.setLineDash([]);
}

function vector() {
  const forward = Number(keys.has("KeyW")) - Number(keys.has("KeyS"));
  const pivot = Number(keys.has("KeyQ")) - Number(keys.has("KeyE"));
  const steering = Number(keys.has("KeyA")) - Number(keys.has("KeyD"));
  return {
    forward,
    strafe: 0,
    turn: pivot || forward * steering,
  };
}

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function drive() {
  const next = vector();
  document.querySelectorAll("[data-key]").forEach((button) => button.classList.toggle("active", keys.has(button.dataset.key)));
  if (!state.armed || !state.you_are_owner) return;
  const direction = Number(keys.has("KeyA")) - Number(keys.has("KeyD"));
  if (direction && !next.forward && !next.turn) {
    send({ type: "sidestep", direction, speed_limit: Number(speed.value) });
  } else {
    send({ type: "drive", ...next, speed_limit: Number(speed.value) });
  }
}

function stop(reason = "client-stop") {
  keys.clear();
  document.querySelectorAll("[data-key]").forEach((button) => button.classList.remove("active"));
  send({ type: "stop", reason });
}

function render(next) {
  state = next;
  const cameraFresh = next.camera_frame_age_ms != null && next.camera_frame_age_ms < 500;
  $("#feed-stats").textContent = `${cameraFresh ? "LIVE" : "STALE"} · 1280×720 · ${(next.camera_fps || 0).toFixed(1)} FPS · MONO GS`;
  $("#vision-rate").textContent = `${(next.aruco_fps || 0).toFixed(1)} processed fps`;
  $("#vision-time").textContent = `${(next.aruco_processing_ms || 0).toFixed(0)} ms compute · ${next.aruco_result_age_ms ?? "—"} ms age`;
  $("#motion").textContent = next.stopped ? "STOPPED" : "MOVING";
  $("#motion").classList.toggle("moving", !next.stopped);
  $("#reason").textContent = next.reason;
  $("#turn-feedback").textContent = `${(next.turn_target_dps || 0).toFixed(0)}°/s target · ${(next.heading_rate_dps || 0).toFixed(0)}°/s actual`;
  $("#ownership").textContent = next.you_are_owner ? "controller" : next.owner ? "busy" : "available";
  $("#arm").textContent = next.armed && next.you_are_owner ? "DISARM / STOP" : "ARM CONTROLS";
  $("#arm").classList.toggle("armed", next.armed && next.you_are_owner);
  $("#clients").textContent = next.clients;
  $("#forward").textContent = next.forward.toFixed(2);
  $("#strafe").textContent = next.strafe.toFixed(2);
  $("#turn").textContent = next.turn.toFixed(2);
  $("#watchdog").textContent = `${next.watchdog_ms} ms`;
  $("#pan").textContent = `${next.pan_us} µs`;
  $("#tilt").textContent = `${next.tilt_us} µs`;
  $("#aruco-toggle").textContent = next.aruco_enabled ? "ARUCO ON" : "ARUCO OFF";
  $("#aruco-toggle").setAttribute("aria-pressed", String(next.aruco_enabled));
  $("#follow-toggle").textContent = next.aruco_follow ? "FOLLOW ON" : "FOLLOW OFF";
  $("#follow-toggle").setAttribute("aria-pressed", String(next.aruco_follow));
  $("#aruco-readout").textContent = next.aruco_visible ? `${next.aruco_markers?.length || 1} tag(s) · target ${next.aruco_id} · ${next.aruco_distance_m.toFixed(2)} m` : next.aruco_status;
  $("#imu-status").textContent = next.imu_connected ? `${next.imu_rate_hz.toFixed(0)} Hz · ${next.imu_age_ms} ms` : next.imu_status;
  const vectorText = (vector, digits) => vector ? ["x", "y", "z"].map(axis => vector[axis].toFixed(digits)).join(" / ") : "— / — / —";
  $("#imu-accel").textContent = vectorText(next.imu_accel_g, 3);
  $("#imu-gyro").textContent = vectorText(next.imu_gyro_dps, 1);
  const heading = Number(next.heading_deg) || 0;
  $("#heading-needle").style.transform = `rotate(${heading}deg)`;
  $("#heading-degrees").textContent = next.heading_calibrated ? String(Math.round(heading) % 360).padStart(3, "0") : "—";
  const headingStatus = (next.heading_status || "hold still · zeroing").toUpperCase();
  const headingRate = Number(next.heading_rate_dps) || 0;
  $("#heading-status").textContent = next.heading_calibrated && !next.heading_stationary
    ? `${headingStatus} · ${headingRate.toFixed(0)}°/S`
    : headingStatus;
  $("#heading-calibrate").disabled = !next.imu_connected;
  $("#heading-widget").classList.toggle("uncalibrated", !next.heading_calibrated);
  $("#heading-widget").classList.toggle("moving", !next.heading_stationary && next.heading_calibrated);
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
    driveRefresh = setInterval(() => {
      if (["KeyW", "KeyS", "KeyA", "KeyD", "KeyQ", "KeyE"].some(key => keys.has(key))) drive();
    }, 150);
  });
  socket.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "state") render(message.state);
  });
  socket.addEventListener("close", () => {
    keys.clear();
    document.querySelectorAll("[data-key]").forEach(button => button.classList.remove("active"));
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
  if (!["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"].includes(event.code) || event.repeat) return;
  event.preventDefault(); keys.add(event.code); drive();
});
document.addEventListener("keyup", (event) => {
  if (event.code.startsWith("Arrow")) {
    event.preventDefault(); document.querySelector(`[data-key="${event.code}"]`)?.classList.remove("active"); return;
  }
  if (!["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"].includes(event.code)) return;
  event.preventDefault(); keys.delete(event.code); drive();
});
$("#stop").addEventListener("click", () => stop("button-stop"));
$("#arm").addEventListener("click", () => state.armed && state.you_are_owner ? stop("disarm") : send({ type: "arm" }));
$("#aruco-toggle").addEventListener("click", () => send({ type: "aruco", enabled: !state.aruco_enabled, follow: state.aruco_follow ? false : null }));
$("#follow-toggle").addEventListener("click", () => send({ type: "aruco", enabled: true, follow: !state.aruco_follow }));
$("#heading-calibrate").addEventListener("click", () => send({ type: "heading-calibrate" }));
speed.addEventListener("input", () => {
  speedValue.textContent = speed.value;
  $("#speed-meter").style.width = `${((speed.value - 500) / 1300) * 100}%`;
  if (keys.size) drive();
});
window.addEventListener("blur", () => stop("window-blur"));
document.addEventListener("visibilitychange", () => { if (document.hidden) stop("hidden"); });
window.addEventListener("pagehide", () => stop("pagehide"));

connect();
