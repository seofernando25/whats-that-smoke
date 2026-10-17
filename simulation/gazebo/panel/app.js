const $ = selector => document.querySelector(selector);
const keyboard = new Set(), pointers = new Map();
const allowed = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE']);
const client = crypto.randomUUID();
let seq = 0, busy = false, suspended = false, online = false, paused = true;
let state = {pose: {x: 0, y: 0, yaw: 0}}, trail = [];
const keys = () => [...new Set([...keyboard, ...pointers.values()])];
function notice(message = '') { $('#notice').textContent = message; }
function paintKeys() {
  document.querySelectorAll('[data-key]').forEach(button => {
    button.classList.toggle('is-down', keys().includes(button.dataset.key));
  });
}
async function post(path, message) {
  const response = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(message)});
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || 'Control request failed');
  return body;
}
function payload() { return {client, seq: ++seq, keys: suspended ? [] : keys(), speed: .12, pwm: Number($('#speed').value)}; }
async function drive() {
  if (busy || suspended || !online || paused || !keys().length || document.hidden) return;
  busy = true;
  try { await post('/api/drive', payload()); }
  catch (error) { notice(error.message); keyboard.clear(); pointers.clear(); paintKeys(); }
  finally { busy = false; }
}
function stop() {
  keyboard.clear(); pointers.clear(); paintKeys();
  const body = JSON.stringify({...payload(), keys: []});
  if (!navigator.sendBeacon('/api/drive', new Blob([body], {type: 'application/json'}))) {
    fetch('/api/drive', {method: 'POST', headers: {'Content-Type': 'application/json'}, body, keepalive: true}).catch(() => {});
  }
}
document.addEventListener('keydown', event => {
  if (event.metaKey || event.ctrlKey || event.altKey || event.target.isContentEditable) return;
  if (event.code === 'Space' || event.code === 'Escape') { event.preventDefault(); stop(); return; }
  if (!allowed.has(event.code)) return;
  event.preventDefault();
  if (event.repeat || suspended || !online || paused) return;
  keyboard.add(event.code); paintKeys(); drive();
});
document.addEventListener('keyup', event => {
  if (!allowed.has(event.code)) return;
  event.preventDefault(); keyboard.delete(event.code); paintKeys();
  if (!keys().length) stop(); else drive();
});
document.querySelectorAll('[data-key]').forEach(button => {
  button.addEventListener('pointerdown', event => {
    event.preventDefault();
    if (!online || paused || suspended) return;
    button.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, button.dataset.key); paintKeys(); drive();
  });
  const release = event => {
    if (!pointers.has(event.pointerId)) return;
    pointers.delete(event.pointerId); paintKeys();
    if (!keys().length) stop(); else drive();
  };
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(name, release);
});
$('#stop').onclick = stop;
$('#speed').addEventListener('input', () => {
  $('#speed-value').innerHTML = `${Number($('#speed').value)} <small>PWM</small>`;
  drive();
});
for (const action of ['launch', 'play', 'pause', 'reset', 'imu-zero']) {
  $(`#${action}`).onclick = async () => {
    stop(); suspended = true; notice();
    document.querySelectorAll('.sim-buttons button, #imu-zero, #launch').forEach(button => button.disabled = true);
    try {
      await post('/api/action', {action});
      if (action === 'reset') { trail = []; notice('Reset complete. Click Play, then hold a driving key.'); }
      if (action === 'imu-zero') notice('Hold still for 3 seconds of simulation time to zero the IMU.');
      if (action === 'launch') notice('Gazebo is opening. The connection indicator will turn green when ready.');
    } catch (error) { notice(error.message); }
    finally { suspended = false; document.querySelectorAll('.sim-buttons button, #imu-zero, #launch').forEach(button => button.disabled = false); }
  };
}
window.addEventListener('blur', () => { stop(); $('#focus-note').textContent = 'Controls released. Click this window to drive again.'; });
window.addEventListener('focus', () => { $('#focus-note').textContent = 'Keep this window focused while driving.'; });
window.addEventListener('pagehide', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
async function poll() {
  try {
    state = await (await fetch('/api/status', {cache: 'no-store'})).json();
    const changed = online !== state.online || paused !== state.paused;
    online = state.online; paused = state.paused;
    if (changed && (!online || paused)) stop();
    $('#connection').className = `connection ${online ? paused ? 'paused' : 'online' : ''}`;
    $('#connection span').textContent = online ? paused ? 'Simulation paused' : 'Connected to Gazebo' : 'Gazebo offline';
    $('#motion').textContent = !online ? 'Open Gazebo to get started' : paused ? 'Paused · click Play to drive' : state.driving || state.linear || state.angular ? 'Driving · release to stop' : 'Ready for your input';
    $('#x').innerHTML = `${state.pose.x.toFixed(2)}<small>m</small>`;
    $('#y').innerHTML = `${state.pose.y.toFixed(2)}<small>m</small>`;
    $('#heading').innerHTML = `${Math.round(state.pose.yaw * 180 / Math.PI)}<small>°</small>`;
    const imu = state.imu || {}, calibrated = online && imu.heading_calibrated;
    const angle = value => Number.isFinite(value) ? `${value.toFixed(2)}<small>°</small>` : '—';
    $('#imu-heading').innerHTML = calibrated ? angle(imu.heading_deg) : '—';
    $('#imu-truth').innerHTML = calibrated ? angle(imu.truth_heading_deg) : '—';
    $('#imu-error').innerHTML = calibrated ? angle(imu.heading_error_deg) : '—';
    $('#imu-status').textContent = !online ? 'IMU offline' : paused ? 'Paused · IMU time held' : !imu.streaming ? 'Waiting for IMU samples' : imu.heading_status;
    const last = trail.at(-1);
    if (online && (!last || Math.hypot(state.pose.x-last.x, state.pose.y-last.y) > .003)) {
      trail.push({...state.pose}); if (trail.length > 500) trail.shift();
    }
    drawMap();
  } catch {
    online = false; stop(); $('#connection').className = 'connection';
    $('#connection span').textContent = 'Control server offline';
  }
  setTimeout(poll, 150);
}
function drawMap() {
  const canvas = $('#map'), ctx = canvas.getContext('2d'), w = canvas.width, h = canvas.height;
  const scale = Math.min(230, 190 / Math.max(.8, Math.abs(state.pose.x), Math.abs(state.pose.y)));
  const pt = (x, y) => [w/2+x*scale, h/2-y*scale];
  ctx.clearRect(0, 0, w, h); ctx.fillStyle = '#dce5d9'; ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#c5d2c2'; ctx.lineWidth = 1;
  for (let i=-30; i<=30; i++) {
    const [x,y] = pt(i*.1,i*.1);
    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke();
  }
  ctx.strokeStyle = '#8daa89'; ctx.lineWidth = 2;
  ctx.strokeRect(w/2-.15*scale,h/2-.1*scale,.3*scale,.2*scale);
  ctx.fillStyle = '#668363'; ctx.font = '14px system-ui'; ctx.fillText('HOME',w/2-22,h/2+.1*scale+21);
  ctx.fillText('+Y',w/2+10,24); ctx.fillText('+X',w-36,h/2-10);
  ctx.strokeStyle = '#82a27a'; ctx.lineWidth = 3; ctx.lineJoin = 'round'; ctx.beginPath();
  trail.forEach((p,i) => { const xy=pt(p.x,p.y); if(i)ctx.lineTo(...xy);else ctx.moveTo(...xy); }); ctx.stroke();
  const [x,y] = pt(state.pose.x,state.pose.y);
  ctx.save(); ctx.translate(x,y); ctx.rotate(-state.pose.yaw);
  ctx.fillStyle = '#294c32'; ctx.strokeStyle = '#dcebcf'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(19,0); ctx.lineTo(-13,-11); ctx.lineTo(-8,0); ctx.lineTo(-13,11); ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.restore();
}
setInterval(drive, 90);
poll();
