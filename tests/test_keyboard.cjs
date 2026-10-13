const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync('src/whats_that_smoke/static/app.js', 'utf8');
const functions = source.slice(source.indexOf('function vector()'), source.indexOf('function stop('));
for (const [pressed, type, forward, turn] of [
  [['KeyW','KeyA'], 'drive', 1, 1],
  [['KeyW','KeyD'], 'drive', 1, -1],
  [['KeyS','KeyA'], 'drive', -1, 1],
  [['KeyS','KeyD'], 'drive', -1, -1],
  [['KeyQ'], 'drive', 0, 1],
  [['KeyE'], 'drive', 0, -1],
  [['KeyA'], 'drive', 0, 1],
  [[], 'drive', 0, 0],
]) {
  const messages = [];
  const context = { keys: new Set(pressed), state: { armed:true, you_are_owner:true },
    speed:{value:1800}, WebSocket:{OPEN:1},
    socket:{readyState:1, send: data => messages.push(JSON.parse(data))},
    document:{querySelectorAll: () => []} };
  vm.runInNewContext(functions + '\ndrive();', context);
  assert.equal(messages[0].type, type);
  assert.equal(messages[0].forward, forward);
  assert.equal(messages[0].turn, turn);
}
console.log('8 keyboard mappings passed');

const handlers = {};
const messages = [];
const context = {
  keys: new Set(), state: {armed:true, you_are_owner:true, aruco_follow:false},
  speed: {value:1800, addEventListener: (event, fn) => { handlers.speed = fn; }},
  speedValue: {}, WebSocket:{OPEN:1},
  socket: {readyState:1, send: data => messages.push(JSON.parse(data))},
  document: {querySelectorAll: () => []},
  $: selector => selector === '#follow-toggle'
    ? {addEventListener: (event, fn) => { handlers.follow = fn; }} : {style:{}}
};
const followListener = source.split('\n').find(line => line.startsWith('$("#follow-toggle").addEventListener'));
const speedListener = source.slice(source.indexOf('speed.addEventListener("input"'), source.indexOf('window.addEventListener("blur"'));
vm.runInNewContext(functions + '\n' + followListener + '\n' + speedListener, context);
handlers.follow();
assert.equal(messages.at(-1).speed_limit, 1800);
assert.equal(messages.at(-1).follow, true);
context.state.aruco_follow = true;
context.speed.value = 1200;
handlers.speed();
assert.equal(messages.at(-1).speed_limit, 1200);
assert.equal(messages.at(-1).follow, undefined);
const count = messages.length;
context.state.you_are_owner = false;
handlers.speed();
assert.equal(messages.length, count);
console.log('Follow speed slider and ownership checks passed');
