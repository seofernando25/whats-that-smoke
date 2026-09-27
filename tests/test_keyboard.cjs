const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync('src/whats_that_smoke/static/app.js', 'utf8');
const functions = source.slice(source.indexOf('function vector()'), source.indexOf('function stop('));
for (const [pressed, type, forward, turn] of [
  [['KeyW','KeyA'], 'drive', 1, 1],
  [['KeyW','KeyD'], 'drive', 1, -1],
  [['KeyS','KeyA'], 'drive', -1, -1],
  [['KeyS','KeyD'], 'drive', -1, 1],
  [['KeyQ'], 'drive', 0, 1],
  [['KeyE'], 'drive', 0, -1],
  [['KeyA'], 'sidestep', undefined, undefined],
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
