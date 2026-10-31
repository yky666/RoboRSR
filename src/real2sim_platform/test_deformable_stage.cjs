// Exercise the exact object-stage function used by the browser drawer.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const html = fs.readFileSync(__dirname + '/index.html', 'utf8');
const match = html.match(/function deformablePhysicsState\(d\) \{[\s\S]*?\n\}/);
assert.ok(match, 'drawer delegates flexible state to a testable function');
const context = {
  todo: text => ({state:'pending',text}),
  bad: text => ({state:'fail',text}),
  ok: text => ({state:'pass',text}),
};
vm.createContext(context);
vm.runInContext(match[0],context);
function run(a) { return context.deformablePhysicsState({deformable:{family:'rope'},auto:{deformable_acceptance:a}}); }
const failed = {family:'rope',status:'fail',mesh_matches:true,registration_matches:true,checks:{reset_state:'fail'}};
assert.equal(run(failed).state,'fail');
assert.match(run(failed).text,/初始状态恢复/);
const other = run({...failed,mesh_matches:false});
assert.equal(other.state,'pending');
assert.match(other.text,/不对应当前库模型/);
assert.match(other.text,/初始状态恢复/);
assert.equal(run({...failed,registration_matches:false}).state,'pending');
assert.equal(run({...failed,status:'pass',checks:{}}).state,'pass');
assert.equal(run(null).state,'pending');
console.log('flexible stage version-binding checks passed');
