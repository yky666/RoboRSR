// Run with a modern Node.js; optionally pass the private runtime snapshot for coverage checks.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
const source = html.slice(html.indexOf('let DS_INVENTORY_DATE'), html.indexOf('// ---------- overview'));
const context = vm.createContext({URL, UI: {ds: {q: '', ep: [0, 100], pick: {}}}});
vm.runInContext(source, context);
const run = (s) => vm.runInContext(s, context);
const version = (id, values = {}) => ({id, name: id, source: 'sim', url: '', state: '已完备',
  episodes: null, hours: null, gb: null, selectedEpisodes: null, selectedHours: null, ...values});
const snapshot = {schemaVersion: 1, date: '2026-10-29', recordCount: 3, groups: [
  {id: 'sample', name: 'Sample', versions: [
    version('original', {episodes: 10, hours: 1, gb: 2, selectedEpisodes: 10, selectedHours: 1}),
    version('format-copy', {episodes: 10, hours: 1, gb: 3}),
  ]},
  {id: 'unknown', name: 'Unknown', versions: [version('metadata-only', {episodes: 0, selectedEpisodes: 0})]},
]};
context.snapshot = snapshot;
run('applyDatasetInventory(snapshot)');
assert.equal(run('DATASETS.length'), 2);
assert.equal(run('DATASETS[0].hours'), 1); // format copy retained, never counted twice
assert.equal(run('DATASETS[0].size'), '5 GB'); // storage keeps all copies
// display units: three significant digits, exact figures kept for tooltips
assert.equal(run('fmtCount(5614)'), '5,614');
assert.equal(run('fmtCount(214783)'), '21.5 万');
assert.equal(run('fmtCount(250304000)'), '2.5 亿');
assert.equal(run('fmtHours(132.445805)'), '132 h');
assert.equal(run('fmtHours(0.4)'), '24 分钟');
assert.equal(run('fmtGB(831.281)'), '831 GB');
assert.equal(run('fmtGB(49561.637)'), '49.6 TB');
assert.equal(run('fmtGB(0.42)'), '420 MB');
assert.equal(run('dsSizeText(DATASETS[0])'), '5 GB');
assert.equal(run('dsExact(DATASETS[0])'), '10 Episode · 1 h · 5 GB');
assert.equal(run('DATASETS[0].versions.length'), 2);
assert.equal(run('DATASETS[1].hours'), null); // unknown is not zero
assert.equal(run('dsEpText(DATASETS[1])'), '0');
assert.equal(run('dsEp(100)'), 10); // recomputed after runtime loading
assert.equal(run("UI.ds.q = 'format-copy'; dsMatch(DATASETS[0])"), true);
for (const mutate of [
  (s) => { s.recordCount++; },
  (s) => { s.groups[1].versions[0].id = 'original'; },
  (s) => { s.groups[1].id = 'sample'; },
  (s) => { s.groups[0].versions[0].hours = -1; },
  (s) => { s.groups[0].versions[0].url = 'javascript:alert(1)'; },
  (s) => { s.groups[0].versions[0].source = 'ego'; },
]) {
  const bad = JSON.parse(JSON.stringify(snapshot));
  mutate(bad);
  context.bad = bad;
  assert.throws(() => run('applyDatasetInventory(bad)'));
  assert.equal(run('DATASETS.length'), 2); // failed validation never replaces the last good catalog
}
context.partial = {schemaVersion: 1, date: '2026-10-29', recordCount: 1, groups: [
  {id: 'partial', name: 'Partial', versions: [version('video', {source: 'ego', episodes: 50, selectedHours: 0.25})]},
]};
run('applyDatasetInventory(partial)');
assert.equal(run('DATASETS[0].hours'), 0.25);
assert.equal(run('DATASETS[0].versions[0].hours'), null); // partial timing never fills the unknown complete total
assert.equal(run('DATASETS[0].episodes'), null); // video clips are not robot episodes
// publisher-declared totals fill only the unknown figures, are labelled, and never enter the local sums
context.declared = {schemaVersion: 1, date: '2026-10-30', recordCount: 1, groups: [
  {id: 'packed', name: 'Packed', versions: [version('tar', {source: 'real', episodes: 5, selectedEpisodes: 5})],
    declared: {episodes: 107000, hours: 300, qualifier: '约', url: 'https://example.org/card'}},
]};
run('applyDatasetInventory(declared)');
assert.equal(run('dsEpText(DATASETS[0])'), '5'); // a local count wins over the declared one
assert.equal(run('dsHoursText(DATASETS[0])'), '约300 h');
assert.equal(run('dsStatLabel(DATASETS[0], "hours")'), '官方时长');
assert.equal(run('DATASETS[0].hours'), null); // the sum still sees an unknown
for (const bad of [{episodes: 1}, {episodes: -1, url: 'https://x.org'}, {url: 'https://x.org'}]) {
  context.bad = {...context.declared, groups: [{...context.declared.groups[0], declared: bad}]};
  assert.throws(() => run('applyDatasetInventory(bad)')); // needs a source link and at least one figure
}
if (process.argv[2]) {
  context.full = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
  run('applyDatasetInventory(full)');
  assert.equal(run('DATASETS.reduce((n, d) => n + d.versions.length, 0)'), context.full.recordCount);
  assert.equal(run('new Set(DATASETS.flatMap(d => d.versions.map(v => v.id))).size'), context.full.recordCount);
  console.log(JSON.stringify(run('({groups: DATASETS.length, records: DS_INVENTORY.recordCount, hours: DATASETS.reduce((s,d) => s + (d.hours || 0), 0)})')));
}
context.INFO = {root: '/example'};
context.file = (p) => p;
context.api = async () => { throw new Error('unavailable'); };
run('loadDatasetInventory()').then(() => {
  assert.match(run('DS_INVENTORY_ERROR'), /完整盘点未加载/);
  assert.equal(run('DATASETS.length'), context.full ? context.full.groups.length : 1);
  console.log('dataset catalog checks passed');
}).catch((err) => { console.error(err); process.exitCode = 1; });
