// Aviation Dynamics — the README's ABI tables against the module's avd_layout().
// SPDX-License-Identifier: MIT
//
//   node web/abi_check.mjs      (after web/build_wasi.sh)
//
// Checks that (1) avd_loader.mjs's EXPECTED_ABI is the module's
// avd_abi_version(); (2) the "ABI version" table in web/README.md lists every
// avd_layout(which) entry, in order, with the module's value, and states the
// version; (3) the sizes the README's prose quotes for the arrays
// ("64 doubles", "1,352 doubles", "at most 4,096" ...) are the module's.
import { readFile } from 'node:fs/promises';
import createAvdModule, { EXPECTED_ABI, LAYOUT_NAMES } from './dist/avd.mjs';

const A = await createAvdModule();
const readme = await readFile(new URL('./README.md', import.meta.url), 'utf8');
let failures = 0;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) failures++; };
const num = (s) => Number(String(s).replace(/,/g, ''));

// (1) version handshake
check(A.abiVersion === EXPECTED_ABI, `loader EXPECTED_ABI ${EXPECTED_ABI} == module avd_abi_version() ${A.abiVersion}`);
const stated = readme.match(/currently \*\*(\d+)\*\*/);
check(stated && num(stated[1]) === A.abiVersion, `README states ABI version ${stated && stated[1]}`);

// (2) the layout table of the "ABI version" section: "| which | name | value | ... |"
const sectionStart = readme.indexOf('## ABI version');
const sectionEnd = readme.indexOf('\n## ', sectionStart + 1);
const section = sectionStart < 0 ? '' : readme.slice(sectionStart, sectionEnd < 0 ? undefined : sectionEnd);
check(sectionStart >= 0, 'README has an "ABI version" section');
const rows = [...section.matchAll(/^\| (\d+) \| `?(\w+)`? \| ([\d,]+) \|/gm)].map((m) => ({ which: +m[1], name: m[2], value: num(m[3]) }));
check(rows.length === LAYOUT_NAMES.length, `README layout table has ${rows.length} rows, loader knows ${LAYOUT_NAMES.length}`);
for (const r of rows) {
  const name = LAYOUT_NAMES[r.which];
  const v = A.avd_layout(r.which);
  check(name === r.name && v === r.value, `which ${r.which}: README ${r.name} = ${r.value}, module ${name} = ${v}`);
}
check(A.avd_layout(LAYOUT_NAMES.length) === -1 && A.avd_layout(-1) === -1, 'avd_layout() out of range returns -1');

// (3) the sizes quoted in the prose
const prose = [
  ['state', /`avd_state\(\)`: ([\d,]+) doubles/],
  ['loadsheet', /`avd_loadsheet\(planned\)`: ([\d,]+) doubles/],
  ['cockpit', /`avd_cockpit\(\)`: ([\d,]+) doubles/],
  ['jet', /`avd_jet\(\)` \| ([\d,]+) doubles/],
  ['flow', /snapshot \(([\d,]+) doubles\)/],
  ['compassCard', /`avd_compass_card\(\)` \| ([\d,]+) doubles/],
  ['wakeMax', /at most ([\d,]+)\. `A\.wake\(\)`/],
  ['bodiesMax', /\(up to ([\d,]+)\): position \(structural/],
  ['flowStrips', /per lifting strip at 16 \+ 20i \(up to ([\d,]+)\)/],
];
for (const [name, re] of prose) {
  const m = readme.match(re);
  check(m && num(m[1]) === A.layout[name], `README prose: ${name} ${m ? m[1] : '(not found)'} vs module ${A.layout[name]}`);
}
console.log(`build id: ${A.buildId()}`);
console.log(failures ? `${failures} FAILED` : 'all ABI checks passed');
process.exit(failures ? 1 : 0);
