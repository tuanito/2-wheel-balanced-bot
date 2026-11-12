// Does the 3D parts viewer actually run? Nothing checked this, and it was dark for a long time.
//
// The page cannot be opened in a browser from the build, so this does the next best thing: it pulls
// the module out of the published page and EXECUTES it against a DOM stub, with WebGLRenderer
// replaced because there is no GPU here. Everything else is the real thing - three.js, all 111
// parts decoded from base64 into BufferGeometry, the wire bundles, the label and toggle wiring.
// Anything that throws here leaves the canvas blank in a browser with nothing to show for it, which
// is exactly how this bug hid: two separate module-resolution faults and a redeclared `_ray`, none
// of which produce a visible error.
//
//   node tools/check_viewer.mjs <page.html>
// node) but everything else - geometry, the wire bundles, the label/toggle wiring, the first frame -
// executes exactly as in the browser. Anything that throws here would leave the canvas blank there.
import fs from 'node:fs';
const page = fs.readFileSync(process.argv[2], 'utf8');
const i = page.indexOf('<script type="module">'), j = page.indexOf('</script>', i);
let src = page.slice(i + '<script type="module">'.length, j);
src = src.replace(/new THREE\.WebGLRenderer\(\{[^}]*\}\)/,
  '({setPixelRatio(){},setSize(){},render(){},domElement:{style:{}},info:{render:{}}})');

const mk = (id) => {
  const el = {
    id, style: {}, dataset: {}, classList: {add(){},remove(){},toggle(){},contains(){return false}},
    children: [], textContent: '', innerHTML: '', value: '', checked: false,
    appendChild(c){ this.children.push(c); return c; }, append(){}, after(){}, before(){},
    remove(){}, addEventListener(){}, removeEventListener(){}, setAttribute(){}, getAttribute(){return null},
    querySelector(){return mk('q')}, querySelectorAll(){return []},
    getBoundingClientRect(){return {left:0,top:0,width:800,height:600,right:800,bottom:600}},
    getContext(){return null}, insertAdjacentHTML(){}, focus(){}, click(){},
  };
  return el;
};
global.document = {
  getElementById: (id) => mk(id), createElement: (t) => mk(t),
  querySelector: () => mk('q'), querySelectorAll: () => [],
  addEventListener(){}, body: mk('body'), documentElement: mk('html'),
};
global.window = global; global.devicePixelRatio = 2;
global.matchMedia = () => ({matches:false, addEventListener(){}, addListener(){}});
global.requestAnimationFrame = () => 0;
global.addEventListener = () => {};
global.location = {hash:'', search:'', href:'http://x/'};
global.atob = (s) => Buffer.from(s, 'base64').toString('binary');
global.performance = {now: () => 0};
global.ResizeObserver = class { observe(){} unobserve(){} disconnect(){} };
global.IntersectionObserver = class { observe(){} unobserve(){} disconnect(){} };
global.getComputedStyle = () => ({getPropertyValue: () => ''});

const file = '/private/tmp/claude-502/-Users-tuan-data-projects-2-wheel-balanced-bot/b4c1c743-8a14-49ee-8c13-d1a132f33e0f/scratchpad/_viewer_run.mjs';
fs.writeFileSync(file, src);
// Running without throwing is not the same as showing the right robot: the page sat at the
// previous build for days with the HC-SR04 in the nose and the zone array nowhere, and nothing
// here would have said so. So assert the content too, on the text of the published page.
const want = [
  [/"name": ?"c_tof"/, 'the VL53L5CX zone array is in PARTS'],
  [/\["VL53L5CX[^\]]*"c_tof"/, 'the zone array has an Inside-view label'],
  [/let front='tof'/, "the front sensor shown by default is the array, not the sonar"],
  [/Front: zone array/, 'the front-sensor toggle exists'],
];
const missing = want.filter(([re]) => !re.test(page)).map(([, w]) => w);
if (missing.length) {
  process.exitCode = 1;
  console.log('CONTENT MISSING: ' + missing.join('; '));
}
console.log(`  front sensor shown = ${/let front='tof'/.test(page) ? 'VL53L5CX zone array' : 'HC-SR04 sonar'}`);

try {
  await import('file://' + file);
  console.log('VIEWER MODULE RAN TO COMPLETION - no exception');
} catch (e) {
  process.exitCode = 1;
  console.log('THREW:', e.constructor.name + ':', e.message);
  console.log((e.stack || '').split('\n').slice(1, 4).join('\n'));
}
