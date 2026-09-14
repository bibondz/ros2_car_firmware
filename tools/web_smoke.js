/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
/**
 * Run web/app.js against a minimal fake DOM and report anything it throws.
 *
 * WHY THIS EXISTS
 *
 * `node --check` only proves the file PARSES. It says nothing about whether the
 * code runs, and the failure that prompted this was pure runtime: a `var` was
 * used at start-up but assigned two hundred lines further down. Hoisting made
 * it defined-but-undefined, applyBasemap() threw, and everything after it -
 * including connect() - never ran. The page sat on "connecting..." while the
 * robot was publishing perfectly well, and every static check was green.
 *
 * This is not a browser and does not pretend to be. It answers one question:
 * does loading this file throw? That is the question that was going unasked.
 *
 *     node tools/web_smoke.js
 *
 * Exits non-zero on a throw, so it can gate a promotion.
 */
const fs = require('fs');
const path = require('path');
const root = path.join(__dirname, '..');
const web = path.join(root, 'gps_localize_ws', 'src', 'gps_localize', 'web');
const src = fs.readFileSync(path.join(web, 'app.js'), 'utf8');
const html = fs.readFileSync(path.join(web, 'index.html'), 'utf8');
const ids = new Set([...html.matchAll(/id="([A-Za-z0-9_]+)"/g)].map(m => m[1]));
const mk = (id) => ({
  id, className: '', textContent: '', innerHTML: '', value: '', disabled: false,
  dataset: {}, style: {}, checked: false, hidden: false,
  addEventListener(){}, removeEventListener(){}, appendChild(){}, removeChild(){},
  setAttribute(){}, getAttribute(){ return null; }, removeAttribute(){},
  focus(){}, click(){},
  querySelectorAll(){ return []; }, getContext(){ return new Proxy({}, {get:()=>()=>{}}); },
  getBoundingClientRect(){ return {left:0,top:0,width:900,height:560}; },
  classList:{add(){},remove(){},toggle(){},contains(){return false;}},
});
const nodes = {};
global.document = {
  body: mk('body'), documentElement: mk('html'),
  getElementById: (id) => ids.has(id) ? (nodes[id] = nodes[id] || mk(id)) : null,
  querySelectorAll: () => [], querySelector: () => null,
  createElement: (t) => mk(t), addEventListener(){},
};
global.window = { localStorage:{getItem:()=>null,setItem(){},removeItem(){}},
                  addEventListener(){}, location:{href:'http://x/',protocol:'http:',host:'x'},
                  matchMedia:()=>({matches:false,addEventListener(){}}), devicePixelRatio:1 };
global.navigator = { userAgent:'node' };
global.fetch = () => new Promise(()=>{});           // never resolves: no network here
global.EventSource = function(){ this.addEventListener=()=>{}; this.close=()=>{}; };
global.Image = function(){ };
global.setInterval = () => 0; global.setTimeout = () => 0;
global.requestAnimationFrame = () => 0;
try {
  new Function(src)();
  console.log('  web_smoke: app.js executed with no exception');
} catch (e) {
  console.log('  web_smoke: THREW ' + e.message);
  console.log('  ' + (e.stack || '').split('\n')[1]);
  process.exit(1);
}
