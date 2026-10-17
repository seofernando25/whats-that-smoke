// Requires a running local panel, Playwright, and a Chromium-compatible browser.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({executablePath:process.env.CHROME_PATH || undefined,headless:true});
 const page=await browser.newPage();const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:8767');await page.waitForTimeout(500);
 const state=async()=>await(await page.request.get('http://127.0.0.1:8767/api/status')).json();
 const action=async action=>{const response=await page.request.post('http://127.0.0.1:8767/api/action',{data:{action}});assert(response.ok());await page.waitForTimeout(400);};
 const results=[];
 try {
  for(const keys of ['w','q','wa','wd','wq','wa']) {
   await action('reset');await action('play');
   const samples=[];const collect=async()=>samples.push({...await state(),t:performance.now()/1000});
   await collect();const timer=setInterval(()=>collect(),30);
   for(const key of keys)await page.keyboard.down(key);
   await page.waitForTimeout(2100);
   const release=performance.now()/1000;
   for(const key of keys)await page.keyboard.up(key);
   await page.waitForTimeout(450);clearInterval(timer);await page.waitForTimeout(60);
   samples.sort((a,b)=>a.t-b.t);
   const active=samples.filter(s=>s.t<=release);let angle=0,path=0;
   for(let i=1;i<active.length;i++){
    angle+=Math.atan2(Math.sin(active[i].pose.yaw-active[i-1].pose.yaw),Math.cos(active[i].pose.yaw-active[i-1].pose.yaw));
    path+=Math.hypot(active[i].pose.x-active[i-1].pose.x,active[i].pose.y-active[i-1].pose.y);
   }
   const late=samples.filter(s=>s.t>release+.2);
   const lateDrift=late.length?Math.hypot(late.at(-1).pose.x-late[0].pose.x,late.at(-1).pose.y-late[0].pose.y):Infinity;
   const degrees=angle*180/Math.PI,radius=Math.abs(angle)>.01?path/Math.abs(angle):null;
   assert.equal((await state()).driving,false,'Release must clear drive intent');
   assert(lateDrift<.005,`${keys}: still drifting 200 ms after release`);
   if(keys==='w') assert(path>1 && Math.abs(degrees)<8,'Forward must stay straight');
   else if(keys==='q')assert(degrees>200 && path<.10,'Q must pivot near its centre');
   else {
    assert(Math.abs(degrees)>120,`${keys}: steering mostly straight (${degrees} deg)`);
    assert((keys==='wd'?degrees<0:degrees>0),'Steering direction reversed');
    assert(path>.08 && radius<.18,'Steering must be a tight translating arc');
   }
   results.push({keys,turn_deg:degrees,path_m:path,radius_m:radius,late_drift_m:lateDrift});
  }
  const wa=results.find(r=>r.keys==='wa'),wq=results.find(r=>r.keys==='wq');
  assert(Math.abs(wa.turn_deg-wq.turn_deg)/wa.turn_deg<.25,'W+A and W+Q should be equivalent');
  assert.deepEqual(errors,[]);console.log(JSON.stringify(results,null,2));
  console.log('PASS: real browser key chords, six resets, pivot vs steering geometry, release braking');
 } finally {await action('reset');await action('play');await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
