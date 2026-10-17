// Requires the local panel, Gazebo, Playwright and Chromium.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({executablePath:process.env.CHROME_PATH || undefined,headless:true});
 const page=await browser.newPage({viewport:{width:1280,height:1120}});const errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 const status=async()=>await(await page.request.get('http://127.0.0.1:8767/api/status')).json();
 const action=async name=>{const response=page.waitForResponse(r=>r.url().endsWith('/api/action'));await page.locator(`#${name}`).click();assert((await response).ok(),`${name} action failed`);};
 const wait=async predicate=>{
  const deadline=Date.now()+25000;
  while(Date.now()<deadline){const s=await status();if(predicate(s))return s;await page.waitForTimeout(60);}
  throw new Error(`IMU timeout: ${JSON.stringify(await status())}`);
 };
 try {
  await page.goto('http://127.0.0.1:8767/');
  await wait(s=>s.online && s.imu.streaming);
  await action('reset');await action('play');
  await wait(s=>s.imu.streaming&&s.imu.heading_calibrated);
  await page.keyboard.down('w');await page.waitForTimeout(600);await page.keyboard.down('d');await page.waitForTimeout(1400);
  await page.keyboard.up('d');await page.keyboard.up('w');
  const turned=await wait(s=>!s.driving&&s.imu.heading_stationary);
  assert(turned.imu.truth_heading_deg>90&&turned.imu.truth_heading_deg<180,'No forward/right arc');
  assert(Math.abs(turned.imu.heading_error_deg)<2,'Noisy IMU heading diverged');
  await page.waitForTimeout(250);
  assert(!await page.locator('#imu-heading').textContent().then(t=>t.includes('—')),'IMU heading is hidden');
  if(process.env.IMU_SCREENSHOT)await page.screenshot({path:process.env.IMU_SCREENSHOT,fullPage:true});
  await action('imu-zero');assert.equal((await status()).imu.heading_calibrated,false);
  const zeroed=await wait(s=>s.imu.heading_calibrated);
  assert(Math.abs(zeroed.imu.heading_error_deg)<.5);
  await action('pause');await page.waitForTimeout(250);
  const stamp=(await status()).imu.t_sim_s;await page.waitForTimeout(350);
  assert.equal((await status()).imu.t_sim_s,stamp,'Paused IMU advances');
  await action('reset');await action('play');
  const reset=await wait(s=>s.imu.heading_calibrated&&s.imu.epoch>turned.imu.epoch);
  assert(Math.abs(reset.imu.heading_error_deg)<.1);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({turn_heading_deg:turned.imu.heading_deg,truth_heading_deg:turned.imu.truth_heading_deg,error_deg:turned.imu.heading_error_deg,zeroed:true,pause:true,reset:true}));
  console.log('PASS: noisy IMU visible, real keyboard W+D, heading comparison, zero, pause and reset');
 } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
