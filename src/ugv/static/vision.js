const feed = document.querySelector("#camera-feed");
const overlay = document.querySelector("#vision-overlay");
const mediaLayer = document.querySelector("#media-layer");
const detectButton = document.querySelector("#detect-toggle");
const stabilizeButton = document.querySelector("#stabilize-toggle");
const ctx = overlay.getContext("2d");
const inputCanvas = document.createElement("canvas");
const inputCtx = inputCanvas.getContext("2d", { willReadFrequently: true });
const motionCanvas = document.createElement("canvas");
const motionCtx = motionCanvas.getContext("2d", { willReadFrequently: true });
inputCanvas.width = inputCanvas.height = 320;
motionCanvas.width = 64; motionCanvas.height = 48;

const labels = ["person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"];

let detecting = false, stabilizing = false, session = null, busy = false, previousGray = null;
let offsetX = 0, offsetY = 0;

function resizeOverlay() {
  const ratio = devicePixelRatio || 1;
  overlay.width = Math.round(overlay.clientWidth * ratio);
  overlay.height = Math.round(overlay.clientHeight * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}
new ResizeObserver(resizeOverlay).observe(overlay);

async function loadModel() {
  if (session) return session;
  detectButton.textContent = "LOADING…";
  ort.env.wasm.wasmPaths = "/static/vendor/";
  ort.env.wasm.numThreads = crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 1) : 1;
  session = await ort.InferenceSession.create("/static/models/yolo11n-320.onnx", {
    executionProviders: ["wasm"], graphOptimizationLevel: "all"
  });
  return session;
}

detectButton.addEventListener("click", async () => {
  detecting = !detecting;
  detectButton.setAttribute("aria-pressed", String(detecting));
  if (!detecting) { detectButton.textContent = "DETECT OFF"; clearBoxes(); return; }
  try { await loadModel(); detectButton.textContent = "DETECT ON"; }
  catch (error) { detecting = false; detectButton.setAttribute("aria-pressed", "false"); detectButton.textContent = "DETECT ERROR"; console.error(error); }
});

stabilizeButton.addEventListener("click", () => {
  stabilizing = !stabilizing; previousGray = null; offsetX = offsetY = 0;
  stabilizeButton.setAttribute("aria-pressed", String(stabilizing));
  stabilizeButton.textContent = stabilizing ? "STABILIZE ON" : "STABILIZE OFF";
  if (!stabilizing) mediaLayer.style.transform = "";
});

function clearBoxes() { ctx.clearRect(0, 0, overlay.clientWidth, overlay.clientHeight); }

function contentRect() {
  const w = overlay.clientWidth, h = overlay.clientHeight;
  const ratio = Math.min(w / (feed.naturalWidth || 640), h / (feed.naturalHeight || 480));
  const cw = (feed.naturalWidth || 640) * ratio, ch = (feed.naturalHeight || 480) * ratio;
  return { x: (w - cw) / 2, y: (h - ch) / 2, w: cw, h: ch };
}

function iou(a, b) {
  const x1 = Math.max(a.x1,b.x1), y1 = Math.max(a.y1,b.y1), x2 = Math.min(a.x2,b.x2), y2 = Math.min(a.y2,b.y2);
  const intersection = Math.max(0,x2-x1)*Math.max(0,y2-y1);
  return intersection / ((a.x2-a.x1)*(a.y2-a.y1)+(b.x2-b.x1)*(b.y2-b.y1)-intersection+1e-6);
}

function decode(data) {
  const count = 2100, boxes = [];
  for (let i=0; i<count; i++) {
    let score=0, cls=0;
    for (let c=0; c<80; c++) { const value=data[(4+c)*count+i]; if (value>score) { score=value; cls=c; } }
    if (score < .38) continue;
    const cx=data[i], cy=data[count+i], w=data[2*count+i], h=data[3*count+i];
    boxes.push({x1:cx-w/2,y1:cy-h/2,x2:cx+w/2,y2:cy+h/2,score,cls});
  }
  boxes.sort((a,b)=>b.score-a.score); const kept=[];
  for (const box of boxes) if (!kept.some(other=>other.cls===box.cls && iou(box,other)>.45)) { kept.push(box); if (kept.length>=30) break; }
  return kept;
}

function drawBoxes(boxes) {
  clearBoxes(); const rect=contentRect(); ctx.lineWidth=2; ctx.font="600 12px system-ui";
  for (const box of boxes) {
    const x=rect.x+box.x1/320*rect.w, y=rect.y+box.y1/320*rect.h, w=(box.x2-box.x1)/320*rect.w, h=(box.y2-box.y1)/320*rect.h;
    const text=`${labels[box.cls]} ${Math.round(box.score*100)}%`; const tw=ctx.measureText(text).width+10;
    ctx.strokeStyle="#1677b8"; ctx.fillStyle="#1677b8"; ctx.strokeRect(x,y,w,h); ctx.fillRect(x,y-20,tw,20);
    ctx.fillStyle="#fff"; ctx.fillText(text,x+5,y-6);
  }
}

async function detectFrame() {
  if (!detecting || !session || busy || !feed.complete || !feed.naturalWidth) return;
  busy=true;
  try {
    inputCtx.drawImage(feed,0,0,320,320); const rgba=inputCtx.getImageData(0,0,320,320).data;
    const chw=new Float32Array(3*320*320), plane=320*320;
    for (let i=0,j=0;i<rgba.length;i+=4,j++) { chw[j]=rgba[i]/255; chw[plane+j]=rgba[i+1]/255; chw[2*plane+j]=rgba[i+2]/255; }
    const result=await session.run({images:new ort.Tensor("float32",chw,[1,3,320,320])});
    drawBoxes(decode(result[session.outputNames[0]].data));
  } catch (error) { console.error(error); }
  finally { busy=false; }
}

function stabilizeFrame() {
  if (!stabilizing || !feed.complete || !feed.naturalWidth) return;
  motionCtx.drawImage(feed,0,0,64,48); const rgba=motionCtx.getImageData(0,0,64,48).data, gray=new Uint8Array(64*48);
  for (let i=0,j=0;i<rgba.length;i+=4,j++) gray[j]=(rgba[i]*3+rgba[i+1]*6+rgba[i+2])/10;
  if (previousGray) {
    let best=Infinity,bx=0,by=0;
    for (let dy=-3;dy<=3;dy++) for (let dx=-3;dx<=3;dx++) {
      let error=0;
      for (let y=6;y<42;y+=2) for (let x=8;x<56;x+=2) error+=Math.abs(gray[y*64+x]-previousGray[(y+dy)*64+x+dx]);
      if (error<best) { best=error; bx=dx; by=dy; }
    }
    offsetX=Math.max(-24,Math.min(24,offsetX*.82-bx*2.2)); offsetY=Math.max(-18,Math.min(18,offsetY*.82-by*2.2));
    mediaLayer.style.transform=`translate(${offsetX.toFixed(1)}px,${offsetY.toFixed(1)}px) scale(1.035)`;
  }
  previousGray=gray;
}

setInterval(detectFrame, 300);
setInterval(stabilizeFrame, 100);
