(() => {
  const panel = document.querySelector('#parking-panel');
  const canvas = document.querySelector('#parking-map');
  const context = canvas.getContext('2d');
  let path = [];
  document.querySelector('#map-toggle').onclick = () => { panel.hidden = !panel.hidden; if (!panel.hidden) send({type:"aruco",enabled:true}); };
  document.querySelector('#park-start').onclick = () => send({type:'parking',action:'start'});
  document.querySelector('#park-stop').onclick = () => send({type:'parking',action:'stop'});
  document.querySelector('#park-preview').onclick = async () => {
    try {
      const result = await (await fetch('/api/parking/plan')).json();
      path = result.path || [];
      document.querySelector('#parking-plan-result').textContent = result.ok ? 'Route checked against full footprint' : result.blockers.join('; ');
    } catch { document.querySelector('#parking-plan-result').textContent = 'Preview unavailable'; }
  };
  window.renderParking = (p, state) => {
    if (!p) return;
    document.querySelector('#parking-status').textContent = p.active ? p.execution_status : p.status;
    document.querySelector('#parking-detail').textContent = `Tags ${p.seen_tag_ids.join(', ') || '—'} · ${p.pose_source} · age ${p.pose_age_ms ?? '—'} ms\nEstimated position uncertainty ±${p.position_sigma_m == null ? '—' : Math.round(p.position_sigma_m*100)} cm\nScene: ${p.odometry.mapped_features} points / ${p.odometry.inliers} pose inliers`;
    document.querySelector('#parking-blockers').textContent = p.entry_ready ? 'Clearance and localization verified' : 'Entry needs measured car dimensions, gear clearance and calibrated camera/parking position.';
    document.querySelector('#parking-calibration-details').textContent = p.blockers.join('; ');
    document.querySelector('#park-start').disabled = !p.entry_ready || !state.you_are_owner || !state.armed || p.active;
    document.querySelector('#park-preview').disabled = !p.entry_ready;
    document.querySelector('#park-stop').disabled = !p.active || !state.you_are_owner;
    context.clearRect(0,0,canvas.width,canvas.height);
    const pose=p.car_pose || p.camera_pose;
    const points = [...p.tags.map(t=>[t.x_m,t.z_m]), ...(pose ? [[pose.x_m,pose.z_m]] : [])];
    const extent=Math.max(.5,...points.flatMap(q=>q.map(Math.abs)))+Math.max(.25,p.position_sigma_m || 0);
    const scale=canvas.height/(2*extent);
    const xy=(x,z)=>[canvas.width/2+x*scale,canvas.height/2-z*scale];
    context.strokeStyle='#cbd5e1';context.lineWidth=1;
    context.beginPath();context.moveTo(canvas.width/2,0);context.lineTo(canvas.width/2,canvas.height);context.moveTo(0,canvas.height/2);context.lineTo(canvas.width,canvas.height/2);context.stroke();
    context.font='11px system-ui';context.fillStyle='#475569';context.fillText(`${extent.toFixed(1)} m from origin`,8,14);
    // Gear and route are only drawn in a calibrated drone/body frame.
    if(p.car_pose) {
      const g=p.gear;context.strokeStyle='#334155';context.lineWidth=Math.max(2,g.bar_radius_m*2*scale);
      [-1,1].forEach(sign=>{const x=sign*(g.clear_width_m/2+g.bar_radius_m);context.beginPath();context.moveTo(...xy(x,-g.bar_length_m/2));context.lineTo(...xy(x,g.bar_length_m/2));context.stroke();});
      context.strokeStyle='#059669';context.lineWidth=2;context.beginPath();path.forEach((q,i)=>context[i?'lineTo':'moveTo'](...xy(q.x,q.z)));context.stroke();
    } else p.tags.forEach(t=>{const [x,y]=xy(t.x_m,t.z_m);context.fillStyle='#e11d48';context.fillRect(x-4,y-4,8,8);context.fillText(`Tag ${t.id}`,x+7,y);});
    if(pose) {
      const [x,y]=xy(pose.x_m,pose.z_m);context.fillStyle='#2563eb';context.beginPath();context.arc(x,y,5,0,Math.PI*2);context.fill();
      if(p.position_sigma_m!=null){context.strokeStyle='#93c5fd';context.lineWidth=1;context.beginPath();context.arc(x,y,p.position_sigma_m*scale,0,Math.PI*2);context.stroke();}
      context.fillText(p.car_pose?'Car centre':'Camera estimate',x+8,y-8);
    }
    document.querySelector('#parking-reference').textContent = p.car_pose ? 'Drone frame · full car footprint planning' : p.map_reference;
  };
})();
