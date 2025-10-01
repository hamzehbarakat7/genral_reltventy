/* ====== ثوابت وحالة عامة محسّنة ====== */
const C = 299792458, G = 6.67430e-11, SOFT = 1e6;
let bodies = [], time = 0, running = false, currentDt = 0;
const state = {
  dt: 1, minDt: 1e-5, maxDt: 20,
  adaptive: true, pn: false, speedMul: 1,
  collision: 'merge', trails: true, showGrid: true,
  relTol: 1e-5, absTol: 1e-2,
  tidalForces: false,
  pn2: false,
  showDistances: false,
  gravWaves: false // موجات الجاذبية (فقدان الطاقة)
};

const trackState = {
  mode: 'free',
  bodyIndex: -1,
  group: new Set(),
};

const view = document.getElementById('view');
const ctx = view.getContext('2d', { alpha: false });
const hudStats = document.getElementById('stats');
const chart = document.getElementById('chart');
const cctx = chart.getContext('2d');
const cardsRoot = document.getElementById('bodyCards');
const q = id => document.getElementById(id);

/* ====== فيزياء محسّنة بدقة عالية ====== */
function computeAccelerations(usePN, useTidal) {
  const n = bodies.length;
  const ax = new Float64Array(n), ay = new Float64Array(n), az = new Float64Array(n);
  
  // حساب نيوتوني محسّن مع softening ديناميكي
  for (let i = 0; i < n; i++) {
    const bi = bodies[i];
    for (let j = i + 1; j < n; j++) {
      const bj = bodies[j];
      const dx = bj.x - bi.x, dy = bj.y - bi.y, dz = bj.z - bi.z;
      const r2 = dx*dx + dy*dy + dz*dz;
      
      // softening يعتمد على نصف القطر الفعلي
      const radiusSum = (bi.radius || 0) + (bj.radius || 0);
      const softening = Math.max(SOFT, radiusSum * 0.05);
      const r2_soft = r2 + softening*softening;
      const r = Math.sqrt(r2_soft);
      const invr3 = 1.0 / (r2_soft * r);
      
      // قوة الجاذبية
      const f = G * bi.mass * bj.mass * invr3;
      const fx = f * dx, fy = f * dy, fz = f * dz;
      
      ax[i] += fx / bi.mass; ay[i] += fy / bi.mass; az[i] += fz / bi.mass;
      ax[j] -= fx / bj.mass; ay[j] -= fy / bj.mass; az[j] -= fz / bj.mass;
      
      // قوى المد والجزر (tidal forces)
      if (useTidal && radiusSum > 0) {
        const r_actual = Math.sqrt(r2);
        const rocheLimit = 2.44 * Math.max(bi.radius, bj.radius) * Math.cbrt(Math.max(bi.mass, bj.mass) / Math.min(bi.mass, bj.mass));
        
        if (r_actual > 0 && r_actual < rocheLimit * 2) {
          const tidal_strength = 2 * G * bi.mass * bj.mass / Math.pow(r_actual, 4);
          const tidal_i = tidal_strength * (bi.radius || 0) / bi.mass;
          const tidal_j = tidal_strength * (bj.radius || 0) / bj.mass;
          
          ax[i] -= tidal_i * dx / r_actual;
          ay[i] -= tidal_i * dy / r_actual;
          az[i] -= tidal_i * dz / r_actual;
          
          ax[j] += tidal_j * dx / r_actual;
          ay[j] += tidal_j * dy / r_actual;
          az[j] += tidal_j * dz / r_actual;
        }
      }
    }
  }
  
  // تصحيحات ما بعد-نيوتونية (1PN + 2PN)
  if (usePN && n >= 2) {
    let ci = 0;
    for (let k = 1; k < n; k++) if (bodies[k].mass > bodies[ci].mass) ci = k;
    const Cb = bodies[ci];
    
    for (let i = 0; i < n; i++) {
      if (i === ci) continue;
      const b = bodies[i];
      const rx = b.x - Cb.x, ry = b.y - Cb.y, rz = b.z - Cb.z;
      const vx = b.vx - Cb.vx, vy = b.vy - Cb.vy, vz = b.vz - Cb.vz;
      const r2 = rx*rx + ry*ry + rz*rz;
      const r = Math.max(1e-9, Math.sqrt(r2));
      const rhatx = rx / r, rhaty = ry / r, rhatz = rz / r;
      const v2rel = vx*vx + vy*vy + vz*vz;
      const vr = vx*rhatx + vy*rhaty + vz*rhatz;
      const GM = G * Cb.mass;
      const C2 = C * C;
      
      // 1PN: تصحيح شوارتزشيلد
      const coef = -GM / (r*r);
      const term1PN = (4 * GM) / (r * C2) - (v2rel / C2);
      const acc1PN_x = coef * (term1PN * rhatx + (4 * vr / C2) * vx);
      const acc1PN_y = coef * (term1PN * rhaty + (4 * vr / C2) * vy);
      const acc1PN_z = coef * (term1PN * rhatz + (4 * vr / C2) * vz);
      
      ax[i] += acc1PN_x;
      ay[i] += acc1PN_y;
      az[i] += acc1PN_z;
      
      // 2PN: تصحيحات أعلى (للأنظمة عالية الكتلة/قريبة)
      if (state.pn2 && v2rel < C2 * 0.05) {
        const beta = GM / (r * C2);
        const gamma = v2rel / C2;
        
        const term2PN = (GM / (r * C2 * C2)) * (
          -2 * beta * (3 + gamma - 4*vr*vr/C2) +
          3.5 * gamma * gamma
        );
        
        ax[i] += term2PN * rhatx;
        ay[i] += term2PN * rhaty;
        az[i] += term2PN * rhatz;
      }
      
      // إشعاع موجات الجاذبية (فقدان طاقة تقريبي)
      if (state.gravWaves && r < 1e12) { // فقط للأنظمة القريبة جداً
        const mu = (b.mass * Cb.mass) / (b.mass + Cb.mass);
        const power = (32 * G**4 * mu * mu * (b.mass + Cb.mass)**2) / (5 * C**5 * r**5);
        const drag_factor = power / (mu * v2rel + 1e-20);
        
        ax[i] -= drag_factor * vx;
        ay[i] -= drag_factor * vy;
        az[i] -= drag_factor * vz;
      }
    }
  }
  
  return { ax, ay, az };
}

/* RK4 محسّن */
function rk4Step(dt, usePN) {
  const n = bodies.length;
  const x0 = new Float64Array(n), y0 = new Float64Array(n), z0 = new Float64Array(n);
  const vx0 = new Float64Array(n), vy0 = new Float64Array(n), vz0 = new Float64Array(n);
  
  for (let i = 0; i < n; i++) {
    const b = bodies[i];
    x0[i]=b.x; y0[i]=b.y; z0[i]=b.z;
    vx0[i]=b.vx; vy0[i]=b.vy; vz0[i]=b.vz;
  }
  
  // K1
  let { ax: ax1, ay: ay1, az: az1 } = computeAccelerations(usePN, state.tidalForces);
  const k1x = vx0.slice(0), k1y = vy0.slice(0), k1z = vz0.slice(0);
  const k1vx = ax1.slice(0), k1vy = ay1.slice(0), k1vz = az1.slice(0);

  // K2
  for (let i=0;i<n;i++){
    bodies[i].x = x0[i] + 0.5*dt*k1x[i];
    bodies[i].y = y0[i] + 0.5*dt*k1y[i];
    bodies[i].z = z0[i] + 0.5*dt*k1z[i];
    bodies[i].vx = vx0[i] + 0.5*dt*k1vx[i];
    bodies[i].vy = vy0[i] + 0.5*dt*k1vy[i];
    bodies[i].vz = vz0[i] + 0.5*dt*k1vz[i];
  }
  let { ax: ax2, ay: ay2, az: az2 } = computeAccelerations(usePN, state.tidalForces);
  const k2x = bodies.map(b=>b.vx), k2y = bodies.map(b=>b.vy), k2z = bodies.map(b=>b.vz);
  const k2vx = ax2.slice(0), k2vy = ay2.slice(0), k2vz = az2.slice(0);

  // K3
  for (let i=0;i<n;i++){
    bodies[i].x = x0[i] + 0.5*dt*k2x[i];
    bodies[i].y = y0[i] + 0.5*dt*k2y[i];
    bodies[i].z = z0[i] + 0.5*dt*k2z[i];
    bodies[i].vx = vx0[i] + 0.5*dt*k2vx[i];
    bodies[i].vy = vy0[i] + 0.5*dt*k2vy[i];
    bodies[i].vz = vz0[i] + 0.5*dt*k2vz[i];
  }
  let { ax: ax3, ay: ay3, az: az3 } = computeAccelerations(usePN, state.tidalForces);
  const k3x = bodies.map(b=>b.vx), k3y = bodies.map(b=>b.vy), k3z = bodies.map(b=>b.vz);
  const k3vx = ax3.slice(0), k3vy = ay3.slice(0), k3vz = az3.slice(0);

  // K4
  for (let i=0;i<n;i++){
    bodies[i].x = x0[i] + dt*k3x[i];
    bodies[i].y = y0[i] + dt*k3y[i];
    bodies[i].z = z0[i] + dt*k3z[i];
    bodies[i].vx = vx0[i] + dt*k3vx[i];
    bodies[i].vy = vy0[i] + dt*k3vy[i];
    bodies[i].vz = vz0[i] + dt*k3vz[i];
  }
  let { ax: ax4, ay: ay4, az: az4 } = computeAccelerations(usePN, state.tidalForces);
  const k4x = bodies.map(b=>b.vx), k4y = bodies.map(b=>b.vy), k4z = bodies.map(b=>b.vz);
  const k4vx = ax4.slice(0), k4vy = ay4.slice(0), k4vz = az4.slice(0);

  // التحديث النهائي
  for (let i=0;i<n;i++){
    bodies[i].x = x0[i] + dt * (k1x[i] + 2*k2x[i] + 2*k3x[i] + k4x[i]) / 6;
    bodies[i].y = y0[i] + dt * (k1y[i] + 2*k2y[i] + 2*k3y[i] + k4y[i]) / 6;
    bodies[i].z = z0[i] + dt * (k1z[i] + 2*k2z[i] + 2*k3z[i] + k4z[i]) / 6;
    bodies[i].vx = vx0[i] + dt * (k1vx[i] + 2*k2vx[i] + 2*k3vx[i] + k4vx[i]) / 6;
    bodies[i].vy = vy0[i] + dt * (k1vy[i] + 2*k2vy[i] + 2*k3vy[i] + k4vy[i]) / 6;
    bodies[i].vz = vz0[i] + dt * (k1vz[i] + 2*k2vz[i] + 2*k3vz[i] + k4vz[i]) / 6;
  }
}

/* تكيف زمني محسّن */
function tryAdvance(dt, usePN) {
  const saved = bodies.map(b => ({ x:b.x,y:b.y,z:b.z,vx:b.vx,vy:b.vy,vz:b.vz }));
  
  rk4Step(dt, usePN);
  const oneStep = bodies.map(b => ({ x:b.x,y:b.y,z:b.z }));
  
  for (let i=0;i<bodies.length;i++) Object.assign(bodies[i], saved[i]);
  rk4Step(dt/2, usePN);
  rk4Step(dt/2, usePN);
  const twoHalf = bodies.map(b => ({ x:b.x,y:b.y,z:b.z }));
  
  let maxErr = 0;
  for (let i=0;i<bodies.length;i++){
    const a=oneStep[i], b=twoHalf[i];
    const ex=Math.abs(a.x-b.x), ey=Math.abs(a.y-b.y), ez=Math.abs(a.z-b.z);
    const scale = state.absTol + Math.max(Math.abs(a.x),Math.abs(b.x)) * state.relTol;
    maxErr = Math.max(maxErr, ex/scale, ey/scale, ez/scale);
  }
  
  if (maxErr > 1) {
    for (let i=0;i<bodies.length;i++) Object.assign(bodies[i], saved[i]);
    return {
      accepted: false,
      suggested: clamp(dt * Math.max(0.2, 0.85/Math.pow(maxErr, 0.2)), state.minDt, state.maxDt)
    };
  }
  
  return {
    accepted: true,
    suggested: clamp(dt * Math.min(2.0, 0.9/Math.pow(maxErr+1e-12, 0.2)), state.minDt, state.maxDt)
  };
}

function integrateAdaptive(baseDt) {
  let dt = baseDt * state.speedMul;
  if (!state.adaptive) {
    rk4Step(dt, state.pn);
    time += dt;
    currentDt = dt;
    return dt;
  }
  
  let ok=false, trial=dt, newDt=dt, guard=0;
  while (!ok && guard < 10) {
    const r = tryAdvance(trial, state.pn);
    if (r.accepted) {
      ok = true;
      newDt = r.suggested;
    } else {
      trial = r.suggested;
    }
    guard++;
  }
  
  time += trial;
  currentDt = trial;
  state.dt = clamp(newDt, state.minDt, state.maxDt);
  if (q('dt')) q('dt').value = state.dt.toFixed(4);
  return trial;
}

/* تصادمات محسّنة مع حفظ الزخم */
function handleCollisions() {
  if (state.collision !== 'merge') return;
  const removed = [];
  
  for (let i=0;i<bodies.length;i++){
    for (let j=i+1;j<bodies.length;j++){
      const bi=bodies[i], bj=bodies[j];
      const dx = bj.x-bi.x, dy = bj.y-bi.y, dz = bj.z-bi.z;
      const r = Math.sqrt(dx*dx + dy*dy + dz*dz);
      const Rsum = (Number(bi.radius)||0) + (Number(bj.radius)||0);
      
      if (r > 0 && r <= Math.max(1e3, Rsum * 1.05)) {
        const M = bi.mass + bj.mass;
        
        // حفظ الزخم الخطي
        const px = bi.vx*bi.mass + bj.vx*bj.mass;
        const py = bi.vy*bi.mass + bj.vy*bj.mass;
        const pz = bi.vz*bi.mass + bj.vz*bj.mass;
        
        // حفظ مركز الكتلة
        bi.x = (bi.x*bi.mass + bj.x*bj.mass) / M;
        bi.y = (bi.y*bi.mass + bj.y*bj.mass) / M;
        bi.z = (bi.z*bi.mass + bj.z*bj.mass) / M;
        
        bi.vx = px / M;
        bi.vy = py / M;
        bi.vz = pz / M;
        
        bi.mass = M;
        
        // حساب نصف القطر الجديد (حفظ الحجم)
        const r1 = Math.max(1, Number(bi.radius)||1);
        const r2 = Math.max(1, Number(bj.radius)||1);
        bi.radius = Math.cbrt(r1**3 + r2**3);
        
        bi.name = bi.name + "⊕" + bj.name;
        bi.trail = bi.trail.concat(bj.trail.slice(-100)).slice(-400);
        
        removed.push(j);
      }
    }
  }
  
  removed.sort((a,b)=>b-a).forEach(idx=>bodies.splice(idx,1));
  if (removed.length) refreshTrackList();
}

/* طاقة وزخم محسّنة */
function energyAndAngMom() {
  let KE=0, PE=0, Lx=0, Ly=0, Lz=0;
  
  for (const b of bodies) {
    KE += 0.5 * b.mass * v2(b.vx,b.vy,b.vz);
  }
  
  for (let i=0;i<bodies.length;i++){
    for (let j=i+1;j<bodies.length;j++){
      const dx = bodies[j].x-bodies[i].x;
      const dy = bodies[j].y-bodies[i].y;
      const dz = bodies[j].z-bodies[i].z;
      const r2 = dx*dx + dy*dy + dz*dz;
      const radiusSum = (bodies[i].radius || 0) + (bodies[j].radius || 0);
      const softening = Math.max(SOFT, radiusSum * 0.05);
      const r = Math.sqrt(r2 + softening*softening);
      PE += -G * bodies[i].mass * bodies[j].mass / r;
    }
  }
  
  for (const b of bodies) {
    Lx += b.mass * (b.y * b.vz - b.z * b.vy);
    Ly += b.mass * (b.z * b.vx - b.x * b.vz);
    Lz += b.mass * (b.x * b.vy - b.y * b.vx);
  }
  
  const L_total = Math.sqrt(Lx*Lx + Ly*Ly + Lz*Lz);
  
  return { E: KE+PE, KE, PE, Lx, Ly, Lz, L_total };
}

/* تحليل الاستقرار المداري */
function analyzeOrbitalStability() {
  if (bodies.length < 2) return null;
  
  let minDistance = Infinity;
  let maxEccentricity = 0;
  let closestPair = null;
  let avgSeparation = 0;
  let count = 0;
  
  for (let i = 0; i < bodies.length; i++) {
    for (let j = i + 1; j < bodies.length; j++) {
      const bi = bodies[i], bj = bodies[j];
      const dx = bj.x - bi.x, dy = bj.y - bi.y, dz = bj.z - bi.z;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
      
      avgSeparation += dist;
      count++;
      
      if (dist < minDistance) {
        minDistance = dist;
        closestPair = [i, j];
      }
      
      if (bi.e) maxEccentricity = Math.max(maxEccentricity, bi.e);
      if (bj.e) maxEccentricity = Math.max(maxEccentricity, bj.e);
    }
  }
  
  avgSeparation = count > 0 ? avgSeparation / count : 0;
  
  return {
    minDistance,
    avgSeparation,
    maxEccentricity,
    closestPair,
    isStable: maxEccentricity < 0.9 && minDistance > SOFT * 100
  };
}

/* مدار نسبي محسّن */
function computeRelativisticOrbitForBody(bodyIndex) {
  if (bodies.length < 2) return;
  let ci = 0;
  for (let k = 1; k < bodies.length; k++) if (bodies[k].mass > bodies[ci].mass) ci = k;
  if (bodyIndex === ci) return;

  const b = bodies[bodyIndex], Cb = bodies[ci];
  const rx = b.x - Cb.x, ry = b.y - Cb.y, rz = b.z - Cb.z;
  const vx = b.vx - Cb.vx, vy = b.vy - Cb.vy, vz = b.vz - Cb.vz;
  const r = mag(rx,ry,rz);
  const speed2 = v2(vx,vy,vz);
  const mu = G * (Cb.mass + b.mass);

  const hx = ry*vz - rz*vy, hy = rz*vx - rx*vz, hz = rx*vy - ry*vx;
  const h2 = hx*hx + hy*hy + hz*hz;
  const E = speed2/2 - mu/r;

  let a = 0;
  if (E < 0) a = -mu/(2*E);
  else if (Math.abs(E) < 1e-12) a = Infinity;
  else a = mu/(2*E);

  let e;
  if (E < 0) e = Math.sqrt(Math.max(0, 1 + 2*E*h2/(mu*mu)));
  else if (Math.abs(E) < 1e-12) e = 1;
  else e = Math.sqrt(Math.max(0, 1 + 2*E*h2/(mu*mu)));

  let deltaPhiRad = 0;
  if (E < 0 && e < 1 && isFinite(a)) {
    const GM = G * Cb.mass;
    const one_minus_e2 = Math.max(1e-12, 1 - e*e);
    const p = a * one_minus_e2;
    deltaPhiRad = (6 * Math.PI * GM) / (C*C * p);
    const P = 2 * Math.PI * Math.sqrt(a*a*a / mu);
    const secPerCentury = 100 * 365.25 * 24 * 3600;
    const arcsecPerRad = 180 / Math.PI * 3600;
    const precessionArcsecPerCentury = (deltaPhiRad / P) * secPerCentury * arcsecPerRad;
    b.precession = isFinite(precessionArcsecPerCentury) ? precessionArcsecPerCentury : 0;
  } else {
    b.precession = 0;
  }
  
  b.a = a;
  b.e = e;
  b.centerBody = Cb.name;
  b.period = isFinite(a) && a > 0 ? 2 * Math.PI * Math.sqrt(a*a*a / mu) : 0;
}

/* بطاقات محسّنة */
function refreshCards() {
  if (!cardsRoot) return;
  cardsRoot.innerHTML = '';
  bodies.forEach((b,i)=>{
    const card = document.createElement('div');
    card.className = 'card' + (b.selected ? ' selected' : '');
    const km = isFinite(b.radius) ? (b.radius/1000).toFixed(0) : '—';
    const period = b.period ? (b.period/86400).toFixed(2) + ' يوم' : '—';
    
    card.innerHTML = `
      <h4>${b.name}
        <span class="badgeSmall" style="background:${b.color};color:#000;border-color:rgba(0,0,0,0.2)">${km} km</span>
      </h4>
      <div class="meta">
        <span>كتلة:</span> <span>${b.mass.toExponential(2)} kg</span>
        <span>موقع:</span> <span>(${(b.x/1e9).toFixed(1)}, ${(b.y/1e9).toFixed(1)}, ${(b.z/1e9).toFixed(1)}) Gm</span>
        <span>سرعة:</span> <span>${mag(b.vx,b.vy,b.vz).toFixed(1)} m/s</span>
      </div>
      ${b.precession ? `
      <div class="meta" style="margin-top:4px">
        <span>مرجع:</span> <span>${b.centerBody}</span>
        <span>a:</span> <span>${isFinite(b.a)?(b.a/1e9).toFixed(2):'—'} Gm</span>
        <span>e:</span> <span>${isFinite(b.e)?b.e.toFixed(4):'—'}</span>
        <span>دورة:</span> <span>${period}</span>
        <span style="color:var(--accent)">تقدّم:</span> <span style="color:var(--accent)">${b.precession.toFixed(3)} "/قرن</span>
      </div>`: '' }
      <div class="rowBtns">
        <button onclick="selectBody(${i})">${b.selected ? '✓ محدد' : 'تحديد'}</button>
        <button onclick="toggleTrackGroup(${i})" style="background:${trackState.group.has(i)?'#2b9348':'#1f2a3d'}">
          🎯 ${trackState.group.has(i)?'مجموعة':'أضِف'}
        </button>
        <button onclick="removeBody(${i})" style="background:var(--bad)">حذف</button>
      </div>
    `;
    cardsRoot.appendChild(card);
  });
}

function selectBody(index){ bodies.forEach((b,i)=>b.selected=(i===index)); refreshCards(); }
function removeBody(index){ bodies.splice(index,1); refreshCards(); refreshTrackList(); }
function toggleTrackGroup(i){
  if (trackState.group.has(i)) trackState.group.delete(i);
  else trackState.group.add(i);
  refreshCards();
}

/* مراكز الكتلة */
function getCenterOfMass(){
  if (!bodies.length) return {x:0,y:0,z:0};
  let m=0,x=0,y=0,z=0;
  for (const b of bodies){ m+=b.mass; x+=b.mass*b.x; y+=b.mass*b.y; z+=b.mass*b.z; }
  return m>0?{x:x/m,y:y/m,z:z/m}:{x:0,y:0,z:0};
}

function getGroupCenter(){
  if (trackState.group.size === 0) return null;
  let M=0, x=0, y=0, z=0;
  for (const i of trackState.group){
    const b = bodies[i]; if (!b) continue;
    const m = Math.max(1, b.mass);
    M+=m; x+=m*b.x; y+=m*b.y; z+=m*b.z;
  }
  if (M===0) return null;
  return {x:x/M, y:y/M, z:z/M};
}

function getTrackOffset(){
  switch(trackState.mode){
    case 'body': {
      const i = trackState.bodyIndex;
      const b = (i>=0 && bodies[i]) ? bodies[i] : null;
      return b ? {x:b.x, y:b.y, z:b.z} : {x:0,y:0,z:0};
    }
    case 'group': {
      const g = getGroupCenter();
      return g ? g : {x:0,y:0,z:0};
    }
    case 'cm': return getCenterOfMass();
    case 'free':
    default: return {x:0,y:0,z:0};
  }
}

function heaviestIndex(){
  if (!bodies.length) return -1;
  let hi = 0;
  for (let i=1;i<bodies.length;i++) if (bodies[i].mass > bodies[hi].mass) hi = i;
  return hi;
}

function getTotalMomentum(){
  let Px=0, Py=0, Pz=0;
  for (const b of bodies){ Px += b.mass*b.vx; Py += b.mass*b.vy; Pz += b.mass*b.vz; }
  return {Px,Py,Pz};
}

function reframeToBarycentric(){
  if (!bodies.length) return;
  let M=0, cx=0, cy=0, cz=0, cvx=0, cvy=0, cvz=0;
  for (const b of bodies){
    M += b.mass;
    cx += b.mass*b.x; cy += b.mass*b.y; cz += b.mass*b.z;
    cvx += b.mass*b.vx; cvy += b.mass*b.vy; cvz += b.mass*b.vz;
  }
  if (M <= 0) return;
  cx/=M; cy/=M; cz/=M; cvx/=M; cvy/=M; cvz/=M;
  for (const b of bodies){
    b.x -= cx; b.y -= cy; b.z -= cz;
    b.vx -= cvx; b.vy -= cvy; b.vz -= cvz;
  }
  refreshCards(); refreshTrackList(); updateCamera();
}

function anchorToHeaviest(){
  const hi = heaviestIndex();
  if (hi < 0) return;
  const H = bodies[hi];
  for (const b of bodies){
    b.x -= H.x; b.y -= H.y; b.z -= H.z;
    b.vx -= H.vx; b.vy -= H.vy; b.vz -= H.vz;
  }
  refreshCards(); refreshTrackList(); updateCamera();
}

function retuneCircularAroundHeaviest(){
  const hi = heaviestIndex();
  if (hi < 0 || bodies.length < 2) return;
  const H = bodies[hi];
  for (let i=0;i<bodies.length;i++){
    if (i === hi) continue;
    const b = bodies[i];
    const rx = b.x - H.x, ry = b.y - H.y, rz = b.z - H.z;
    const r = Math.hypot(rx, ry, rz);
    if (r < 1) continue;
    const v_circ = Math.sqrt(G * H.mass / r);

    let vx = b.vx - H.vx, vy = b.vy - H.vy, vz = b.vz - H.vz;
    const rhatx = rx/r, rhaty = ry/r, rhatz = rz/r;
    const vdotr = vx*rhatx + vy*rhaty + vz*rhatz;
    let vpx = vx - vdotr*rhatx, vpy = vy - vdotr*rhaty, vpz = vz - vdotr*rhatz;
    let vp = Math.hypot(vpx,vpy,vpz);

    if (vp < 1e-9){
      const nx = -ry, ny = rx, nz = 0;
      const nlen = Math.hypot(nx,ny,nz) || 1;
      vpx = nx/nlen; vpy = ny/nlen; vpz = nz/nlen; vp = 1;
    } else {
      vpx /= vp; vpy /= vp; vpz /= vp;
    }
    vx = vpx * v_circ; vy = vpy * v_circ; vz = vpz * v_circ;
    b.vx = H.vx + vx; b.vy = H.vy + vy; b.vz = H.vz + vz;
  }
  refreshCards();
}

/* رسم محسّن */
function drawGrid(ctx){
  if (!state.showGrid) return;
  const center = getTrackOffset();
  const base = cam.dist * 0.5;
  const gridSize = Math.pow(10, Math.floor(Math.log10(Math.max(1, base))));
  const fine = gridSize / 5;

  const offX = ((center.x % gridSize) + gridSize) % gridSize;
  const offY = ((center.y % gridSize) + gridSize) % gridSize;
  const N = 50;

  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(160, 180, 210, .08)';
  for (let i=-N;i<=N;i++){
    const xw = center.x - offX + i*fine;
    const yw = center.y - offY + i*fine;

    const p1 = project(xw, center.y - N*fine, center.z);
    const p2 = project(xw, center.y + N*fine, center.z);
    if (p1 && p2){ ctx.beginPath(); ctx.moveTo(p1[0],p1[1]); ctx.lineTo(p2[0],p2[1]); ctx.stroke(); }

    const p3 = project(center.x - N*fine, yw, center.z);
    const p4 = project(center.x + N*fine, yw, center.z);
    if (p3 && p4){ ctx.beginPath(); ctx.moveTo(p3[0],p3[1]); ctx.lineTo(p4[0],p4[1]); ctx.stroke(); }
  }

  ctx.lineWidth = 1.5;
  ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--grid') || '#1f2a3d';
  for (let i=-N;i<=N;i++){
    const xw = center.x - offX + i*gridSize;
    const yw = center.y - offY + i*gridSize;

    const p1 = project(xw, center.y - N*gridSize, center.z);
    const p2 = project(xw, center.y + N*gridSize, center.z);
    if (p1 && p2){ ctx.beginPath(); ctx.moveTo(p1[0],p1[1]); ctx.lineTo(p2[0],p2[1]); ctx.stroke(); }

    const p3 = project(center.x - N*gridSize, yw, center.z);
    const p4 = project(center.x + N*gridSize, yw, center.z);
    if (p3 && p4){ ctx.beginPath(); ctx.moveTo(p3[0],p3[1]); ctx.lineTo(p4[0],p4[1]); ctx.stroke(); }
  }
}

function drawAxes(){
  const c = getCenterOfMass();
  const L = cam.dist * 0.3;
  const axes = [
    {p:[c.x, c.y, c.z], q:[c.x+L, c.y, c.z], col:'#ff6666', label:'X'},
    {p:[c.x, c.y, c.z], q:[c.x, c.y+L, c.z], col:'#66ff66', label:'Y'},
    {p:[c.x, c.y, c.z], q:[c.x, c.y, c.z+L], col:'#66ccff', label:'Z'},
  ];
  ctx.lineWidth = 2.5;
  for (const a of axes){
    const P = project(...a.p), Q = project(...a.q);
    if (P && Q){
      ctx.strokeStyle = a.col;
      ctx.beginPath();
      ctx.moveTo(P[0],P[1]);
      ctx.lineTo(Q[0],Q[1]);
      ctx.stroke();
    }
  }
}

function draw(){
  ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--bg') || '#000';
  ctx.fillRect(0,0,view.width,view.height);
  drawGrid(ctx);
  drawAxes();

  const trackOffset = getTrackOffset();
  const sorted = bodies.map((b,i)=>({...b,__i:i}))
    .sort((A,B)=>{
      const pa = project(A.x - trackOffset.x, A.y-trackOffset.y, A.z-trackOffset.z);
      const pb = project(B.x - trackOffset.x, B.y-trackOffset.y, B.z-trackOffset.z);
      return ((pb?.[2]||0) - (pa?.[2]||0));
    });

  for (const b of sorted){
    const pos = project(b.x - trackOffset.x, b.y - trackOffset.y, b.z - trackOffset.z);
    if (!pos) continue;
    const [X,Y,z2,scale] = pos;
    const rDyn = Math.max(2.5, Math.cbrt(Math.max(1e0, b.mass)/1e20) * (cam.dist/Math.max(1,z2)) * 0.0006);
    const radius = isFinite(rDyn)? rDyn : 2.5;

    if (state.trails && b.trail && b.trail.length > 1){
      ctx.beginPath();
      ctx.strokeStyle = b.color;
      ctx.lineWidth = 1.2;
      ctx.globalAlpha = 0.7;
      let first = true;
      for (const p of b.trail){
        const t = project(p.x - trackOffset.x, p.y - trackOffset.y, p.z - trackOffset.z);
        if (!t) continue;
        if (first){ ctx.moveTo(t[0],t[1]); first=false; } else ctx.lineTo(t[0],t[1]);
      }
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    }

    ctx.fillStyle = b.color;
    ctx.beginPath();
    ctx.arc(X,Y,radius,0,Math.PI*2);
    if (isFinite(X) && isFinite(Y) && isFinite(radius)) ctx.fill();
    
    if (b.selected){
      ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent') || '#59c1ff';
      ctx.lineWidth = 2.5;
      ctx.stroke();
    }
  }
  
  drawMouseCoordinates(); // هذا السطر مهم
}

/* HUD محسّن */
let lastEnergy = 0, energyHistory = [], distanceHistory = [];
function updateStats(){
  const stats = energyAndAngMom();
  const { E, KE, PE, L_total } = stats;
  if (lastEnergy === 0) lastEnergy = E;
  const dE = Math.abs((E - lastEnergy) / (Math.abs(lastEnergy) + 1e-30)) * 1e2 || 0;
  
  const stability = analyzeOrbitalStability();

  hudStats.innerHTML = `
    <span>الوقت (س):</span> <span>${(time/3600).toFixed(2)}</span>
    <span>الطاقة الكلية:</span> <span>${E.toExponential(4)} J</span>
    <span>التغير (٪):</span> <span style="color:var(--${dE < 0.01 ? 'good' : dE < 0.1 ? 'warn' : 'bad'})">${dE.toFixed(6)} %</span>
    <span>|L| (زخم):</span> <span>${L_total.toExponential(4)}</span>
    <span>KE:</span> <span>${KE.toExponential(3)} J</span>
    <span>PE:</span> <span>${PE.toExponential(3)} J</span>
    <span>dt:</span> <span>${currentDt.toExponential(2)} ث</span>
    ${stability ? `
    <span>أقل مسافة:</span> <span style="color:var(--${stability.isStable?'good':'warn'})">${(stability.minDistance/1e9).toFixed(3)} Gm</span>
    <span>متوسط:</span> <span>${(stability.avgSeparation/1e9).toFixed(2)} Gm</span>
    <span>انحراف max:</span> <span>${stability.maxEccentricity.toFixed(4)}</span>
    ` : ''}
  `;
  
  energyHistory.push(dE);
  if (energyHistory.length>150) energyHistory.shift();
  
  if (stability) {
    distanceHistory.push(stability.minDistance / 1e9);
    if (distanceHistory.length > 150) distanceHistory.shift();
  }

  cctx.clearRect(0,0,chart.width,chart.height);
  const maxErr = Math.max(0.0001, ...energyHistory);
  const scaleY = chart.height * 0.85 / maxErr;
  
  cctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent') || '#59c1ff';
  cctx.lineWidth = 2;
  cctx.beginPath();
  const startX = Math.max(0, chart.width - energyHistory.length);
  cctx.moveTo(startX, chart.height - energyHistory[0] * scaleY);
  energyHistory.forEach((v,i)=>cctx.lineTo(startX+i, chart.height - v*scaleY));
  cctx.stroke();
  
  if (state.showDistances && distanceHistory.length > 1) {
    const maxDist = Math.max(1, ...distanceHistory);
    const scaleD = chart.height * 0.85 / maxDist;
    cctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--good') || '#7ee787';
    cctx.lineWidth = 1.8;
    cctx.globalAlpha = 0.65;
    cctx.beginPath();
    const startXd = Math.max(0, chart.width - distanceHistory.length);
    cctx.moveTo(startXd, chart.height - distanceHistory[0] * scaleD);
    distanceHistory.forEach((v,i)=>cctx.lineTo(startXd+i, chart.height - v*scaleD));
    cctx.stroke();
    cctx.globalAlpha = 1.0;
  }
}

/* حلقة التحديث */
function updateSimulation(){
  const t0 = performance.now();
  let steps = 0;
  while (running && (performance.now() - t0) < 12){
    const dt = +q('dt').value || 1;
    integrateAdaptive(dt);
    handleCollisions();
    bodies.forEach(b=>{
      b.trail.push({x:b.x,y:b.y,z:b.z});
      if (b.trail.length>600) b.trail.shift();
    });
    steps++;
  }
  if (steps>0) updateStats();
}

function loop(){ 
  if (running) updateSimulation();
  draw();
  requestAnimationFrame(loop);
}

/* مشاهد محسّنة */
const initialBodies = [
  { name:"Sun", mass:1.989e30, x:0,y:0,z:0, vx:0,vy:0,vz:0, radius:696340e3, color:'#f7f43e' },
  { name:"Mercury", mass:3.302e23, x:46.0e9,y:0,z:0, vx:0,vy:58980,vz:0, radius:2440e3, color:'#bdbdbd' },
  { name:"Venus", mass:4.867e24, x:108.2e9,y:0,z:0, vx:0,vy:35020,vz:0, radius:6052e3, color:'#ffc649' },
  { name:"Earth", mass:5.972e24, x:1.5e11,y:0,z:0, vx:0,vy:29780,vz:0, radius:6371e3, color:'#59c1ff' },
  { name:"Mars", mass:6.417e23, x:227.9e9,y:0,z:0, vx:0,vy:24070,vz:0, radius:3390e3, color:'#ff6b6b' }
];

function loadPreset(name){
  time=0; bodies=[]; energyHistory=[]; distanceHistory=[]; lastEnergy=0;
  let presetBodies=[];
  
  switch(name){
    case 'solar':
      presetBodies = initialBodies;
      break;
    case 'binary':
      presetBodies = [
        { name:"Star A", mass:2e30, x:-1e11,y:0,z:0, vx:0,vy:-15000,vz:0, radius:7e8, color:'#f7cc60' },
        { name:"Star B", mass:1e30, x:2e11,y:0,z:0, vx:0,vy:30000,vz:0, radius:5e8, color:'#c39df3' }
      ];
      break;
    case 'three':
      presetBodies = [
        { name:"Body 1", mass:1e27, x:1e10,y:0,z:0, vx:0,vy:15000,vz:0, radius:1e7, color:randColor() },
        { name:"Body 2", mass:1e27, x:-1e10,y:0,z:0, vx:0,vy:-15000,vz:0, radius:1e7, color:randColor() },
        { name:"Body 3", mass:1e27, x:0,y:1.5e10,z:0, vx:-15000,vy:0,vz:0, radius:1e7, color:randColor() }
      ];
      break;
    case 'bhStar':
      presetBodies = [
        { name:"Black Hole", mass:1e32, x:0,y:0,z:0, vx:0,vy:0,vz:0, radius:10e3, color:'#2b2b2b' },
        { name:"Star", mass:2e30, x:5e11,y:0,z:0, vx:0,vy:25000,vz:5000, radius:7e8, color:'#ff7b72' }
      ];
      q('enablePN').checked = true;
      state.pn = true;
      break;
    case 'random':
      for (let i=0;i<8;i++){
        presetBodies.push({
          name:"R-Body "+(i+1),
          mass: 5e24 + Math.random()*5e25,
          x:(Math.random()-0.5)*4e11,
          y:(Math.random()-0.5)*4e11,
          z:(Math.random()-0.5)*2e11,
          vx:(Math.random()-0.5)*15000,
          vy:(Math.random()-0.5)*15000,
          vz:(Math.random()-0.5)*8000,
          radius: 1e7 + Math.random()*5e7,
          color:randColor()
        });
      }
      break;
  }
  
  presetBodies.forEach(b=>{ b.trail=[]; bodies.push(b); });
  if (q('enablePN')) q('enablePN').checked = state.pn;
  
  refreshCards();
  refreshTrackList();
  updateCamera();
  reorbit();
  
  if (q('anchorHeavy')?.checked) anchorToHeaviest();
  else reframeToBarycentric();
}

function refreshTrackList(){
  const select = q('track');
  const prev = Number(select.value);
  select.innerHTML = '<option value="-1">المرجح (CM)</option>';
  bodies.forEach((b,i)=>{
    const o=document.createElement('option');
    o.value=i;
    o.textContent=b.name;
    select.appendChild(o);
  });
  select.value = (!isFinite(prev) || prev>=bodies.length) ? '-1' : String(prev);
  trackState.bodyIndex = Number(select.value);
}

function reorbit(){
  bodies.forEach((b,i)=>computeRelativisticOrbitForBody(i));
  refreshCards();
}

/* زر التحليل الشامل */
function performAnalysis(){
  const analysis = analyzeOrbitalStability();
  const energy = energyAndAngMom();
  const momentum = getTotalMomentum();
  
  let report = "=== تحليل النظام ===\n\n";
  report += `الوقت: ${(time/3600).toFixed(2)} ساعة\n`;
  report += `عدد الأجسام: ${bodies.length}\n\n`;
  
  report += "الطاقة:\n";
  report += `  الكلية: ${energy.E.toExponential(4)} J\n`;
  report += `  حركية: ${energy.KE.toExponential(4)} J\n`;
  report += `  وضع: ${energy.PE.toExponential(4)} J\n\n`;
  
  report += "الزخم الزاوي:\n";
  report += `  |L|: ${energy.L_total.toExponential(4)}\n`;
  report += `  Lx: ${energy.Lx.toExponential(3)}\n`;
  report += `  Ly: ${energy.Ly.toExponential(3)}\n`;
  report += `  Lz: ${energy.Lz.toExponential(3)}\n\n`;
  
  report += "الزخم الخطي:\n";
  report += `  Px: ${momentum.Px.toExponential(3)}\n`;
  report += `  Py: ${momentum.Py.toExponential(3)}\n`;
  report += `  Pz: ${momentum.Pz.toExponential(3)}\n\n`;
  
  if (analysis) {
    report += "الاستقرار المداري:\n";
    report += `  أقل مسافة: ${(analysis.minDistance/1e9).toFixed(3)} Gm\n`;
    report += `  متوسط المسافات: ${(analysis.avgSeparation/1e9).toFixed(2)} Gm\n`;
    report += `  أكبر انحراف: ${analysis.maxEccentricity.toFixed(4)}\n`;
    report += `  الحالة: ${analysis.isStable ? 'مستقر ✓' : 'غير مستقر ⚠'}\n`;
  }
  
  console.log(report);
  alert(report);
}

/* واجهة الأحداث */
function bindUI(){
  q('btnPlay').onclick = () => {
    running = !running;
    q('btnPlay').textContent = running ? '⏸ إيقاف' : '▶ تشغيل';
  };
  
  q('btnStep').onclick = () => {
    if (!running){
      integrateAdaptive(+q('dt').value || 1);
      handleCollisions();
      updateStats();
    }
  };
  
  q('btnReset').onclick = () => loadPreset(q('preset').value);
  
  q('speedMul').onchange = e => state.speedMul = Math.max(0.1, +e.target.value || 1);
  q('dt').onchange = e => state.dt = +e.target.value || 1;
  q('adaptive').onchange = e => state.adaptive = e.target.checked;
  q('enablePN').onchange = e => {
    state.pn = e.target.checked;
    if (e.target.checked) reorbit();
  };
  
  if (q('enablePN2')) {
    q('enablePN2').onchange = e => {
      state.pn2 = e.target.checked;
      if (e.target.checked) {
        state.pn = true;
        q('enablePN').checked = true;
        reorbit();
      }
    };
  }
  
  if (q('tidalForces')) {
    q('tidalForces').onchange = e => state.tidalForces = e.target.checked;
  }
  
  if (q('gravWaves')) {
    q('gravWaves').onchange = e => state.gravWaves = e.target.checked;
  }
  
  q('collisionMode').onchange = e => state.collision = e.target.value;
  
  q('preset').onchange = () => { q('btnLoadPreset').disabled = false; };
  q('btnLoadPreset').onclick = () => { loadPreset(q('preset').value); };
  
  q('showTrails').onchange = e => state.trails = e.target.checked;
  q('showGrid').onchange = e => state.showGrid = e.target.checked;
  q('btnReorbit').onclick = reorbit;
  q('btnTrackRecalc').onclick = () => {
    reorbit();
    setTimeout(reorbit, 40000 / Math.max(0.1,state.speedMul));
  };
  
  if (q('trackMode')) {
    q('trackMode').onchange = e => { trackState.mode = e.target.value; };
  }
  
  if (q('showDistChart')) {
    q('showDistChart').onchange = e => state.showDistances = e.target.checked;
  }
  
  q('btnAdd').onclick = () => {
    const m = +q('bMass').value || 1;
    const newBody = {
      name: q('bName').value || "Body",
      mass: m,
      x:+q('bX').value||0, y:+q('bY').value||0, z:+q('bZ').value||0,
      vx:+q('bVX').value||0, vy:+q('bVY').value||0, vz:+q('bVZ').value||0,
      radius: Math.max(1000, Math.cbrt(Math.max(1,m)/1e20) * 1e7),
      color: randColor(),
      trail: []
    };
    bodies.push(newBody);
    
    const totalOthers = bodies.slice(0,-1).reduce((s,b)=>s+b.mass,0);
    const heavyChanged = (m > totalOthers*0.5) || (heaviestIndex() === bodies.length-1);
    
    if (q('anchorHeavy')?.checked && heavyChanged){
      anchorToHeaviest();
    } else {
      reframeToBarycentric();
    }
    
    refreshCards();
    refreshTrackList();
    reorbit();
  };
  
  q('btnClear').onclick = () => {
    bodies=[];
    refreshCards();
    refreshTrackList();
  };
  
  q('btnCSV').onclick = () => {
    let csv = "Time(s),Body Name,Mass(kg),x(m),y(m),z(m),vx(m/s),vy(m/s),vz(m/s),a(m),e,precession(\"/century)\n";
    bodies.forEach(b=>{
      csv += `${time},${b.name},${b.mass},${b.x},${b.y},${b.z},${b.vx},${b.vy},${b.vz},${b.a||0},${b.e||0},${b.precession||0}\n`;
    });
    const blob = new Blob([csv], { type:'text/csv' });
    const a = document.createElement('a');
    a.download = 'nbody_sim_' + new Date().toISOString().replace(/[:.]/g,'-') + '.csv';
    a.href = URL.createObjectURL(blob);
    a.click();
    URL.revokeObjectURL(a.href);
  };
  
  q('btnBary').onclick = () => { reframeToBarycentric(); reorbit(); };
  q('btnAnchorHeavy').onclick = () => { anchorToHeaviest(); reorbit(); };
  q('btnRetune').onclick = () => {
    retuneCircularAroundHeaviest();
    if (q('anchorHeavy')?.checked) anchorToHeaviest();
    reorbit();
  };
  
  q('btnWarmup').onclick = () => warmup(200, 0.15);
  
  if (q('btnAnalyze')) {
    q('btnAnalyze').onclick = performAnalysis;
  }
}

function warmup(steps=200, dtFactor=0.15){
  const base = state.dt;
  const oldAdaptive = state.adaptive;
  state.adaptive = false;
  
  for (let k=0;k<steps;k++){
    rk4Step(base * dtFactor, state.pn);
    time += base * dtFactor;
    bodies.forEach(b=>{
      b.trail.push({x:b.x,y:b.y,z:b.z});
      if (b.trail.length>600) b.trail.shift();
    });
  }
  
  state.adaptive = oldAdaptive;
  updateStats();
}

let autoReorbitInterval;
function startAutoReorbit(){
  clearInterval(autoReorbitInterval);
  autoReorbitInterval = setInterval(()=>{
    if (running && q('autoReorbit')?.checked) reorbit();
  }, 30000);
}

/* أدوات مساعدة */
const v2 = (x, y, z) => x*x + y*y + z*z;
const mag = (x, y, z) => Math.sqrt(x*x + y*y + z*z);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi,v));
const palette = ["#58a6ff", "#ff7b72", "#8ddb8c", "#f2cc60", "#c39df3", "#76e4f7", "#f78fb3", "#a8d1ff"];
const randColor = () => palette[(Math.random()*palette.length)|0];

/* كاميرا */
let cam = { dist: 5e11, rotX: 30 * Math.PI / 180, rotY: 45 * Math.PI / 180, scale: 1 };

function resize() {
  if (!view || !chart) return;
  const pr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  view.width = Math.floor(view.clientWidth * pr);
  view.height = Math.floor(view.clientHeight * pr);
  chart.width = Math.floor(chart.clientWidth * pr);
  chart.height = Math.floor(90 * pr);
  ctx.setTransform(pr,0,0,pr,0,0);
  cctx.setTransform(pr,0,0,pr,0,0);
}

window.addEventListener('resize', resize);

function updateCamera() {
  if (!q('zoomRange') || !q('rotX') || !q('rotY')) return;
  const zr = parseFloat(q('zoomRange').value || 0);
  cam.dist = 2e11 * Math.exp(zr * 0.0115);
  cam.rotX = parseFloat(q('rotX').value || 30) * Math.PI / 180;
  cam.rotY = parseFloat(q('rotY').value || 45) * Math.PI / 180;
  cam.scale = view.width / Math.max(1, (cam.dist * 0.8));
  const label = document.getElementById('zoomLabel');
  if (label && isFinite(cam.dist)) label.textContent = `المسافة: ${(cam.dist / 1e9).toFixed(1)} Gm`;
}

if (q('zoomRange')) q('zoomRange').oninput = updateCamera;
if (q('rotX')) q('rotX').oninput = updateCamera;
if (q('rotY')) q('rotY').oninput = updateCamera;

function project(x, y, z) {
  const cx = Math.cos(cam.rotY), sx = Math.sin(cam.rotY);
  const cy = Math.cos(cam.rotX), sy = Math.sin(cam.rotX);
  const x1 = x * cx - y * sx, y1 = x * sx + y * cx, z1 = z;
  const y2 = y1 * cy - z1 * sy, z2 = y1 * sy + z1 * cy + cam.dist;
  if (!isFinite(x1) || !isFinite(y2) || !isFinite(z2) || z2 <= 0) return null;
  const scale = cam.scale * (cam.dist / z2);
  if (!isFinite(scale)) return null;
  const X = (view.width / 2) + x1 * scale, Y = (view.height / 2) - y2 * scale;
  return (isFinite(X) && isFinite(Y)) ? [X, Y, z2, scale] : null;
}

/* تحكم بالفأرة */
let dragging = false, lastX = 0, lastY = 0;
if (view) {
  view.addEventListener('mousedown', e => {
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
  });
  
  window.addEventListener('mouseup', () => dragging = false);
  
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    q('rotY').value = (+q('rotY').value || 0) + (e.clientX - lastX) * 0.5;
    q('rotX').value = (+q('rotX').value || 0) - (e.clientY - lastY) * 0.5;
    lastX = e.clientX;
    lastY = e.clientY;
    updateCamera();
  });
  
  view.addEventListener('wheel', e => {
    q('zoomRange').value = (+q('zoomRange').value || 0) - e.deltaY * 0.1;
    updateCamera();
  }, { passive: true });
}

/* تهيئة */
if (q('collisionMode')) q('collisionMode').value = 'merge';

q('btnTheme').onclick = () => {
  document.documentElement.classList.toggle('light');
};
/* ... كل الكود السابق ... */



// ====== أضف هنا الكود الجديد ======

/* ====== عرض إحداثيات الموقع عند الماوس ====== */
let mouseCoords = { x: 0, y: 0, z: 0, valid: false };

function screenToWorld(screenX, screenY) {
  const trackOffset = getTrackOffset();
  
  const centerX = view.width / 2;
  const centerY = view.height / 2;
  const dx = screenX - centerX;
  const dy = centerY - screenY;
  
  const scale = cam.scale;
  const x_view = dx / scale;
  const y_view = dy / scale;
  
  const cx = Math.cos(cam.rotY), sx = Math.sin(cam.rotY);
  const cy = Math.cos(cam.rotX), sy = Math.sin(cam.rotX);
  
  const z_view = 0;
  
  const y1 = y_view * cy + z_view * sy;
  const z1 = -y_view * sy + z_view * cy;
  
  const x_world = x_view * cx + y1 * sx;
  const y_world = -x_view * sx + y1 * cx;
  const z_world = z1;
  
  return {
    x: x_world + trackOffset.x,
    y: y_world + trackOffset.y,
    z: z_world + trackOffset.z
  };
}

if (view) {
  view.addEventListener('mousemove', e => {
    const rect = view.getBoundingClientRect();
    const screenX = (e.clientX - rect.left) * (view.width / rect.width);
    const screenY = (e.clientY - rect.top) * (view.height / rect.height);
    
    const coords = screenToWorld(screenX, screenY);
    mouseCoords.x = coords.x;
    mouseCoords.y = coords.y;
    mouseCoords.z = coords.z;
    mouseCoords.valid = true;
  });
  
  view.addEventListener('mouseleave', () => {
    mouseCoords.valid = false;
  });
  
  view.addEventListener('click', e => {
    if (e.ctrlKey && mouseCoords.valid) {
      const text = `x: ${mouseCoords.x.toExponential(4)}\ny: ${mouseCoords.y.toExponential(4)}\nz: ${mouseCoords.z.toExponential(4)}`;
      navigator.clipboard.writeText(text).then(() => {
        console.log('تم نسخ الإحداثيات:', text);
      });
    }
  });
}

function drawMouseCoordinates() {
  if (!mouseCoords.valid) return;
  
  ctx.save();
  ctx.font = '12px monospace';
  ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--text') || '#e6edf3';
  ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--bg') || '#0b0f16';
  ctx.lineWidth = 3;
  
  const x_gm = (mouseCoords.x / 1e9).toFixed(3);
  const y_gm = (mouseCoords.y / 1e9).toFixed(3);
  const z_gm = (mouseCoords.z / 1e9).toFixed(3);
  
  const text = `x: ${x_gm} Gm  y: ${y_gm} Gm  z: ${z_gm} Gm`;
  const textWidth = ctx.measureText(text).width;
  
  const posX = 10;
  const posY = view.height - 15;
  
  ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
  ctx.fillRect(posX - 5, posY - 15, textWidth + 10, 20);
  
  ctx.fillStyle = '#59c1ff';
  ctx.fillText(text, posX, posY);
  
  ctx.font = '10px system-ui';
  ctx.fillStyle = '#9fb3c8';
  ctx.fillText('(Ctrl+Click لنسخ)', posX + textWidth + 15, posY);
  
  ctx.restore();
}


/* تشغيل */
resize();
updateCamera();
loadPreset('solar');
bindUI();
startAutoReorbit();
requestAnimationFrame(loop);