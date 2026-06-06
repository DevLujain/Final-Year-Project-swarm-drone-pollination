// dashboard.js — Stage 16 v12 (matches index.html IDs exactly)

const $ = id => document.getElementById(id);

// Bloom-state colours — v13 vocabulary: open | closed (+ pollinated flag)
const STATE_STYLE = {
  open:      { col: '#ffd23f', r: 5.0 },  // vibrant yellow — open, target
  closed:    { col: '#f5e9b8', r: 3.5 },  // light yellow — discovered bud
  not_ready: { col: '#f5e9b8', r: 3.5 },  // legacy alias of closed
  spent:     { col: '#777777', r: 2.5 },
  unknown:   { col: '#3a3a3a', r: 2.5 },
};

const PHASE_PILL_CLASSES = {
  IDLE:'idle', TAKEOFF:'active', SURVEY:'active', HEIGHT_PROBE:'active',
  RETURN_TO_BASE:'active',
  LAND_SURVEY:'active', AWAIT_ROUTE:'waiting', GOTO_FLOWER:'active',
  DESCEND:'active', BRUSH:'brushing', ASCEND_GENTLE:'active',
  RETREAT:'active', RTB:'returning', LAND_RTB:'returning',
  DAY_DONE:'done', RTB_BATTERY:'returning', RECHARGING:'recharging',
};

const DRONE_COLS = ['#38bdf8','#fb923c','#a3e635','#e879f9'];

// ── Drone cards (injected once) ───────────────────────────────────
let dronesBuilt = false;
function buildDroneCards(n) {
  if (dronesBuilt) return;
  dronesBuilt = true;
  const card = $('drones-card');
  if (!card) return;
  card.innerHTML = '<div class="card-title">DRONES</div>';
  for (let i = 0; i < n; i++) {
    card.innerHTML += `
      <div class="drone-row" id="dr-${i}">
        <span class="drone-dot" style="background:${DRONE_COLS[i%DRONE_COLS.length]}">D${i}</span>
        <span class="phase-pill" id="dr-${i}-phase">IDLE</span>
      </div>`;
  }
}

// ── Field map ─────────────────────────────────────────────────────
function drawMap(snap) {
  const canvas = $('map-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const fw = snap.field_w || 15, fh = snap.field_h || 15;
  const scaleX = W / fw, scaleY = H / fh;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#0d1a0d';
  ctx.fillRect(0, 0, W, H);

  // Grid lines
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.lineWidth = 0.5;
  for (let x = 0; x <= fw; x++) {
    ctx.beginPath(); ctx.moveTo(x*scaleX, 0); ctx.lineTo(x*scaleX, H); ctx.stroke();
  }
  for (let y = 0; y <= fh; y++) {
    ctx.beginPath(); ctx.moveTo(0, y*scaleY); ctx.lineTo(W, y*scaleY); ctx.stroke();
  }

  // Sector divider
  const n = snap.n_drones || 2;
  for (let i = 1; i < n; i++) {
    const sx = (fw / n) * i * scaleX;
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(180,140,60,0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, H); ctx.stroke();
    ctx.setLineDash([]);
  }

  // Flowers
  const flowers = snap.flower_positions || [];
  for (const f of flowers) {
    const px = f.x * scaleX;
    const py = H - f.y * scaleY;
    if (f.pollinated) {
      ctx.beginPath(); ctx.arc(px, py, 5.5, 0, Math.PI * 2);
      ctx.fillStyle = '#22c55e'; ctx.fill();
      ctx.strokeStyle = '#bbf7d0'; ctx.lineWidth = 1.5; ctx.stroke();
    } else {
      const st = STATE_STYLE[f.state] || STATE_STYLE.unknown;
      ctx.beginPath(); ctx.arc(px, py, st.r, 0, Math.PI * 2);
      ctx.fillStyle = st.col; ctx.fill();
    }
  }

  // Drone dots
  const poses = snap.drone_poses || {};
  for (let i = 0; i < n; i++) {
    const p = poses[i]; if (!p) continue;
    const dx = p[0] * scaleX, dy = H - p[1] * scaleY;
    ctx.beginPath(); ctx.arc(dx, dy, 7, 0, Math.PI * 2);
    ctx.fillStyle = DRONE_COLS[i % DRONE_COLS.length]; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center'; ctx.fillText(i, dx, dy + 3.5);
  }

  // Legend (top-left)
  const legend = [
    { col: '#22c55e', label: 'pollinated ✓' },
    { col: '#ffd23f', label: 'open (target)' },
    { col: '#f5e9b8', label: 'closed (bud)' },
  ];
  ctx.textAlign = 'left';
  ctx.font = '10px monospace';
  legend.forEach((item, i) => {
    const ly = 14 + i * 13;
    ctx.beginPath(); ctx.arc(9, ly - 3, 4, 0, Math.PI * 2);
    ctx.fillStyle = item.col; ctx.fill();
    ctx.fillStyle = 'rgba(210,220,210,0.85)';
    ctx.fillText(item.label, 18, ly);
  });
}

// ── Main update ───────────────────────────────────────────────────
let _lastEvCount = 0;
let _cumPollinated = 0;

function updateUI(snap) {
  const n = snap.n_drones || 2;
  buildDroneCards(n);

  // Header
  const setText = (id, v) => { const el = $(id); if (el) el.textContent = v; };
  setText('day-num',  snap.current_day || '—');
  setText('elapsed',  snap.elapsed_s   || '0.0');

  // Connection dot
  const dot   = $('conn-dot'),   lbl = $('conn-label');
  const conn  = snap.connected;
  if (dot) dot.style.background = conn ? '#4ade80' : '#f87171';
  if (lbl) lbl.textContent = conn ? 'connected' : 'connecting…';

  // Coverage card
  const pct  = snap.coverage_pct || 0;
  const poll = snap.flowers_pollinated || 0;
  const tot  = snap.total_flowers || 0;
  setText('coverage-pct', pct.toFixed(1));
  setText('poll-num', poll);
  setText('poll-den', tot);
  const bar = $('coverage-bar');
  if (bar) bar.style.width = pct + '%';
  const ms  = $('mission-status');
  if (ms) ms.textContent = snap.mission_complete ? '✓ mission complete'
                          : snap.mission_active  ? 'mission in progress'
                          : 'awaiting drones…';

  // Metrics
  setText('m-observed',   snap.total_flowers  || 0);
  setText('m-pollinated', snap.flowers_pollinated || 0);
  setText('m-day',        snap.current_day    || '—');

  // Battery bars (HTML IDs: batt-0, batt-0-pct, batt-1, batt-1-pct)
  const batt = snap.battery || {};
  for (let i = 0; i < n; i++) {
    const pct = batt[i] ?? 100;
    const barEl = $('batt-' + i);
    const pctEl = $('batt-' + i + '-pct');
    if (barEl) {
      barEl.style.width = pct + '%';
      barEl.style.background = pct > 40 ? '#4ade80' : pct > 20 ? '#facc15' : '#f87171';
    }
    if (pctEl) pctEl.textContent = Math.round(pct);
  }

  // Last update
  if (snap.last_update) {
    const d = new Date(snap.last_update * 1000);
    setText('m-lastupd', d.toTimeString().slice(0, 8));
  }

  // Drone phase pills
  const states = snap.drone_states || {};
  for (let i = 0; i < n; i++) {
    const raw   = states[i] || 'IDLE';
    const phase = raw.split(' ')[0];
    const pill  = $('dr-' + i + '-phase');
    if (pill) {
      pill.textContent  = raw;
      pill.className    = 'phase-pill ' + (PHASE_PILL_CLASSES[phase] || '');
    }
  }

  // Event log
  const evBody = $('ev-body');
  const evLog  = snap.event_log || [];
  if (evBody && evLog.length !== _lastEvCount) {
    _lastEvCount = evLog.length;
    setText('ev-count', evLog.length);
    evBody.innerHTML = evLog.slice(-20).reverse().map(e => {
      const t   = new Date(e.ts * 1000).toTimeString().slice(0, 8);
      const did = e.drone_id != null ? e.drone_id : '—';
      const det = e.type === 'pollination_success'
        ? `✿ flower ${e.payload?.flower_id ?? ''}`
        : e.type === 'day_start'
        ? `day ${e.payload?.day ?? ''} | ${e.payload?.n_targets ?? 0} targets`
        : e.type === 'routes_assigned'
        ? JSON.stringify(e.payload?.route_counts || {})
        : '';
      return `<tr><td>${t}</td><td>${e.type}</td><td>${did}</td><td>${det}</td></tr>`;
    }).join('');
  }

  // Daily summary table
  const daily = snap.daily_summary || [];
  const dBody = $('daily-body');
  if (dBody && daily.length > 0) {
    let cum = 0;
    dBody.innerHTML = daily.map(row => {
      cum += row.pollinated || 0;
      return `<tr>
        <td>${row.day}</td>
        <td>${row.new_obs || 0}</td>
        <td>${row.pollinated || 0}</td>
        <td>${cum}</td>
      </tr>`;
    }).join('');
  }

  // v14: labels follow the real field size instead of hardcoded 15 m
  const sn = document.querySelector('.map-card .size-note');
  if (sn) sn.textContent = `${snap.field_w || 15} \u00d7 ${snap.field_h || 15} m`;
  const keys = document.querySelectorAll('.map-key .key-item');
  if (keys.length >= 2) {
    const half = (snap.field_w || 15) / 2;
    keys[0].innerHTML = `<span class="swatch dr0"></span>LEFT (x&lt;${half})`;
    keys[1].innerHTML = `<span class="swatch dr1"></span>RIGHT (x\u2265${half})`;
  }

  drawMap(snap);
}

// ── SSE ───────────────────────────────────────────────────────────
const evtSrc = new EventSource('/stream');
evtSrc.addEventListener('snapshot', e => {
  try { updateUI(JSON.parse(e.data)); }
  catch (err) { console.error('snapshot parse error:', err); }
});
evtSrc.onerror = () => console.warn('SSE disconnected — browser will reconnect');

// ── Reset button ──────────────────────────────────────────────────
const resetBtn = $('btn-reset');
if (resetBtn) {
  resetBtn.addEventListener('click', () => {
    fetch('/api/reset', { method: 'POST' }).catch(() => {});
  });
}
