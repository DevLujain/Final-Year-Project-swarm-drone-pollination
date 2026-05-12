/**
 * dashboard.js  —  Stage 13 DaaS Dashboard (Multi-day)
 * =======================================================
 * Handles:
 *   - Phase banner (MAPPING / POLLINATING / BETWEEN_DAYS / COMPLETE)
 *   - Coverage card (shows mapping % or pollination % by phase)
 *   - Drone cards (SWEEP state during mapping, full 8 states during pollination)
 *   - Health map canvas (only discovered flowers, colour-coded by state)
 *   - 3-day summary table
 *   - Live pollination event log (with day badge)
 *
 * Field: 60 m × 20 m  →  canvas 600 × 200 px (10 px/m)
 * Health grid: 60 cols × 20 rows
 */

// ── Canvas constants ──────────────────────────────────────────────────────────
const FIELD_W   = 60.0;
const FIELD_H   = 20.0;
const CANVAS_W  = 600;
const CANVAS_H  = 200;
const GRID_COLS = 60;
const GRID_ROWS = 20;
const CELL_W    = CANVAS_W / GRID_COLS;
const CELL_H    = CANVAS_H / GRID_ROWS;
const SECTOR_PX = [(20 / FIELD_W) * CANVAS_W, (40 / FIELD_W) * CANVAS_W];

// ── State colours ─────────────────────────────────────────────────────────────
const STATE_CLASS = {
    SWEEP:     's-SWEEP',
    PREFLIGHT: 's-PREFLIGHT',
    ARMING:    's-ARMING',
    TAKEOFF:   's-TAKEOFF',
    SEARCH:    's-SEARCH',
    APPROACH:  's-APPROACH',
    HOVER:     's-HOVER',
    POLLINATE: 's-POLLINATE',
    LAND:      's-LAND',
    IDLE:      's-IDLE',
    DONE:      's-DONE',
};
const STATE_BORDER = {
    SWEEP:     '#3b82f6',
    SEARCH:    '#10b981',
    APPROACH:  '#f59e0b',
    HOVER:     '#6366f1',
    POLLINATE: '#E9C46A',
    LAND:      '#ef4444',
    IDLE:      '#2D6A4F',
    DONE:      '#22c55e',
};

// ── Drone sector labels ───────────────────────────────────────────────────────
const DRONE_SECTOR = ['LEFT', 'CENTER', 'RIGHT'];

// ── Flower dot colours by state ───────────────────────────────────────────────
const FLOWER_COLOUR = {
    pollinated: 'rgba(34, 197, 94, 0.90)',    // green
    ready:      'rgba(234, 179, 8, 0.90)',    // yellow
    future:     'rgba(96, 165, 250, 0.70)',   // light blue
    closed:     'rgba(120, 120, 120, 0.50)',  // grey
};

// ── Caches ────────────────────────────────────────────────────────────────────
let _flowerCache   = [];
let _missionStart  = Date.now();
let _prevPhase     = '';
let _prevDay       = 0;


// ── SSE ───────────────────────────────────────────────────────────────────────

function connectSSE() {
    const src = new EventSource('/api/stream');
    src.onmessage = (e) => {
        try { updateDashboard(JSON.parse(e.data)); }
        catch (err) { console.error('SSE parse:', err); }
    };
    src.onerror = () => {
        const b = document.getElementById('connection-badge');
        if (b) { b.textContent = '⬤ Disconnected'; b.className = 'badge badge-offline'; }
    };
}


// ── Master update ─────────────────────────────────────────────────────────────

function updateDashboard(data) {
    updateBadge(data.connected, data.mission_active);
    updatePhaseBanner(data);
    updateCoverageCard(data);
    updateDroneCards(data);
    updateDroneMiniStats(data);
    updateHealthMap(data.health_grid, data.flower_positions);
    updateFieldStats(data);
    updateMetrics(data.tsp_pct, data.bloom_pct, data.total_events);
    updateDayTable(data);
    updatePerfTable(data);
    updateEventLog(data.pollination_events);
}


// ── Badge ─────────────────────────────────────────────────────────────────────

function updateBadge(connected, active) {
    const b = document.getElementById('connection-badge');
    if (!b) return;
    if (connected)    { b.textContent = '⬤ ROS 2 Live'; b.className = 'badge badge-online'; }
    else if (active)  { b.textContent = '⬤ Simulation'; b.className = 'badge badge-sim'; }
    else              { b.textContent = '⬤ Standby';    b.className = 'badge badge-offline'; }
}


// ── Phase banner ──────────────────────────────────────────────────────────────

function updatePhaseBanner(data) {
    const banner  = document.getElementById('phase-banner');
    const icon    = document.getElementById('phase-icon');
    const dayLbl  = document.getElementById('phase-day-label');
    const statLbl = document.getElementById('phase-status-label');
    const barFill = document.getElementById('phase-bar-fill');
    const pctLbl  = document.getElementById('phase-pct');
    if (!banner) return;

    const phase  = data.current_phase || 'IDLE';
    const day    = data.current_day   || 0;
    let pct = 0;

    banner.className = 'phase-banner';

    switch (phase) {
        case 'MAPPING':
            banner.classList.add('phase-mapping');
            icon.textContent    = '📡';
            dayLbl.textContent  = `Day ${day} — Mapping`;
            statLbl.textContent = `Sweeping field... ${data.discovered_today ?? 0} flowers found`;
            pct = data.mapping_progress ?? 0;
            pctLbl.textContent  = pct.toFixed(1) + '%';
            break;
        case 'POLLINATING':
            banner.classList.add('phase-pollinating');
            icon.textContent    = '🌸';
            dayLbl.textContent  = `Day ${day} — Pollinating`;
            statLbl.textContent = `${data.pollinated_today ?? 0} / ${data.ready_today ?? 0} ready flowers pollinated`;
            pct = data.coverage_today_pct ?? 0;
            pctLbl.textContent  = pct.toFixed(1) + '%';
            break;
        case 'BETWEEN_DAYS':
            banner.classList.add('phase-between');
            icon.textContent    = '⏳';
            dayLbl.textContent  = `Day ${day} Complete`;
            statLbl.textContent = `Preparing Day ${day + 1}...`;
            pct = 100;
            pctLbl.textContent  = '100%';
            break;
        case 'COMPLETE': {
            banner.classList.add('phase-complete');
            icon.textContent    = '✅';
            dayLbl.textContent  = `Mission Complete — Day ${day}`;
            const reasons = {
                all_pollinated: `All ${data.total_pollinated_all ?? 0} target flowers pollinated`,
                zero_yield:     `Bloom window ended — no new flowers for 2 days`,
                day_limit:      `Bloom window naturally completed`,
                hard_limit:     `Safety ceiling (Day ${data.mission_hard_limit}) reached`,
            };
            statLbl.textContent = reasons[data.mission_stop_reason] || 'Mission ended';
            pct = 100;
            pctLbl.textContent  = '✓';
            break;
        }
        default:
            banner.classList.add('phase-idle');
            icon.textContent    = '🛰️';
            dayLbl.textContent  = 'Ready';
            statLbl.textContent = `Press Start — ${data.total_field_flowers ?? '?'} flowers in field, mission runs until bloom window closes`;
            pct = 0;
            pctLbl.textContent  = '—';
    }

    if (barFill) barFill.style.width = Math.min(pct, 100) + '%';
}


// ── Coverage card ─────────────────────────────────────────────────────────────

function updateCoverageCard(data) {
    const titleEl  = document.getElementById('coverage-card-title');
    const bigEl    = document.getElementById('coverage-pct');
    const subEl    = document.getElementById('coverage-sub');
    const barEl    = document.getElementById('coverage-bar');
    const targetEl = document.getElementById('coverage-target');
    if (!bigEl) return;

    const phase = data.current_phase || 'IDLE';
    const day   = data.current_day   || 0;

    if (phase === 'MAPPING') {
        if (titleEl)  titleEl.textContent = `Day ${day} — Field Scan`;
        bigEl.textContent  = (data.mapping_progress ?? 0).toFixed(0) + '%';
        if (subEl)    subEl.textContent  =
            `${data.ready_today ?? 0} ready  /  ${data.not_ready_today ?? 0} not ready discovered`;
        if (barEl)    barEl.style.width  = (data.mapping_progress ?? 0) + '%';
        if (targetEl) targetEl.textContent = 'Open-ended mission';
    } else if (phase === 'POLLINATING') {
        if (titleEl)  titleEl.textContent = `Day ${day} — Pollination`;
        bigEl.textContent  = (data.coverage_today_pct ?? 0).toFixed(1) + '%';
        if (subEl)    subEl.textContent   =
            `${data.pollinated_today ?? 0} / ${data.ready_today ?? 0} ready flowers`;
        if (barEl)    barEl.style.width   = (data.coverage_today_pct ?? 0) + '%';
        if (targetEl) targetEl.textContent = 'Target: ≥ 90 %';
    } else if (phase === 'COMPLETE') {
        if (titleEl)  titleEl.textContent = `${day}-Day Mission Total`;
        const total_pol = data.total_pollinated_all ?? 0;
        const possible  = data.total_pollination_possible ?? total_pol;
        const pct       = possible > 0 ? ((total_pol / possible) * 100).toFixed(1) : '100';
        bigEl.textContent  = pct + '%';
        if (subEl)    subEl.textContent   =
            `${total_pol} / ${possible} target flowers pollinated`;
        if (barEl)    barEl.style.width   = pct + '%';
        if (targetEl) targetEl.textContent = 'Mission complete ✅';
    } else {
        if (titleEl)  titleEl.textContent = 'Field Coverage';
        bigEl.textContent  = '—';
        if (subEl)    subEl.textContent   = 'Press Start to begin';
        if (barEl)    barEl.style.width   = '0%';
        if (targetEl) targetEl.textContent = 'Open-ended mission';
    }
}


// ── Drone cards ───────────────────────────────────────────────────────────────

function updateDroneCards(data) {
    const phase    = data.current_phase || 'IDLE';
    const states   = data.drone_states    || {};
    const targets  = data.current_targets || {};
    const sReady   = data.sector_ready    || {};
    const sNotRdy  = data.sector_not_ready|| {};

    for (let d = 0; d < 3; d++) {
        const state  = states[d]  || 'IDLE';
        const target = targets[d] || null;
        const sector = DRONE_SECTOR[d];

        // State pill
        const pill = document.getElementById(`drone-state-${d}`);
        if (pill) {
            pill.textContent = state;
            pill.className   = 'drone-state-pill ' + (STATE_CLASS[state] || 's-IDLE');
        }

        // Card border
        const card = document.getElementById(`drone-card-${d}`);
        if (card) card.style.borderTopColor = STATE_BORDER[state] || '#2D6A4F';

        // Current target / sweep position
        const tEl = document.getElementById(`drone-target-${d}`);
        if (tEl) {
            if (target && state === 'SWEEP') {
                tEl.textContent = `→ (${target.x?.toFixed(1)}, ${target.y?.toFixed(1)})`;
            } else if (target && target.id && !target.id.startsWith('(')) {
                tEl.textContent = `→ ${target.id}  (${target.x}, ${target.y})`;
            } else {
                tEl.textContent = '';
            }
        }

        // Progress text — different by phase
        const pEl = document.getElementById(`drone-prog-${d}`);
        if (pEl) {
            if (phase === 'MAPPING') {
                const r  = sReady[sector]  || 0;
                const nr = sNotRdy[sector] || 0;
                pEl.textContent = `${r} ready · ${nr} not ready`;
            } else if (phase === 'POLLINATING') {
                // Show ready count for this sector as denominator
                const r = sReady[sector] || 0;
                pEl.textContent = `target: ${r} flowers`;
            } else {
                pEl.textContent = '';
            }
        }
    }
}


// ── Health map canvas ─────────────────────────────────────────────────────────

function updateHealthMap(grid, flowers) {
    const canvas = document.getElementById('health-map-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    canvas.width  = CANVAS_W;
    canvas.height = CANVAS_H;

    // 1. Background
    ctx.fillStyle = '#111827';
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

    // 2. Health grid cells
    if (grid && grid.length === GRID_ROWS) {
        for (let row = 0; row < GRID_ROWS; row++) {
            for (let col = 0; col < GRID_COLS; col++) {
                const v = grid[row][col];
                if (v <= 0) continue;
                ctx.fillStyle = confidenceColour(v);
                ctx.fillRect(col * CELL_W, (GRID_ROWS - 1 - row) * CELL_H, CELL_W, CELL_H);
            }
        }
    }

    // 3. Sector lines
    ctx.strokeStyle = 'rgba(0,220,255,0.65)';
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([5, 4]);
    SECTOR_PX.forEach(xPx => {
        ctx.beginPath(); ctx.moveTo(xPx, 0); ctx.lineTo(xPx, CANVAS_H); ctx.stroke();
    });
    ctx.setLineDash([]);

    // 4. Labels
    ctx.font      = '9px monospace';
    ctx.fillStyle = 'rgba(0,220,255,0.80)';
    ctx.fillText('DRONE 0 — LEFT',   6, 14);
    ctx.fillText('DRONE 1 — CENTER', SECTOR_PX[0] + 6, 14);
    ctx.fillText('DRONE 2 — RIGHT',  SECTOR_PX[1] + 6, 14);

    // 5. Flower dots (only discovered)
    if (flowers && flowers.length > 0) _flowerCache = flowers;
    _flowerCache.forEach(f => {
        const xPx = (f.x / FIELD_W) * CANVAS_W;
        const yPx = CANVAS_H - (f.y / FIELD_H) * CANVAS_H;
        ctx.beginPath();
        ctx.arc(xPx, yPx, 1.8, 0, Math.PI * 2);
        ctx.fillStyle = FLOWER_COLOUR[f.state] || 'rgba(200,200,200,0.4)';
        ctx.fill();
    });
}

function confidenceColour(v) {
    const t = Math.min(1, Math.max(0, (v - 0.70) / 0.28));
    let r, g;
    if (t < 0.5) { r = 180; g = Math.round(30 + 150 * (t / 0.5)); }
    else         { r = Math.round(180 - 160 * ((t-0.5)/0.5)); g = Math.round(180 + 60 * ((t-0.5)/0.5)); }
    return `rgba(${r},${g},20,0.55)`;
}


// ── Field stats bar ───────────────────────────────────────────────────────────

function updateFieldStats(data) {
    const el = document.getElementById('field-stats-bar');
    if (!el) return;
    const phase = data.current_phase || 'IDLE';
    if (phase === 'IDLE') {
        el.textContent = `Field: ${data.total_field_flowers ?? '—'} flowers total`;
        return;
    }
    const sr = data.sector_ready     || {};
    const sn = data.sector_not_ready || {};
    el.textContent =
        `Discovered today: ${data.discovered_today ?? 0} flowers  —  ` +
        `LEFT: ${(sr.LEFT||0)} ready / ${(sn.LEFT||0)} not  |  ` +
        `CENTER: ${(sr.CENTER||0)} ready / ${(sn.CENTER||0)} not  |  ` +
        `RIGHT: ${(sr.RIGHT||0)} ready / ${(sn.RIGHT||0)} not`;
}


// ── Metrics ───────────────────────────────────────────────────────────────────

function updateMetrics(tsp, bloom, totalEvents) {
    const tspEl   = document.getElementById('tsp-value');
    const bloomEl = document.getElementById('bloom-value');
    const evEl    = document.getElementById('events-count');
    if (tspEl)   tspEl.textContent   = tsp   != null ? `-${tsp}%`   : '—';
    if (bloomEl) bloomEl.textContent  = bloom != null ? `+${bloom}%` : '—';
    if (evEl)    evEl.textContent     = totalEvents ?? 0;
}


// ── Day summary table (dynamic rows for 8–10 days) ───────────────────────────

function updateDayTable(data) {
    const tbody = document.getElementById('day-summary-body');
    if (!tbody) return;
    const ds   = data.day_stats || {};
    const days = Object.keys(ds).map(Number).sort((a,b) => a-b);
    if (!days.length) return;

    let html = '';
    let totScan=0, totReady=0, totNot=0, totPol=0, totMiss=0;
    let anyCumulative=0, anyComplete=false;

    for (const d of days) {
        const stat = ds[String(d)] || {};
        const isPending  = !stat.status || stat.status === 'pending';
        const isComplete = stat.status === 'complete';
        const cls = isPending ? 'day-pending' : isComplete ? 'day-complete' : 'day-active';
        const v = (val) => (isPending ? '—' : (val ?? '—'));

        html += `<tr class="day-row ${cls}">
            <td class="day-label">📅 Day ${d}</td>
            <td>${v(stat.new_discovered)}</td>
            <td>${v(stat.prev_pollinated)}</td>
            <td>${v(stat.ready)}</td>
            <td>${v(stat.not_ready)}</td>
            <td>${v(stat.pollinated)}</td>
            <td>${v(stat.missed)}</td>
        </tr>`;

        if (isComplete) {
            anyComplete     = true;
            totScan        += stat.new_discovered || 0;
            totReady       += stat.ready          || 0;
            totNot         += stat.not_ready      || 0;
            totPol         += stat.pollinated     || 0;
            totMiss        += stat.missed         || 0;
            anyCumulative   = stat.cumulative_scanned || anyCumulative;
        }
    }

    if (anyComplete) {
        const neverOpen = data.total_never_open ?? totNot;
        html += `<tr class="day-row day-total">
            <td class="day-label">📊 Total</td>
            <td>${anyCumulative}</td><td>—</td>
            <td>${totReady}</td><td>${neverOpen}</td>
            <td>${totPol}</td><td>${totMiss}</td>
        </tr>`;
    }

    tbody.innerHTML = html || '<tr><td colspan="7" class="empty-row">Waiting for Day 1...</td></tr>';

    const lbl = document.getElementById('mission-totals-label');
    if (lbl && data.current_phase === 'COMPLETE') {
        const reasons = {
            all_pollinated: '✅ All target flowers pollinated',
            zero_yield:     '🌿 Bloom window ended — 2 consecutive zero days',
            day_limit:      `📅 Day ${day} — hard limit reached`,
            hard_limit:     `🔒 Safety ceiling (Day ${data.mission_hard_limit}) reached`,
        };
        const r = reasons[data.mission_stop_reason] || '';
        lbl.textContent = r
            ? `— ${r}  ·  ${data.total_pollinated_all} pollinated, ${data.total_never_open} never opened`
            : '';
    }
}


// ── Drone mini-stats (speed / distance / flight time) ─────────────────────────

function _fmtDist(m) {
    if (m === null || m === undefined) return '—';
    if (m >= 1500) return (m / 1000).toFixed(2) + ' km';
    return m.toFixed(2) + ' m';
}

function _fmtTime(secs) {
    if (secs === null || secs === undefined || secs === 0) return '0 min 00 s';
    const m = Math.floor(secs / 60);
    const s = Math.floor(secs % 60).toString().padStart(2, '0');
    return `${m} min ${s} s`;
}

function updateDroneMiniStats(data) {
    const speeds = data.drone_speed     || {};
    const dists  = data.drone_distance  || {};
    const secs   = data.drone_real_secs || {};

    for (let d = 0; d < 3; d++) {
        const spd  = speeds[String(d)] ?? 0;
        const dist = dists[String(d)]  ?? 0;
        const sec  = secs[String(d)]   ?? 0;

        const spdEl  = document.getElementById(`d${d}-speed`);
        const distEl = document.getElementById(`d${d}-dist`);
        const timeEl = document.getElementById(`d${d}-time`);

        if (spdEl)  spdEl.textContent  = spd.toFixed(1) + ' m/s';
        if (distEl) distEl.textContent = _fmtDist(dist);
        if (timeEl) timeEl.textContent = _fmtTime(sec);
    }
}


// ── Performance table (dynamic rows for 8–10 days) ───────────────────────────

function updatePerfTable(data) {
    const tbody = document.getElementById('perf-summary-body');
    if (!tbody) return;
    const dp   = data.day_perf || {};
    const days = Object.keys(dp).map(Number).sort((a,b) => a-b);
    if (!days.length) return;

    let html = '';
    for (const d of days) {
        const perf = dp[String(d)] || {};
        const isPending = !perf.status || perf.status === 'pending';
        const cls = isPending ? 'day-pending'
                  : perf.status === 'active' ? 'day-active' : 'day-complete';
        const v   = (val) => isPending ? '—' : val;
        const totalSecs = (perf.map_time || 0) + (perf.pol_time || 0);
        const totalDist = (perf.map_dist || 0) + (perf.pol_dist || 0);

        html += `<tr class="day-row ${cls}">
            <td class="day-label">📅 Day ${d}</td>
            <td>${v((perf.avg_speed||0).toFixed(2) + ' m/s')}</td>
            <td>${v(_fmtTime(perf.map_time))}</td>
            <td>${v(_fmtTime(perf.pol_time))}</td>
            <td>${v(_fmtTime(totalSecs))}</td>
            <td>${v(_fmtDist(totalDist))}</td>
        </tr>`;
    }

    tbody.innerHTML = html || '<tr><td colspan="6" class="empty-row">Waiting for Day 1...</td></tr>';
}


// ── Event log ─────────────────────────────────────────────────────────────────

function updateEventLog(events) {
    const tbody = document.getElementById('event-log-body');
    if (!tbody) return;
    if (!events || events.length === 0) {
        tbody.innerHTML = '<tr><td colspan="6" class="empty-row">Press Start to begin mission...</td></tr>';
        return;
    }
    tbody.innerHTML = [...events].reverse().map(ev => {
        const t   = new Date((ev.timestamp || 0) * 1000).toTimeString().slice(0, 8);
        const day = ev.day || 1;
        return `<tr>
            <td><span class="day-badge day-badge-${day}">Day ${day}</span></td>
            <td>${t}</td>
            <td>Drone ${ev.drone_id}</td>
            <td>${ev.flower_id}</td>
            <td>(${ev.flower_x?.toFixed(1)}, ${ev.flower_y?.toFixed(1)})</td>
            <td>${ev.sector || '—'}</td>
        </tr>`;
    }).join('');
}


// ── Mission controls ──────────────────────────────────────────────────────────

async function sendControl(action) {
    const fb = document.getElementById('control-feedback');
    if (fb) fb.textContent = '...';
    if (action === 'start') _missionStart = Date.now();
    if (action === 'reset') { _flowerCache = []; _prevPhase = ''; _prevDay = 0; }

    try {
        const resp = await fetch('/api/control', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({action}),
        });
        const d = await resp.json();
        if (fb) { fb.textContent = d.message || 'Done'; setTimeout(() => { if(fb) fb.textContent=''; }, 3000); }
    } catch (e) {
        if (fb) fb.textContent = 'Error: ' + e.message;
    }
}


// ── Elapsed clock ─────────────────────────────────────────────────────────────

function tickClock() {
    const el = document.getElementById('elapsed-time');
    if (!el) return;
    const s = Math.floor((Date.now() - _missionStart) / 1000);
    el.textContent = `${Math.floor(s/60).toString().padStart(2,'0')}:${(s%60).toString().padStart(2,'0')}`;
}


// ── Init ──────────────────────────────────────────────────────────────────────
connectSSE();
setInterval(tickClock, 1000);
