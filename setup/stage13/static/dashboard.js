/**
 * dashboard.js  —  Stage 13 DaaS Dashboard
 * ==========================================
 * Receives live JSON from Flask SSE stream (every 0.5 s).
 *
 * Renders:
 *   A. Per-sector live progress in each drone card
 *   B. Current-target line in each drone card
 *   C. Dynamic TSP + Bloom metrics (recalculated each run)
 *   D. Field stats bar under canvas (total + per-sector flower counts)
 *   E. Mission-complete banner when coverage hits 100 %
 *
 * Health map:
 *   Canvas 600 × 200 px  =  60 m × 20 m field at 10 px / m
 *   Grid cells: 60 cols × 20 rows  (1 cell = 1 m²)
 *   Flowers plotted as small dots from server data (300–600 per run)
 *   Sector boundaries at x = 20 m (px 200) and x = 40 m (px 400)
 *
 * All element IDs match index.html exactly — no aliases.
 */

// ── Field / canvas constants ──────────────────────────────────────────────────
const FIELD_W   = 60.0;   // metres
const FIELD_H   = 20.0;   // metres
const CANVAS_W  = 600;    // px
const CANVAS_H  = 200;    // px
const GRID_COLS = 60;
const GRID_ROWS = 20;
const CELL_W    = CANVAS_W / GRID_COLS;   // 10 px
const CELL_H    = CANVAS_H / GRID_ROWS;   // 10 px

// Sector x boundaries in pixels (20 m and 40 m)
const SECTOR_PX = [
    (20 / FIELD_W) * CANVAS_W,   // 200 px
    (40 / FIELD_W) * CANVAS_W,   // 400 px
];

// Drone → sector mapping for progress lookup
const DRONE_SECTOR = ['LEFT', 'CENTER', 'RIGHT'];

// State → pill CSS class
const STATE_CLASS = {
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

// Border accent colour per state (for drone card top border)
const STATE_BORDER = {
    PREFLIGHT: '#94a3b8',
    ARMING:    '#8b5cf6',
    TAKEOFF:   '#3b82f6',
    SEARCH:    '#10b981',
    APPROACH:  '#f59e0b',
    HOVER:     '#6366f1',
    POLLINATE: '#E9C46A',
    LAND:      '#ef4444',
    IDLE:      '#2D6A4F',
    DONE:      '#22c55e',
};

// Cache flower positions between SSE ticks to avoid redraw flicker on empty frames
let _flowerCache = [];

// Mission complete flag — prevent banner showing repeatedly
let _missionCompleteFired = false;

// Mission start ms for elapsed clock
let _missionStartMs = Date.now();


// ── SSE connection ─────────────────────────────────────────────────────────────

function connectSSE() {
    const src = new EventSource('/api/stream');

    src.onmessage = (e) => {
        try {
            updateDashboard(JSON.parse(e.data));
        } catch (err) {
            console.error('SSE parse error:', err);
        }
    };

    src.onerror = () => {
        const badge = document.getElementById('connection-badge');
        if (badge) {
            badge.textContent = '⬤ Disconnected';
            badge.className   = 'badge badge-offline';
        }
    };
}


// ── Master update ─────────────────────────────────────────────────────────────

function updateDashboard(data) {
    updateBadge(data.connected, data.mission_active);
    updateCoverage(data.coverage_pct, data.flowers_pollinated, data.total_flowers);
    updateDroneCards(data.drone_states, data.current_targets, data.sector_progress);
    updateMetrics(data.tsp_pct, data.bloom_pct, data.pollination_events);
    updateHealthMap(data.health_grid, data.flower_positions);
    updateFieldStats(data.total_field_flowers, data.sector_totals);
    updateEventLog(data.pollination_events);
    checkMissionComplete(data.coverage_pct, data.elapsed_s, data.total_field_flowers);
}


// ── Connection badge ──────────────────────────────────────────────────────────

function updateBadge(connected, active) {
    const b = document.getElementById('connection-badge');
    if (!b) return;
    if (connected) {
        b.textContent = '⬤ ROS 2 Live';
        b.className   = 'badge badge-online';
    } else if (active) {
        b.textContent = '⬤ Simulation';
        b.className   = 'badge badge-sim';
    } else {
        b.textContent = '⬤ Standby';
        b.className   = 'badge badge-offline';
    }
}


// ── Coverage bar ──────────────────────────────────────────────────────────────

function updateCoverage(pct, pollinated, total) {
    const pctEl   = document.getElementById('coverage-pct');
    const countEl = document.getElementById('flowers-count');
    const barEl   = document.getElementById('coverage-bar');

    if (pctEl)   pctEl.textContent  = (pct !== undefined ? pct : '—') + '%';
    if (countEl) countEl.textContent = `${pollinated ?? '—'} / ${total ?? '—'} flowers pollinated`;
    if (barEl)   barEl.style.width   = Math.min(pct ?? 0, 100) + '%';
}


// ── Drone cards (additives A + B) ─────────────────────────────────────────────

function updateDroneCards(states, targets, sectorProgress) {
    for (let d = 0; d < 3; d++) {
        const state   = (states  && states[d])  ? states[d]  : 'IDLE';
        const target  = (targets && targets[d]) ? targets[d] : null;
        const sector  = DRONE_SECTOR[d];
        const prog    = sectorProgress ? sectorProgress[sector] : null;

        // State pill
        const pill = document.getElementById(`drone-state-${d}`);
        if (pill) {
            pill.textContent = state;
            // Remove all state classes, add current
            pill.className = 'drone-state-pill ' + (STATE_CLASS[state] || 's-IDLE');
        }

        // Card border colour
        const card = document.getElementById(`drone-card-${d}`);
        if (card) {
            card.style.borderTopColor = STATE_BORDER[state] || '#2D6A4F';
        }

        // Additive B — current flower target
        const tEl = document.getElementById(`drone-target-${d}`);
        if (tEl) {
            if (target) {
                tEl.textContent = `→ ${target.id}  (${target.x}, ${target.y})`;
            } else {
                tEl.textContent = '';
            }
        }

        // Additive A — sector progress
        const pEl = document.getElementById(`drone-prog-${d}`);
        if (pEl && prog) {
            pEl.textContent = `${prog.pollinated} / ${prog.total}`;
        }
    }
}


// ── Metrics (additive C — dynamic per run) ────────────────────────────────────

function updateMetrics(tsp, bloom, events) {
    const tspEl   = document.getElementById('tsp-value');
    const bloomEl = document.getElementById('bloom-value');
    const evEl    = document.getElementById('events-count');

    if (tspEl)   tspEl.textContent   = tsp   != null ? `-${tsp}%`   : '—';
    if (bloomEl) bloomEl.textContent  = bloom != null ? `+${bloom}%` : '—';
    if (evEl)    evEl.textContent     = events ? events.length : 0;
}


// ── Health map canvas ─────────────────────────────────────────────────────────

function updateHealthMap(grid, flowers) {
    const canvas = document.getElementById('health-map-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    // Ensure canvas size matches constants
    canvas.width  = CANVAS_W;
    canvas.height = CANVAS_H;

    // 1. Dark soil background
    ctx.fillStyle = '#111827';
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

    // 2. Health grid (60×20 cells)
    if (grid && grid.length === GRID_ROWS) {
        for (let row = 0; row < GRID_ROWS; row++) {
            for (let col = 0; col < GRID_COLS; col++) {
                const val     = grid[row][col];
                if (val <= 0) continue;                 // transparent = no data
                const canvasY = (GRID_ROWS - 1 - row) * CELL_H;   // flip y
                ctx.fillStyle = confidenceColour(val);
                ctx.fillRect(col * CELL_W, canvasY, CELL_W, CELL_H);
            }
        }
    }

    // 3. Sector boundary lines
    ctx.strokeStyle = 'rgba(0, 220, 255, 0.70)';
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([5, 4]);
    SECTOR_PX.forEach(xPx => {
        ctx.beginPath();
        ctx.moveTo(xPx, 0);
        ctx.lineTo(xPx, CANVAS_H);
        ctx.stroke();
    });
    ctx.setLineDash([]);

    // 4. Sector labels
    ctx.font      = '9px monospace';
    ctx.fillStyle = 'rgba(0, 220, 255, 0.80)';
    ctx.fillText('DRONE 0 — LEFT',    6, 14);
    ctx.fillText('DRONE 1 — CENTER',  SECTOR_PX[0] + 6, 14);
    ctx.fillText('DRONE 2 — RIGHT',   SECTOR_PX[1] + 6, 14);

    // 5. Flower dots
    if (flowers && flowers.length > 0) _flowerCache = flowers;

    _flowerCache.forEach(f => {
        const xPx = (f.x / FIELD_W) * CANVAS_W;
        const yPx = CANVAS_H - (f.y / FIELD_H) * CANVAS_H;   // flip y

        let colour;
        if (f.bloom === 0) {
            colour = 'rgba(120, 120, 120, 0.40)';   // closed — dim grey
        } else if (f.pollinated) {
            colour = 'rgba(34, 197, 94, 0.90)';     // pollinated — green
        } else {
            colour = 'rgba(234, 179, 8, 0.85)';     // ready — yellow
        }

        ctx.beginPath();
        ctx.arc(xPx, yPx, 1.8, 0, Math.PI * 2);
        ctx.fillStyle = colour;
        ctx.fill();
    });
}

/**
 * Map confidence 0–1 to a semi-transparent fill colour.
 *   ≤ 0.70  → transparent (no data)
 *   0.70    → dark red
 *   0.85    → amber
 *   ≥ 0.98  → bright green
 */
function confidenceColour(v) {
    const t = Math.min(1, Math.max(0, (v - 0.70) / 0.28));
    let r, g;
    if (t < 0.5) {
        r = 180; g = Math.round(30 + 150 * (t / 0.5));
    } else {
        r = Math.round(180 - 160 * ((t - 0.5) / 0.5));
        g = Math.round(180 + 60  * ((t - 0.5) / 0.5));
    }
    return `rgba(${r},${g},20,0.55)`;
}


// ── Field stats bar (additive D) ──────────────────────────────────────────────

function updateFieldStats(totalFlowers, sectorTotals) {
    const el = document.getElementById('field-stats-bar');
    if (!el || !sectorTotals) return;
    const L = sectorTotals.LEFT   || 0;
    const C = sectorTotals.CENTER || 0;
    const R = sectorTotals.RIGHT  || 0;
    el.textContent =
        `Total: ${totalFlowers ?? '—'} flowers  —  ` +
        `LEFT: ${L}  |  CENTER: ${C}  |  RIGHT: ${R}  (ready flowers per column)`;
}


// ── Event log ─────────────────────────────────────────────────────────────────

function updateEventLog(events) {
    const tbody = document.getElementById('event-log-body');
    if (!tbody) return;

    if (!events || events.length === 0) {
        tbody.innerHTML =
            '<tr><td colspan="5" class="empty-row">Press Start to begin mission...</td></tr>';
        return;
    }

    tbody.innerHTML = [...events].reverse().map(ev => {
        const t   = new Date((ev.timestamp || 0) * 1000);
        const hms = t.toTimeString().slice(0, 8);
        return `<tr>
            <td>${hms}</td>
            <td>Drone ${ev.drone_id}</td>
            <td>${ev.flower_id}</td>
            <td>(${ev.flower_x?.toFixed(1)}, ${ev.flower_y?.toFixed(1)})</td>
            <td>${ev.sector || '—'}</td>
        </tr>`;
    }).join('');
}


// ── Mission complete banner (additive E) ──────────────────────────────────────

function checkMissionComplete(pct, elapsedS, total) {
    if (_missionCompleteFired || pct < 100) return;
    _missionCompleteFired = true;

    const banner  = document.getElementById('mission-complete-banner');
    const timeEl  = document.getElementById('banner-time');
    const flowEl  = document.getElementById('banner-flowers');
    if (!banner) return;

    const mins = Math.floor(elapsedS / 60).toString().padStart(2, '0');
    const secs = Math.floor(elapsedS % 60).toString().padStart(2, '0');
    if (timeEl) timeEl.textContent  = `${mins}:${secs}`;
    if (flowEl) flowEl.textContent  = total;

    banner.classList.remove('hidden');

    // Auto-dismiss after 8 seconds
    setTimeout(() => { banner.classList.add('hidden'); }, 8000);
}


// ── Elapsed clock ─────────────────────────────────────────────────────────────

function tickClock() {
    const el = document.getElementById('elapsed-time');
    if (!el) return;
    const s    = Math.floor((Date.now() - _missionStartMs) / 1000);
    const mins = Math.floor(s / 60).toString().padStart(2, '0');
    const secs = (s % 60).toString().padStart(2, '0');
    el.textContent = `${mins}:${secs}`;
}

// Reset complete flag when Start is pressed so banner can fire again next run
async function sendControl(action) {
    const fb = document.getElementById('control-feedback');
    if (fb) fb.textContent = '...';

    if (action === 'start') {
        _missionCompleteFired = false;
        _missionStartMs = Date.now();
    }
    if (action === 'reset') {
        _missionCompleteFired = false;
        _flowerCache = [];
        const banner = document.getElementById('mission-complete-banner');
        if (banner) banner.classList.add('hidden');
    }

    try {
        const resp = await fetch('/api/control', {
            method:  'POST',
            headers: { 'Content-Type': 'application/json' },
            body:    JSON.stringify({ action }),
        });
        const data = await resp.json();
        if (fb) {
            fb.textContent = data.message || 'Done';
            setTimeout(() => { if (fb) fb.textContent = ''; }, 3000);
        }
    } catch (e) {
        if (fb) fb.textContent = 'Error: ' + e.message;
    }
}


// ── Init ──────────────────────────────────────────────────────────────────────

connectSSE();
setInterval(tickClock, 1000);
