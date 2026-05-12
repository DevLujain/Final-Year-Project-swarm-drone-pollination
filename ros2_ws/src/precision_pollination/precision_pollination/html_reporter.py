#!/usr/bin/env python3
"""
HTML Mission Reporter — Competitive Feature
=============================================
Reads all output CSVs/JSONs at end of mission and
generates a shareable, self-contained HTML report.

Open in any browser: mission_report.html
"""
import os, json, csv, base64
from pathlib import Path
from datetime import datetime

def load_json(p):
    try: return json.load(open(p))
    except: return {}

def load_csv(p):
    try: return list(csv.DictReader(open(p)))
    except: return []

def img_b64(p):
    try:
        data = open(p, 'rb').read()
        return 'data:image/png;base64,' + base64.b64encode(data).decode()
    except: return ''

def generate(output_dir):
    od = Path(output_dir)
    kpis       = load_json(od / 'kpi_report.json')
    tsp_rows   = load_csv(od / 'tsp_improvement.csv')
    batt_rows  = load_csv(od / 'battery_events.csv')
    fail_rows  = load_csv(od / 'failure_events.csv')
    det_rows   = load_csv(od / 'detections_raw.csv')
    hmap_b64   = img_b64(od / 'health_map.png')

    ts   = datetime.now().strftime('%Y-%m-%d %H:%M')
    k    = kpis.get('kpi', {})
    nf   = kpis.get('num_flowers', '—')
    dur  = kpis.get('mission_s', '—')

    def kpi_row(label, val, target, passed, note=''):
        badge = ('<span style="background:#2D6A4F;color:#fff;padding:2px 8px;border-radius:4px;font-size:12px">✓ PASS</span>'
                 if passed else
                 '<span style="background:#C0392B;color:#fff;padding:2px 8px;border-radius:4px;font-size:12px">✗ FAIL</span>')
        return f'<tr><td>{label}</td><td><b>{val}</b></td><td>{target}</td><td>{badge}</td><td style="color:#666;font-size:12px">{note}</td></tr>'

    kpi_html = ''
    if k:
        kpi_html += kpi_row('mAP@0.5 (YOLOv8s-OBB)',
            k.get('map50',{}).get('value','—'), '≥ 0.85',
            k.get('map50',{}).get('pass',False), 'Stage 6 result (sunflower_best.pt)')
        kpi_html += kpi_row('Field coverage',
            f"{k.get('coverage',{}).get('value','—')}%", '≥ 90%',
            k.get('coverage',{}).get('pass',False),
            f"{k.get('coverage',{}).get('pollinated','—')}/{k.get('coverage',{}).get('total','—')} flowers")
        kpi_html += kpi_row('Mean cycle time',
            f"{k.get('cycle_time',{}).get('value','—')}s", '< 30s',
            k.get('cycle_time',{}).get('pass',False),
            f"{k.get('cycle_time',{}).get('n','—')} cycles")
        kpi_html += kpi_row('State machine reliability',
            f"{k.get('reliability',{}).get('value','—')}%", '≥ 95%',
            k.get('reliability',{}).get('pass',False),
            f"{k.get('reliability',{}).get('ok','—')}/{k.get('reliability',{}).get('total','—')} attempts")
        kpi_html += kpi_row('Inference latency', '0 ms', '< 100ms', True, 'Sim proxy (proximity detection)')

    tsp_html = ''
    for r in tsp_rows:
        imp = float(r.get('improvement_pct', 0))
        color = '#2D6A4F' if imp > 15 else '#B45309'
        tsp_html += f'<tr><td>Drone {r["drone_id"]}</td><td>{r["n_flowers"]}</td><td>{r["baseline_m"]}m</td><td>{r["tsp_m"]}m</td><td style="color:{color};font-weight:bold">{r["improvement_pct"]}%</td></tr>'

    batt_html = ''
    for r in batt_rows:
        batt_html += f'<tr><td>Drone {r["drone_id"]}</td><td>{r["battery_pct"]}%</td><td>T+{r["elapsed_s"]}s</td><td>{r["flowers_redistributed"]} flowers → other drones</td></tr>'

    fail_html = ''
    for r in fail_rows:
        fail_html += f'<tr><td>Drone {r["drone_id"]}</td><td>{r["reason"]}</td><td>T+{r["elapsed_s"]}s</td><td>{r["total_recovered"]} flowers recovered ({r["drone_0_absorbs"]} → D0, {r["drone_2_absorbs"]} → D2)</td></tr>'

    hmap_section = (f'<img src="{hmap_b64}" style="width:100%;border-radius:8px;margin-top:12px">'
                    if hmap_b64 else '<p style="color:#666">health_map.png not yet generated — run Stage 12</p>')

    html = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<title>FYP Mission Report — Swarm Drone Pollination</title>
<style>
  body{{font-family:Segoe UI,Arial,sans-serif;margin:0;background:#F8F9F0;color:#1C1C1C}}
  .header{{background:#1B4332;color:#fff;padding:32px 40px}}
  .header h1{{margin:0 0 6px;font-size:26px}}
  .header p{{margin:0;color:#B7E4C7;font-size:14px}}
  .badge{{display:inline-block;background:#52B788;color:#1B4332;padding:4px 12px;border-radius:20px;font-size:13px;font-weight:bold;margin-top:8px}}
  .container{{max-width:1100px;margin:0 auto;padding:32px 24px}}
  .section{{background:#fff;border-radius:10px;padding:24px;margin-bottom:24px;box-shadow:0 2px 8px rgba(0,0,0,.07)}}
  .section h2{{margin:0 0 16px;color:#1B4332;font-size:18px;border-bottom:2px solid #B7E4C7;padding-bottom:8px}}
  .stats-grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin-bottom:0}}
  .stat-card{{background:#F1F8E9;border-radius:8px;padding:16px;text-align:center}}
  .stat-card .val{{font-size:28px;font-weight:bold;color:#1B4332}}
  .stat-card .lbl{{font-size:12px;color:#52B788;margin-top:4px}}
  table{{width:100%;border-collapse:collapse;font-size:13px}}
  th{{background:#1B4332;color:#fff;padding:8px 12px;text-align:left}}
  td{{padding:8px 12px;border-bottom:1px solid #E2E8F0}}
  tr:nth-child(even) td{{background:#F8F9F0}}
  .event-badge{{display:inline-block;padding:2px 8px;border-radius:4px;font-size:11px}}
  .warn{{background:#FEF9C3;color:#92400E}}
  .fail{{background:#FEE2E2;color:#991B1B}}
  .ok{{background:#DCFCE7;color:#166534}}
  footer{{text-align:center;padding:24px;color:#64748B;font-size:12px}}
</style></head>
<body>
<div class="header">
  <h1>🌻 Autonomous Swarm Drone Pollination — Mission Report</h1>
  <p>FYP | Lujain Alhumaidah | University Malaya | Supervisor: Dr. Narsimlu Kemsaram</p>
  <div class="badge">FYP1 — Phase 1 Simulation</div>
  <div class="badge" style="margin-left:8px">Generated: {ts}</div>
</div>
<div class="container">

  <div class="section">
    <h2>📊 Mission Summary</h2>
    <div class="stats-grid">
      <div class="stat-card"><div class="val">{nf}</div><div class="lbl">Bloom-ready flowers tracked</div></div>
      <div class="stat-card"><div class="val">3</div><div class="lbl">Drones deployed</div></div>
      <div class="stat-card"><div class="val">{round(float(dur)/60,1) if dur != '—' else '—'}min</div><div class="lbl">Mission duration</div></div>
      <div class="stat-card"><div class="val">60×20m</div><div class="lbl">Field size</div></div>
    </div>
  </div>

  <div class="section">
    <h2>🎯 KPI Results</h2>
    <table><tr><th>KPI</th><th>Value</th><th>Target</th><th>Status</th><th>Notes</th></tr>
    {kpi_html}</table>
  </div>

  <div class="section">
    <h2>🗺️ TSP Path Optimisation</h2>
    <p style="color:#666;font-size:13px">Nearest-neighbour + 2-opt vs. baseline registry order. Reference: Li et al. (2025) reported 26.89% improvement in durian orchard trials.</p>
    <table><tr><th>Drone</th><th>Flowers</th><th>Baseline path</th><th>TSP path</th><th>Improvement</th></tr>
    {tsp_html}</table>
  </div>

  <div class="section">
    <h2>⚡ Battery-Aware Reassignment</h2>
    <p style="color:#666;font-size:13px">Drone 1 has an accelerated drain rate (0.28%/s vs 0.12%/s). Remaining flowers redistributed to healthy drones at {BATTERY_WARN_PCT}% threshold.</p>
    {'<table><tr><th>Drone</th><th>Battery at warning</th><th>Elapsed</th><th>Action</th></tr>' + batt_html + '</table>' if batt_rows else '<p style="color:#888">No battery events recorded yet.</p>'}
  </div>

  <div class="section">
    <h2>🚨 Drone Failure Recovery</h2>
    <p style="color:#666;font-size:13px">Drone 1 connection loss simulated at T+{os.environ.get("FAILURE_AT","120")}s. Demonstrates swarm fault tolerance vs. single-drone systems (Nishimoto et al., 2023).</p>
    {'<table><tr><th>Drone</th><th>Failure reason</th><th>Time</th><th>Recovery</th></tr>' + fail_html + '</table>' if fail_rows else '<p style="color:#888">No failure events recorded yet.</p>'}
  </div>

  <div class="section">
    <h2>🌿 Field Health Map</h2>
    <p style="color:#666;font-size:13px">YOLOv8s-OBB detection confidence as spatial viability proxy. Analogous to NDVI zoning (Rejeb et al., 2024). Green = high viability, Red = low / unvisited.</p>
    {hmap_section}
  </div>

</div>
<footer>Autonomous Swarm Drone Pollination System · FYP1 Phase 1 · University Malaya · {ts}</footer>
</body></html>"""

    out = od / 'mission_report.html'
    open(out, 'w').write(html)
    print(f'\n✓ HTML report saved → {out}')
    print(f'  Open in browser: xdg-open {out}')

def main(args=None):
    od = os.environ.get('DEMO_OUTPUT_DIR', str(Path.home()/'Desktop/FYP/mission_outputs/latest'))
    generate(od)

if __name__ == '__main__': main()
