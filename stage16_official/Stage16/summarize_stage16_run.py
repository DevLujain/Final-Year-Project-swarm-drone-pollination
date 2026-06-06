#!/usr/bin/env python3
"""
summarize_stage16_run.py — turn a Stage 16 run's CSV logs into the
results table for the report (limitation 3 / next-step 3).

The orchestrator writes, per run, into its mission_outputs/stage16 folder:
    events_stage16_<ts>.csv        ts, day, drone_id, type, payload_json
    pollination_stage16_<ts>.csv   run_id, day, drone_id, flower_id, x, y, t
    telemetry_stage16_<ts>.csv     2 Hz state/pose/battery per drone
    mission_stage16_<ts>.txt       end-of-mission summary

USAGE
    python3 summarize_stage16_run.py                # latest run, auto-found
    python3 summarize_stage16_run.py /path/to/events_stage16_XXX.csv

It auto-searches the usual places, then falls back to scanning ~/Desktop.
"""

from __future__ import annotations
import csv, json, os, sys, glob, time

CANDIDATE_DIRS = [
    "~/Desktop/FYP/stage15_official/Stage16/mission_outputs/stage16",
    "~/Desktop/FYP/mission_outputs/stage16",
    "~/Desktop/Drone_Testing_Infrastructure/mission_outputs/stage16",
]

def find_events_csv() -> str | None:
    hits = []
    for d in CANDIDATE_DIRS:
        hits += glob.glob(os.path.expanduser(d + "/events_stage16_*.csv"))
    if not hits:
        # last resort: scan Desktop (can take a few seconds)
        for root, _dirs, files in os.walk(os.path.expanduser("~/Desktop")):
            for f in files:
                if f.startswith("events_stage16_") and f.endswith(".csv"):
                    hits.append(os.path.join(root, f))
    if not hits:
        return None
    return max(hits, key=os.path.getmtime)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else find_events_csv()
    if not path or not os.path.isfile(path):
        print("No events_stage16_*.csv found.")
        print("Locate it manually with:")
        print("  find ~/Desktop -name 'events_stage16_*.csv' -printf '%T@ %p\\n' | sort -n | tail -3")
        sys.exit(1)

    print(f"run log: {path}\n")

    days: dict[int, dict] = {}
    totals = {"pollinated": 0, "duration_s": None}

    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                etype = row["type"]
                day = int(float(row["day"])) if row["day"] != "" else -1
                ts = float(row["ts"])
                payload = json.loads(row["payload_json"] or "{}")
            except Exception:
                continue
            d = days.setdefault(day, {"obs": {}, "poll": 0})
            if etype == "day_start":
                d["t_start"] = ts
                d["n_targets_truth"] = payload.get("n_targets")
            elif etype == "survey_done":
                d["obs"][row.get("drone_id", "?")] = payload.get("n_observed")
            elif etype == "merge_complete":
                d["unique"] = payload.get("unique_targets")
            elif etype == "landmark_map":
                d["lm_known"] = payload.get("known", payload.get("n_known"))
                d["lm_new"] = payload.get("new_today", payload.get("n_new"))
                d["lm_done"] = payload.get("already_pollinated", payload.get("n_pollinated"))
                d["lm_routing"] = payload.get("routing", payload.get("n_routing"))
            elif etype == "routes_assigned":
                d["routes"] = payload.get("route_counts")
            elif etype == "pollination_success":
                d["poll"] += 1
            elif etype == "day_complete":
                d["t_end"] = ts
                d["poll_reported"] = payload.get("pollinated_today")
            elif etype == "mission_complete":
                totals["pollinated"] = payload.get("total_pollinated")
                totals["duration_s"] = payload.get("duration_s")

    print(f"{'day':>4} {'observed/drone':>16} {'unique':>7} "
          f"{'landmarks(new/done)':>20} {'routed':>7} {'pollinated':>11} {'duration':>9}")
    print("-" * 82)
    grand_poll = 0
    for day in sorted(k for k in days if k >= 0):
        d = days[day]
        obs = "+".join(str(v) for v in d["obs"].values()) if d["obs"] else "-"
        lm = (f"{d.get('lm_known','-')} ({d.get('lm_new','-')}/"
              f"{d.get('lm_done','-')})") if "lm_known" in d else "-"
        routed = (sum(d["routes"].values()) if isinstance(d.get("routes"), dict)
                  else d.get("lm_routing", "-"))
        dur = (f"{d['t_end']-d['t_start']:.0f} s"
               if "t_start" in d and "t_end" in d else "-")
        poll = d.get("poll_reported", d["poll"])
        grand_poll += d["poll"]
        print(f"{day:>4} {obs:>16} {str(d.get('unique','-')):>7} "
              f"{lm:>20} {str(routed):>7} {str(poll):>11} {dur:>9}")

    print("-" * 82)
    if totals["duration_s"] is not None:
        print(f"mission_complete: total pollinated = {totals['pollinated']}, "
              f"duration = {totals['duration_s']} s")
    else:
        print(f"(no mission_complete event yet) pollination_success events "
              f"counted so far: {grand_poll}")

    # cross-check against the pollination CSV if present
    pcsv = path.replace("events_", "pollination_")
    if os.path.isfile(pcsv):
        with open(pcsv) as f:
            n = sum(1 for _ in f) - 1
        print(f"cross-check {os.path.basename(pcsv)}: {n} pollination rows")

if __name__ == "__main__":
    main()
