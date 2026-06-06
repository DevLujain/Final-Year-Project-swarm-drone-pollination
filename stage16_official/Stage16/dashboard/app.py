"""
app.py  —  Stage 16 dashboard (lite, no-cheat)
================================================
Flask app exposing:
  GET  /                     dashboard page
  GET  /snapshot             current MissionState as JSON   (poll fallback)
  GET  /stream               Server-Sent Events  (0.5 Hz)
  POST /api/reset            clear local state (does NOT command drones)

NOTE:  The Start button in Stage 13 used to launch the built-in
simulation engine; in Stage 16 that engine is gone — the real
orchestrator drives the mission.  The Start button now just RESETS
the dashboard's view of state (useful when re-running tests).

Reads env vars STAGE16_DASH_PORT / STAGE16_DASH_HOST  for binding.
"""

from __future__ import annotations

import json
import os
import time

from flask import Flask, Response, jsonify, render_template, request

from mission_state import MissionState
import ros_bridge


# ── App + state ────────────────────────────────────────────────────
app = Flask(__name__,
            static_folder='static',
            template_folder='templates')

state = MissionState()
bridge_node = None


# ── Routes ─────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/snapshot')
def snapshot():
    return jsonify(state.get_snapshot())


@app.route('/stream')
def stream():
    def gen():
        while True:
            snap = state.get_snapshot()
            yield f"event: snapshot\ndata: {json.dumps(snap)}\n\n"
            time.sleep(0.5)
    return Response(gen(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/api/reset', methods=['POST'])
def api_reset():
    state.reset()
    return jsonify({'ok': True, 'message': 'state reset'})


@app.route('/healthz')
def healthz():
    return jsonify({'ok': True,
                    'connected_to_ros': state.connected_to_ros,
                    'uptime_s': round(time.time() - state.mission_start_time, 1)})


# ── ROS bridge startup ─────────────────────────────────────────────
def _start_bridge_once():
    global bridge_node
    if bridge_node is not None:
        return
    try:
        bridge_node = ros_bridge.start(state)
        print('[app] ROS 2 bridge started')
    except Exception as e:
        print(f'[app] ROS 2 bridge failed to start: {e}')


def main():
    host = os.environ.get('STAGE16_DASH_HOST', '127.0.0.1')
    port = int(os.environ.get('STAGE16_DASH_PORT', 5000))
    _start_bridge_once()
    # threaded=True allows SSE stream + snapshot polling concurrently;
    # use_reloader=False prevents double-init of rclpy in dev mode.
    print(f'[app] serving Stage 16 dashboard on http://{host}:{port}')
    app.run(host=host, port=port, debug=False,
            threaded=True, use_reloader=False)


if __name__ == '__main__':
    main()
