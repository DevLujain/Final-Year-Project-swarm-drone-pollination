"""
app.py  —  Stage 13 DaaS Dashboard
====================================
Flask server.  Single entry point.

Endpoints:
  GET  /              dashboard HTML
  GET  /api/stream    SSE live stream (1 Hz)
  GET  /api/state     one-shot JSON snapshot
  POST /api/control   action: start | stop | reset
"""

import json
import time
import threading
from flask import Flask, render_template, jsonify, request, Response

from mission_state import MissionState
from ros_bridge    import start_ros_bridge

app   = Flask(__name__)
state = MissionState()

# ROS 2 reader runs in daemon thread — does not block Flask
_ros_thread = threading.Thread(
    target=start_ros_bridge, args=(state,), daemon=True)
_ros_thread.start()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/state')
def get_state():
    return jsonify(state.get_snapshot())


@app.route('/api/stream')
def stream():
    """Server-Sent Events — browser connects once, server pushes JSON every 0.5 s."""
    def gen():
        while True:
            yield f"data: {json.dumps(state.get_snapshot())}\n\n"
            time.sleep(0.5)

    return Response(gen(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/api/control', methods=['POST'])
def control():
    body   = request.get_json(silent=True) or {}
    action = body.get('action', '')

    if action == 'start':
        # Always launch the simulation engine.
        # If real ROS 2 data starts flowing, ros_bridge overrides automatically.
        state.start_simulation()
        return jsonify({'status': 'ok',
                        'message': f'Mission started — {state.total_flowers} target flowers.'})

    elif action == 'stop':
        state.stop_simulation()
        return jsonify({'status': 'ok', 'message': 'Mission stopped.'})

    elif action == 'reset':
        state.reset()
        return jsonify({'status': 'ok',
                        'message': f'Reset — new field: {state.total_flowers} ready flowers.'})

    return jsonify({'status': 'error', 'message': f'Unknown action: {action}'}), 400


if __name__ == '__main__':
    print()
    print('╔══════════════════════════════════════════════════════╗')
    print('║  Stage 13 — DaaS Dashboard                         ║')
    print('║  Autonomous Swarm Drone Pollination — FYP           ║')
    print('╠══════════════════════════════════════════════════════╣')
    print(f'║  Field: {state.total_flowers} ready flowers generated          ║')
    print('║  Open:  http://localhost:5000                       ║')
    print('║  Stop:  Ctrl+C                                      ║')
    print('╚══════════════════════════════════════════════════════╝')
    print()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
