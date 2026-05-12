"""
app.py  —  Stage 13 DaaS Dashboard
====================================
Flask server. Unchanged except Start always triggers simulation.
"""

import json
import time
import threading
from flask import Flask, render_template, jsonify, request, Response

from mission_state import MissionState
from ros_bridge    import start_ros_bridge

app   = Flask(__name__)
state = MissionState()

threading.Thread(target=start_ros_bridge, args=(state,), daemon=True).start()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/state')
def get_state():
    return jsonify(state.get_snapshot())


@app.route('/api/stream')
def stream():
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
        state.start_simulation()
        return jsonify({'status': 'ok',
                        'message': f'3-day mission started — '
                                   f'{state.total_field_flowers if hasattr(state, "total_field_flowers") else len(state.flowers)} flowers in field.'})
    elif action == 'stop':
        state.stop_simulation()
        return jsonify({'status': 'ok', 'message': 'Mission stopped.'})
    elif action == 'reset':
        state.reset()
        return jsonify({'status': 'ok',
                        'message': f'Reset — new field: {len(state.flowers)} flowers.'})

    return jsonify({'status': 'error', 'message': f'Unknown: {action}'}), 400


if __name__ == '__main__':
    print()
    print('╔══════════════════════════════════════════════════════╗')
    print('║  Stage 13 — DaaS Dashboard  (3-day mission)        ║')
    print('╠══════════════════════════════════════════════════════╣')
    print(f'║  Field: {len(state.flowers)} flowers generated                  ║')
    print('║  Open:  http://localhost:5000                       ║')
    print('╚══════════════════════════════════════════════════════╝')
    print()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
