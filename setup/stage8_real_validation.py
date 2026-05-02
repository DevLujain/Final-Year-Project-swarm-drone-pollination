#!/usr/bin/env python3
"""
Stage 8 — Real Footage Validation (OBB-corrected)
===================================================
Uses r.obb instead of r.boxes for YOLOv8s-OBB model.
"""
from ultralytics import YOLO
import cv2, os, csv, json
from datetime import datetime

VIDEO_PATH  = os.path.expanduser('~/Desktop/FYP/real_footage/sunflower3.mp4')
MODEL_PATH  = os.path.expanduser('~/Desktop/FYP/fyp_training/runs/sunflower_fyp_v2/weights/best.pt')
OUTPUT_DIR  = os.path.expanduser('~/Desktop/FYP/fyp_training/real_validation')
FRAME_SKIP  = 5
CONF_THRESH = 0.01

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(f'{OUTPUT_DIR}/annotated', exist_ok=True)

print("╔══════════════════════════════════════════════════════╗")
print("║  Stage 8 — Real Footage Validation (OBB)            ║")
print("║  FYP: Autonomous Swarm Drone Pollination             ║")
print("╚══════════════════════════════════════════════════════╝")

model = YOLO(MODEL_PATH, task='obb')
print("✓ Model loaded — YOLOv8s-OBB")

cap   = cv2.VideoCapture(VIDEO_PATH)
fps   = cap.get(cv2.CAP_PROP_FPS)
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
w     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
duration = total / fps if fps > 0 else 0
print(f"Video: {w}x{h} @ {fps:.0f}fps | {total} frames | {duration:.1f}s")

results_log = []
frame_idx = processed = total_dets = 0
conf_scores = []

while True:
    ret, frame = cap.read()
    if not ret: break
    if frame_idx % FRAME_SKIP == 0:
        results = model.predict(frame, conf=CONF_THRESH, task='obb', verbose=False)
        r = results[0]
        n_dets = len(r.obb) if r.obb is not None else 0
        confs  = r.obb.conf.tolist() if r.obb is not None and n_dets > 0 else []
        total_dets  += n_dets
        conf_scores += confs
        processed   += 1
        if n_dets > 0:
            annotated = r.plot()
            cv2.imwrite(f'{OUTPUT_DIR}/annotated/frame_{frame_idx:05d}_det{n_dets}.jpg', annotated)
        results_log.append({
            'frame': frame_idx,
            'timestamp': round(frame_idx / fps, 2) if fps > 0 else 0,
            'detections': n_dets,
            'mean_conf': round(sum(confs)/len(confs), 3) if confs else 0.0,
            'confidences': json.dumps([round(c,3) for c in confs]),
        })
        if processed % 10 == 0:
            print(f"  Frame {frame_idx}: {n_dets} dets")
    frame_idx += 1

cap.release()

frames_with_dets = sum(1 for r in results_log if r['detections'] > 0)
detection_rate   = frames_with_dets / processed * 100 if processed > 0 else 0
mean_conf_all    = sum(conf_scores) / len(conf_scores) if conf_scores else 0
max_dets         = max((r['detections'] for r in results_log), default=0)

csv_path = f'{OUTPUT_DIR}/real_validation_obb_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
with open(csv_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['frame','timestamp','detections','mean_conf','confidences'])
    writer.writeheader()
    writer.writerows(results_log)

print(f"\n╔══════════════════════════════════════════════════════╗")
print(f"║  REAL VALIDATION SUMMARY                            ║")
print(f"╚══════════════════════════════════════════════════════╝")
print(f"  Frames processed:       {processed}")
print(f"  Frames with detections: {frames_with_dets} ({detection_rate:.1f}%)")
print(f"  Total detections:       {total_dets}")
print(f"  Mean confidence:        {mean_conf_all:.3f}")
print(f"  Max dets in one frame:  {max_dets}")
print(f"  CSV: {csv_path}")
print(f"  Annotated: {OUTPUT_DIR}/annotated/")
print(f"\n  Simulation mAP@0.5:     0.855")
print(f"  Real footage mean conf: {mean_conf_all:.3f}")
gap = 0.855 - mean_conf_all
print(f"  Domain gap:             {gap:.3f}")
