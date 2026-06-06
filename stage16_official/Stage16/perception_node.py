#!/usr/bin/env python3
"""
perception_node.py  —  Stage 16 v4
====================================
Per-drone perception with:

  * DUAL HSV detection — yellow petals + brown disc.  The brown disc
    centroid is the precise pollination target (where the pollen sits);
    the yellow blob defines "there is a flower here".

  * ONLINE WEIGHTED CLUSTERING — each detection contributes to the
    running weighted-mean position of the nearest cluster (within
    cluster_match_radius_m); if nothing matches, a new cluster is born.
    Per-detection weight = (1 - r/r_max)^2 where r is the centroid's
    pixel distance from image centre.  This makes centre-of-image
    detections (which have minimal parallax error) dominate the
    estimate while edge detections still vote for coverage.

  * SNAPSHOT PUBLISHING — every snapshot_publish_period_s, perception
    emits the entire current cluster map on /perception/drone_N/snapshot
    as one JSON message.  Downstream consumers REPLACE state on receive
    (not extend), so the latest snapshot is the source of truth.

DESIGN NOTES
------------
- Each cluster stores running weighted means for yellow_x, yellow_y,
  and (if seen) brown_x, brown_y.  When the brown disc has been seen
  at least min_brown_samples times, brown_xy is used as the target;
  otherwise the yellow centroid is the fallback.
- No explicit parallax-from-z reprojection.  Center-weighted clustering
  effectively suppresses parallax-biased edge detections, so the cluster
  centre converges to the true position as more frames accumulate.
- The drone_controller subscribes to the snapshot topic, augments each
  cluster with a measured height via TOF (height-probe phase), then
  republishes the enriched map on /survey/drone_N/observations.
"""

from __future__ import annotations

import json
import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # ── Parameters ─────────────────────────────────────────────
        self.declare_parameter('drone_id',                  0)
        self.declare_parameter('world_name',                'stage16_world')
        self.declare_parameter('camera_hfov_deg',           62.0)
        self.declare_parameter('camera_width',              640)
        self.declare_parameter('camera_height',             480)
        self.declare_parameter('flower_head_z_default',     1.7)
        # Yellow (petals) HSV
        self.declare_parameter('hsv_h_min',                 18)
        self.declare_parameter('hsv_h_max',                 36)
        self.declare_parameter('hsv_s_min',                 130)
        self.declare_parameter('hsv_v_min',                 130)
        # Brown (disc) HSV  — RGB (115, 69, 13) → H≈9, S≈225, V≈115
        self.declare_parameter('hsv_brown_h_min',           5)
        self.declare_parameter('hsv_brown_h_max',           18)
        self.declare_parameter('hsv_brown_s_min',           120)
        self.declare_parameter('hsv_brown_v_min',           60)
        self.declare_parameter('hsv_brown_v_max',           160)
        # v6: publish a cluster only if its brown disc was actually seen.
        # A yellow blob with no brown core is not a sunflower.
        self.declare_parameter('require_brown_disc',        True)
        # v6: green BUD detection (closed heads). Bud green and leaf green
        # are nearly identical in hue, so SHAPE separates them: a bud from
        # above is a compact near-circle (sphere r=head_r); a leaf is a
        # ~2.2:1 ellipse. Circularity + aspect-ratio gates do the work.
        self.declare_parameter('hsv_green_h_min',           40)
        self.declare_parameter('hsv_green_h_max',           80)
        self.declare_parameter('hsv_green_s_min',           150)
        self.declare_parameter('hsv_green_v_min',           140)
        self.declare_parameter('bud_min_area_px',           150)
        self.declare_parameter('bud_max_area_px',           20000)
        self.declare_parameter('bud_min_circularity',       0.70)
        self.declare_parameter('bud_max_aspect',            1.50)
        self.declare_parameter('min_bud_samples',           3)
        # Blob area filters (yellow contour size)
        self.declare_parameter('min_blob_area_px',          300)
        self.declare_parameter('max_blob_area_px',          60000)
        self.declare_parameter('min_brown_area_px',         40)
        # Projection sign/swap (kept for empirical adjustment)
        self.declare_parameter('image_to_world_x_sign',     -1)
        self.declare_parameter('image_to_world_y_sign',     -1)
        self.declare_parameter('image_swap_xy',             True)
        # Clustering
        self.declare_parameter('cluster_match_radius_m',    0.60)
        self.declare_parameter('center_weight_power',       2.0)
        self.declare_parameter('min_cluster_samples',       6)
        self.declare_parameter('consolidate_radius_m',      0.45)
        self.declare_parameter('min_brown_samples',         2)
        # Field bounds — reject obvious projection artefacts
        self.declare_parameter('field_size_x_m',            15.0)
        self.declare_parameter('field_size_y_m',            15.0)
        self.declare_parameter('field_margin_m',            0.5)
        # Altitudes / publishing
        self.declare_parameter('min_altitude_for_detect_m', 2.50)
        self.declare_parameter('snapshot_publish_period_s', 5.0)

        self.did = int(self.get_parameter('drone_id').value)
        self.world_name = str(self.get_parameter('world_name').value)
        hfov_deg  = float(self.get_parameter('camera_hfov_deg').value)
        self.img_w = int(self.get_parameter('camera_width').value)
        self.img_h = int(self.get_parameter('camera_height').value)
        self.flower_z = float(self.get_parameter('flower_head_z_default').value)
        self.h_min = int(self.get_parameter('hsv_h_min').value)
        self.h_max = int(self.get_parameter('hsv_h_max').value)
        self.s_min = int(self.get_parameter('hsv_s_min').value)
        self.v_min = int(self.get_parameter('hsv_v_min').value)
        self.bh_min = int(self.get_parameter('hsv_brown_h_min').value)
        self.bh_max = int(self.get_parameter('hsv_brown_h_max').value)
        self.bs_min = int(self.get_parameter('hsv_brown_s_min').value)
        self.bv_min = int(self.get_parameter('hsv_brown_v_min').value)
        self.bv_max = int(self.get_parameter('hsv_brown_v_max').value)
        self.min_area = int(self.get_parameter('min_blob_area_px').value)
        self.max_area = int(self.get_parameter('max_blob_area_px').value)
        self.min_brown_area = int(self.get_parameter('min_brown_area_px').value)
        self.sx = int(self.get_parameter('image_to_world_x_sign').value)
        self.sy = int(self.get_parameter('image_to_world_y_sign').value)
        self.swap_xy = bool(self.get_parameter('image_swap_xy').value)
        self.match_r = float(self.get_parameter('cluster_match_radius_m').value)
        self.weight_pow = float(self.get_parameter('center_weight_power').value)
        self.min_samples = int(self.get_parameter('min_cluster_samples').value)
        self.consolidate_r = float(self.get_parameter('consolidate_radius_m').value)
        self.min_brown_samples = int(self.get_parameter('min_brown_samples').value)
        self.field_x = float(self.get_parameter('field_size_x_m').value)
        self.field_y = float(self.get_parameter('field_size_y_m').value)
        self.field_margin = float(self.get_parameter('field_margin_m').value)
        self.min_alt = float(self.get_parameter('min_altitude_for_detect_m').value)
        self.require_brown = bool(self.get_parameter('require_brown_disc').value)
        self.gh_min = int(self.get_parameter('hsv_green_h_min').value)
        self.gh_max = int(self.get_parameter('hsv_green_h_max').value)
        self.gs_min = int(self.get_parameter('hsv_green_s_min').value)
        self.gv_min = int(self.get_parameter('hsv_green_v_min').value)
        self.bud_min_area = int(self.get_parameter('bud_min_area_px').value)
        self.bud_max_area = int(self.get_parameter('bud_max_area_px').value)
        self.bud_min_circ = float(self.get_parameter('bud_min_circularity').value)
        self.bud_max_aspect = float(self.get_parameter('bud_max_aspect').value)
        self.min_bud_samples = int(self.get_parameter('min_bud_samples').value)
        self.snap_period = float(self.get_parameter('snapshot_publish_period_s').value)

        # Derived
        self.hfov = math.radians(hfov_deg)
        self.fx = (self.img_w / 2.0) / math.tan(self.hfov / 2.0)
        aspect = self.img_h / self.img_w
        vfov = 2 * math.atan(math.tan(self.hfov / 2.0) * aspect)
        self.fy = (self.img_h / 2.0) / math.tan(vfov / 2.0)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        # Max pixel radius (for normalising centre weight)
        self.r_max = math.hypot(self.cx, self.cy)

        # ── State ─────────────────────────────────────────────────
        # Each cluster:
        #   {'fid', 'x', 'y', 'w_total', 'n',
        #    'brown_x', 'brown_y', 'brown_w_total', 'brown_n'}
        self.clusters: list[dict] = []
        self.pose = None
        self.bridge = CvBridge()
        self.day = -1
        self.last_snap_pub = 0.0
        self._next_fid_counter = 0
        # v6: separate bud map — counted/reported only, never routed
        self.bud_clusters: list[dict] = []

        # ── QoS ───────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)

        # ── Topics ─────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, f'/model/drone_{self.did}/pose',
            self._on_pose, sensor_qos)
        self.create_subscription(
            Image, f'/drone_{self.did}/camera/image_raw',
            self._on_image, sensor_qos)
        self.create_subscription(
            Int32, '/bloom/current_day',
            self._on_day, 10)

        # Snapshot topic — full cluster map.  Drone_controller (not
        # orchestrator) subscribes; controller enriches with TOF heights
        # and republishes on /survey/drone_N/observations.
        self.pub_snap = self.create_publisher(
            String, f'/perception/drone_{self.did}/snapshot', 10)
        self.pub_dbg = self.create_publisher(
            Image, f'/perception/drone_{self.did}/debug_image', 10)

        # 1 Hz snapshot timer (or whatever snap_period is)
        self.create_timer(self.snap_period, self._publish_snapshot)

        self.get_logger().info(
            f'perception_node[{self.did}] v6 up [brown-gate+bud-shape]. '
            f'hfov={hfov_deg:.1f}°  fx={self.fx:.1f}px  '
            f'cluster_match_r={self.match_r:.2f}m  '
            f'min_samples={self.min_samples}  weight_p={self.weight_pow}  '
            f'snap_period={self.snap_period}s'
        )

    # ── Callbacks ──────────────────────────────────────────────────
    def _on_pose(self, m: PoseStamped):
        q = m.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.pose = (m.pose.position.x, m.pose.position.y,
                     m.pose.position.z, yaw)

    def _on_day(self, m: Int32):
        new_day = int(m.data)
        if new_day == self.day:
            return
        self.day = new_day
        # On a new day, the clusters carry over (we don't forget what
        # we've already seen).  The snapshot publisher will keep the
        # orchestrator informed.

    def _on_image(self, m: Image):
        if self.pose is None:
            return
        dx, dy, dz, yaw = self.pose
        if dz < self.min_alt:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'cv_bridge: {e}')
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Yellow petals mask
        mask_y = cv2.inRange(
            hsv, (self.h_min, self.s_min, self.v_min),
                 (self.h_max, 255,        255))
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_CLOSE, kern)

        # Brown disc mask
        mask_b = cv2.inRange(
            hsv, (self.bh_min, self.bs_min, self.bv_min),
                 (self.bh_max, 255,         self.bv_max))
        mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_CLOSE, kern)

        # Find yellow contours (each = one flower's petals)
        contours, _ = cv2.findContours(
            mask_y, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dbg = img.copy()
        height_above = dz - self.flower_z

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            yc_x = M['m10'] / M['m00']
            yc_y = M['m01'] / M['m00']

            # ── Brown disc inside this yellow blob ──
            # ROI = bounding box of the yellow contour; intersect with
            # the brown mask; find the largest blob within.
            x0, y0, w0, h0 = cv2.boundingRect(cnt)
            x1, y1 = x0 + w0, y0 + h0
            x0c, y0c = max(0, x0), max(0, y0)
            x1c, y1c = min(self.img_w, x1), min(self.img_h, y1)
            roi_brown = mask_b[y0c:y1c, x0c:x1c]
            brown_yc_x = brown_yc_y = None
            if roi_brown.size > 0:
                bcontours, _ = cv2.findContours(
                    roi_brown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # Pick the brown contour with biggest area
                best_bcnt = None
                best_barea = 0.0
                for bc in bcontours:
                    ba = cv2.contourArea(bc)
                    if ba >= self.min_brown_area and ba > best_barea:
                        best_barea = ba
                        best_bcnt = bc
                if best_bcnt is not None:
                    Mb = cv2.moments(best_bcnt)
                    if Mb['m00'] != 0:
                        brown_yc_x = (Mb['m10'] / Mb['m00']) + x0c
                        brown_yc_y = (Mb['m01'] / Mb['m00']) + y0c

            # ── Project yellow centroid to world ──
            wx, wy = self._project(yc_x, yc_y, dx, dy, dz, yaw, height_above)
            if wx is None:
                continue

            # Reject detections outside field bounds
            if (wx < -self.field_margin
                    or wx > self.field_x + self.field_margin
                    or wy < -self.field_margin
                    or wy > self.field_y + self.field_margin):
                cv2.circle(dbg, (int(yc_x), int(yc_y)), 7, (0, 0, 255), 1)
                continue

            # ── Center-of-image weight ──
            r_pix = math.hypot(yc_x - self.cx, yc_y - self.cy)
            w_yellow = max(0.0, (1.0 - r_pix / self.r_max)) ** self.weight_pow
            if w_yellow < 0.01:
                w_yellow = 0.01      # don't let weights vanish entirely

            # ── Project brown centroid (if found) ──
            brown_wx = brown_wy = None
            w_brown = 0.0
            if brown_yc_x is not None and brown_yc_y is not None:
                brown_wx, brown_wy = self._project(
                    brown_yc_x, brown_yc_y, dx, dy, dz, yaw, height_above)
                if brown_wx is not None:
                    if (brown_wx < -self.field_margin
                            or brown_wx > self.field_x + self.field_margin
                            or brown_wy < -self.field_margin
                            or brown_wy > self.field_y + self.field_margin):
                        brown_wx = brown_wy = None
                    else:
                        r_pix_b = math.hypot(
                            brown_yc_x - self.cx, brown_yc_y - self.cy)
                        w_brown = max(0.0,
                            (1.0 - r_pix_b / self.r_max)) ** self.weight_pow
                        if w_brown < 0.01:
                            w_brown = 0.01

            # ── Cluster update ──
            cluster = self._update_cluster(
                wx, wy, w_yellow, brown_wx, brown_wy, w_brown)

            # Draw debug
            colour = (0, 255, 0) if cluster['n'] >= self.min_samples else (255, 200, 0)
            cv2.circle(dbg, (int(yc_x), int(yc_y)), 12, colour, 2)
            cv2.putText(dbg, f"F{cluster['fid']} n={cluster['n']}",
                        (int(yc_x) + 14, int(yc_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
            if brown_yc_x is not None:
                cv2.circle(dbg, (int(brown_yc_x), int(brown_yc_y)),
                           6, (139, 69, 19), 2)

        # ── v6: green BUD detection (shape-gated) ──
        mask_g = cv2.inRange(
            hsv, (self.gh_min, self.gs_min, self.gv_min),
                 (self.gh_max, 255, 255))
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_CLOSE, kern)
        gcontours, _ = cv2.findContours(
            mask_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for gc in gcontours:
            garea = cv2.contourArea(gc)
            if garea < self.bud_min_area or garea > self.bud_max_area:
                continue
            peri = cv2.arcLength(gc, True)
            if peri <= 0:
                continue
            circularity = 4.0 * math.pi * garea / (peri * peri)
            if circularity < self.bud_min_circ:
                continue                      # leaf / stem: not compact
            if len(gc) >= 5:
                (_, _), (MA, ma), _ = cv2.fitEllipse(gc)
                lo = max(min(MA, ma), 1e-6)
                if (max(MA, ma) / lo) > self.bud_max_aspect:
                    continue                  # elongated: a leaf, not a bud
            Mg = cv2.moments(gc)
            if Mg['m00'] == 0:
                continue
            gx_px = Mg['m10'] / Mg['m00']
            gy_px = Mg['m01'] / Mg['m00']
            gwx, gwy = self._project(gx_px, gy_px, dx, dy, dz, yaw,
                                     height_above)
            if gwx is None:
                continue
            if (gwx < -self.field_margin
                    or gwx > self.field_x + self.field_margin
                    or gwy < -self.field_margin
                    or gwy > self.field_y + self.field_margin):
                continue
            self._update_bud_cluster(gwx, gwy)
            cv2.circle(dbg, (int(gx_px), int(gy_px)), 10, (0, 200, 0), 2)
            cv2.putText(dbg, "BUD", (int(gx_px) + 12, int(gy_px)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)

        # Publish debug image
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            dbg_msg.header = m.header
            self.pub_dbg.publish(dbg_msg)
        except Exception:
            pass

    # ── Online weighted clustering ─────────────────────────────────
    def _update_cluster(self, wx, wy, w_yellow,
                        brown_wx, brown_wy, w_brown) -> dict:
        # Find nearest existing cluster within match radius
        best = None
        best_d = self.match_r
        for c in self.clusters:
            # Use brown center if available, else yellow
            cx_ref = c['brown_x'] if c['brown_n'] > 0 else c['x']
            cy_ref = c['brown_y'] if c['brown_n'] > 0 else c['y']
            d = math.hypot(wx - cx_ref, wy - cy_ref)
            if d < best_d:
                best = c
                best_d = d

        if best is None:
            self._next_fid_counter += 1
            fid = (self.did + 1) * 100000 + self._next_fid_counter
            new_c = {
                'fid':           fid,
                'x':             wx,
                'y':             wy,
                'w_total':       w_yellow,
                'n':             1,
                'brown_x':       brown_wx if brown_wx is not None else wx,
                'brown_y':       brown_wy if brown_wy is not None else wy,
                'brown_w_total': w_brown,
                'brown_n':       1 if brown_wx is not None else 0,
                'measured_z':    None,
            }
            self.clusters.append(new_c)
            return new_c

        # Update existing cluster (weighted running mean)
        nt = best['w_total'] + w_yellow
        best['x'] = (best['x'] * best['w_total'] + wx * w_yellow) / nt
        best['y'] = (best['y'] * best['w_total'] + wy * w_yellow) / nt
        best['w_total'] = nt
        best['n'] += 1

        if brown_wx is not None:
            if best['brown_n'] == 0:
                best['brown_x'] = brown_wx
                best['brown_y'] = brown_wy
                best['brown_w_total'] = w_brown
                best['brown_n'] = 1
            else:
                bt = best['brown_w_total'] + w_brown
                best['brown_x'] = (
                    best['brown_x'] * best['brown_w_total']
                    + brown_wx * w_brown) / bt
                best['brown_y'] = (
                    best['brown_y'] * best['brown_w_total']
                    + brown_wy * w_brown) / bt
                best['brown_w_total'] = bt
                best['brown_n'] += 1
        return best

    def _update_bud_cluster(self, wx, wy):
        """v6: simple running-mean clustering for green buds. Buds are
        counted and reported only — never routed, never pollinated."""
        best = None
        best_d = self.match_r
        for b in self.bud_clusters:
            d = math.hypot(wx - b['x'], wy - b['y'])
            if d < best_d:
                best, best_d = b, d
        if best is None:
            self.bud_clusters.append({'x': wx, 'y': wy, 'n': 1})
            return
        best['x'] = (best['x'] * best['n'] + wx) / (best['n'] + 1)
        best['y'] = (best['y'] * best['n'] + wy) / (best['n'] + 1)
        best['n'] += 1

    def _bud_census(self):
        """Confirmed, consolidated bud positions."""
        cands = [b for b in self.bud_clusters
                 if b['n'] >= self.min_bud_samples]
        cands.sort(key=lambda b: b['n'], reverse=True)
        kept = []
        for b in cands:
            if any(math.hypot(b['x'] - k['x'], b['y'] - k['y'])
                   < self.consolidate_r for k in kept):
                continue
            kept.append(b)
        return kept

    # ── Projection (unchanged from v3 — center-weighting handles parallax) ──
    def _project(self, px, py, dx, dy, dz, yaw, h):
        if h <= 0.1:
            return None, None
        u = (px - self.cx) / self.fx
        v = (py - self.cy) / self.fy
        if self.swap_xy:
            cam_local_x = self.sx * v * h
            cam_local_y = self.sy * u * h
        else:
            cam_local_x = self.sx * u * h
            cam_local_y = self.sy * v * h
        c, s = math.cos(yaw), math.sin(yaw)
        wx = dx + c * cam_local_x - s * cam_local_y
        wy = dy + s * cam_local_x + c * cam_local_y
        return wx, wy

    # ── Snapshot publisher ─────────────────────────────────────────
    def _publish_snapshot(self):
        # v5: consolidate split clusters before emitting. A single
        # physical flower can spawn >1 cluster (seen at different survey
        # altitudes, where the fixed-height projection places it slightly
        # differently). Keep the strongest cluster at each spot and drop
        # weaker clusters within consolidate_radius_m of an already-kept
        # one. The radius is well below the 0.8 m planting spacing, so two
        # DISTINCT flowers are never merged.
        cands = [c for c in self.clusters if c['n'] >= self.min_samples]
        # v6 BROWN-DISC GATE: a sunflower has a brown core. Clusters whose
        # disc was never confirmed (brown_n < min_brown_samples) are glare,
        # brush tips, or split artefacts — drop them before publishing.
        n_yellow_only = 0
        if self.require_brown:
            n_before = len(cands)
            cands = [c for c in cands
                     if c['brown_n'] >= self.min_brown_samples]
            n_yellow_only = n_before - len(cands)
        cands.sort(key=lambda c: c['n'], reverse=True)
        kept = []
        for c in cands:
            use_brown = c['brown_n'] >= self.min_brown_samples
            tx = c['brown_x'] if use_brown else c['x']
            ty = c['brown_y'] if use_brown else c['y']
            if any(math.hypot(tx - k['_tx'], ty - k['_ty']) < self.consolidate_r
                   for k in kept):
                continue
            kept.append({'_tx': tx, '_ty': ty, '_use_brown': use_brown, '_c': c})

        out = []
        for k in kept:
            c = k['_c']
            out.append({
                'flower_id':       int(c['fid']),
                'x':               round(float(k['_tx']), 3),
                'y':               round(float(k['_ty']), 3),
                'z':               (round(float(c['measured_z']), 3)
                                    if c['measured_z'] is not None
                                    else self.flower_z),
                'yellow_x':        round(float(c['x']), 3),
                'yellow_y':        round(float(c['y']), 3),
                'brown_seen':      k['_use_brown'],
                'n_samples':       int(c['n']),
                'brown_samples':   int(c['brown_n']),
                'state':           'prime_target',
                'observed_by':     self.did,
                'observed_at':     round(time.time(), 3),
            })
        # v6: bud census rides along under a SEPARATE key. Downstream
        # consumers (controller height-probe, orchestrator routing) read
        # only 'observations', so buds are displayed but NEVER routed.
        buds = self._bud_census()
        msg = String()
        msg.data = json.dumps(
            {'drone_id': self.did,
             'snapshot_ts': round(time.time(), 3),
             'n_clusters': len(out),
             'observations': out,
             'n_buds': len(buds),
             'bud_observations': [
                 {'x': round(float(b['x']), 3),
                  'y': round(float(b['y']), 3),
                  'n_samples': int(b['n'])} for b in buds]},
            separators=(',', ':'))
        self.pub_snap.publish(msg)
        if out or buds:
            self.get_logger().info(
                f'[v6 brown-gate] snapshot: {len(out)} brown-confirmed '
                f'flower(s) ({n_yellow_only} yellow-only dropped), '
                f'{len(buds)} green bud(s)')


def main(args=None):
    rclpy.init(args=args)
    n = PerceptionNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
