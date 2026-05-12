#!/usr/bin/env python3
import rclpy, json, csv, math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from px4_msgs.msg import VehicleLocalPosition
from pathlib import Path

REGISTRY_CSV    = Path.home() / "Desktop/FYP/setup/flower_registry.csv"
DETECTION_RANGE = 3.0
MODEL_BASE_CONF = 0.855

def load_sector_flowers(drone_id):
    if not REGISTRY_CSV.exists():
        fallback = {0:[(2.,2.),(2.,5.),(2.,8.)],
                    1:[(5.,2.),(5.,5.),(5.,8.)],
                    2:[(8.,2.),(8.,5.),(8.,8.)]}
        return fallback[drone_id]
    return [(float(r["x"]), float(r["y"]))
            for r in csv.DictReader(open(REGISTRY_CSV))
            if int(r["drone_id"]) == drone_id and r["bloom_day"] != "0"]

class FlowerDetectorSim(Node):
    def __init__(self):
        super().__init__("flower_detector_sim")
        self.declare_parameter("drone_id", 0)
        self.drone_id   = self.get_parameter("drone_id").value
        self.my_flowers = load_sector_flowers(self.drone_id)
        self.detected   = set()
        self.create_subscription(VehicleLocalPosition,
            f"/px4_{self.drone_id}/fmu/out/vehicle_local_position",
            self._pos_cb, 10)
        self.targets_pub = self.create_publisher(PoseArray,
            f"/drone_{self.drone_id}/sunflower_targets", 10)
        self.det_pub = self.create_publisher(String,
            f"/drone_{self.drone_id}/yolov8/detections", 10)
        self.get_logger().info(
            f"Drone {self.drone_id}: {len(self.my_flowers)} flowers loaded. Range: {DETECTION_RANGE}m")

    def _pos_cb(self, msg):
        dx, dy = float(msg.x), float(msg.y)
        detections, poses = [], PoseArray()
        for idx, (fx, fy) in enumerate(self.my_flowers):
            dist = math.sqrt((dx-fx)**2 + (dy-fy)**2)
            if dist <= DETECTION_RANGE:
                p = Pose()
                p.position.x = fx
                p.position.y = fy
                p.position.z = 0.0
                poses.poses.append(p)
                conf = float(np.clip(
                    MODEL_BASE_CONF - 0.15*(dist/DETECTION_RANGE) + np.random.normal(0, 0.03),
                    0.5, 1.0))
                detections.append({"flower_id": idx, "flower_x": fx, "flower_y": fy,
                                   "distance": round(dist,2), "conf": round(conf,4)})
                if idx not in self.detected:
                    self.detected.add(idx)
                    self.get_logger().info(
                        f"Drone {self.drone_id}: flower {idx} at ({fx:.1f},{fy:.1f}) "
                        f"dist={dist:.1f}m conf={conf:.3f}")
        if poses.poses:
            self.targets_pub.publish(poses)
        if detections:
            m = String()
            m.data = json.dumps(detections)
            self.det_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = FlowerDetectorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
