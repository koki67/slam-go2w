#!/usr/bin/env python3
"""Relay Hesai PandarXT point clouds into the Velodyne XYZIRT layout LIO-SAM expects."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


VELODYNE_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=24, datatype=PointField.FLOAT32, count=1),
]


class HesaiToVelodyne(Node):
    def __init__(self):
        super().__init__("hesai_to_velodyne")
        self.sub = self.create_subscription(
            PointCloud2, "/points_raw", self._cloud_cb, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            PointCloud2, "/points_velodyne", qos_profile_sensor_data
        )

    def _cloud_cb(self, msg):
        try:
            points = list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z", "intensity", "timestamp", "ring"),
                    skip_nans=True,
                )
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to read /points_raw fields: {exc}")
            return

        if not points:
            self.pub.publish(point_cloud2.create_cloud(msg.header, VELODYNE_FIELDS, []))
            return

        min_timestamp = min(float(p[4]) for p in points)
        converted = []
        for x, y, z, intensity, timestamp, ring in points:
            rel_time = float(timestamp) - min_timestamp
            if rel_time < 0.0 or not math.isfinite(rel_time):
                rel_time = 0.0
            converted.append(
                (
                    float(x),
                    float(y),
                    float(z),
                    float(intensity),
                    int(ring),
                    rel_time,
                )
            )

        # LIO-SAM's imageProjection treats header.stamp as the scan-start absolute time
        # (timeScanCur), and per-point `time` as the offset from it. Re-anchor header.stamp
        # to the earliest point's firing time so header.stamp + point.time = absolute capture
        # time. Without this, IMU deskew is offset by the Hesai driver's header convention.
        sec = int(min_timestamp)
        nanosec = int(round((min_timestamp - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        out_header = Header()
        out_header.frame_id = msg.header.frame_id
        out_header.stamp.sec = sec
        out_header.stamp.nanosec = nanosec

        self.pub.publish(point_cloud2.create_cloud(out_header, VELODYNE_FIELDS, converted))


def main():
    rclpy.init()
    rclpy.spin(HesaiToVelodyne())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
