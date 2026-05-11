#!/usr/bin/env python3
"""Relay /Odometry from FAST-LIO with orientation corrected from IMU frame to base_link frame.

The IMU (FAST-LIO body frame) is mounted at +90 deg yaw from the robot base_link.
Multiplying the output quaternion by Rz(-90 deg) converts body orientation to base_link.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

_CZ = math.sin(-math.pi / 4)   # z component of Rz(-90 deg) quaternion
_CW = math.cos(-math.pi / 4)   # w component


def _qmul(ax, ay, az, aw, bx, by, bz, bw):
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


class OdomBodyToBaselink(Node):
    def __init__(self):
        super().__init__('odom_body_to_baselink')
        self.sub = self.create_subscription(Odometry, '/Odometry', self._cb, 10)
        self.pub = self.create_publisher(Odometry, '/Odometry_corrected', 10)

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        x, y, z, w = _qmul(q.x, q.y, q.z, q.w, 0.0, 0.0, _CZ, _CW)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_link'
        out.pose = msg.pose
        out.pose.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        out.twist = msg.twist
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(OdomBodyToBaselink())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
