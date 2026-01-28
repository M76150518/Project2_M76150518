import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def normalize_quat(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def quat_to_euler_zyx(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class JointController(Node):
    def __init__(self):
        super().__init__('gimbal_joint_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.euler_sub = self.create_subscription(Vector3, '/cmd_euler_rpy', self.on_euler, 10)
        self.quat_sub = self.create_subscription(Quaternion, '/cmd_quaternion', self.on_quat, 10)

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_state)
        self.get_logger().info("JointController running. Listening on /cmd_euler_rpy and /cmd_quaternion")

    def on_euler(self, msg: Vector3):
        self.roll = msg.x
        self.pitch = clamp(msg.y, -math.pi/2, math.pi/2)
        self.yaw = msg.z

        if abs(abs(self.pitch) - math.pi/2) < math.radians(2.0):
            self.get_logger().warn("Near gimbal lock: pitch ≈ ±90°. Yaw and roll become coupled.")

    def on_quat(self, msg: Quaternion):
        qx, qy, qz, qw = normalize_quat(msg.x, msg.y, msg.z, msg.w)
        roll, pitch, yaw = quat_to_euler_zyx(qx, qy, qz, qw)

        self.roll = roll
        self.pitch = clamp(pitch, -math.pi/2, math.pi/2)
        self.yaw = yaw

        if abs(abs(self.pitch) - math.pi/2) < math.radians(2.0):
            self.get_logger().warn("Quat->Euler near gimbal lock: Euler may jump, quaternion is well-defined.")

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['yaw_joint', 'pitch_joint', 'roll_joint']
        js.position = [self.yaw, self.pitch, self.roll]
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = JointController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def normalize_quat(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def quat_to_euler_zyx(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class JointController(Node):
    def __init__(self):
        super().__init__('gimbal_joint_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.euler_sub = self.create_subscription(Vector3, '/cmd_euler_rpy', self.on_euler, 10)
        self.quat_sub = self.create_subscription(Quaternion, '/cmd_quaternion', self.on_quat, 10)

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_state)
        self.get_logger().info("JointController running. Listening on /cmd_euler_rpy and /cmd_quaternion")

    def on_euler(self, msg: Vector3):
        self.roll = msg.x
        self.pitch = clamp(msg.y, -math.pi/2, math.pi/2)
        self.yaw = msg.z

        if abs(abs(self.pitch) - math.pi/2) < math.radians(2.0):
            self.get_logger().warn("Near gimbal lock: pitch ≈ ±90°. Yaw and roll become coupled.")

    def on_quat(self, msg: Quaternion):
        qx, qy, qz, qw = normalize_quat(msg.x, msg.y, msg.z, msg.w)
        roll, pitch, yaw = quat_to_euler_zyx(qx, qy, qz, qw)

        self.roll = roll
        self.pitch = clamp(pitch, -math.pi/2, math.pi/2)
        self.yaw = yaw

        if abs(abs(self.pitch) - math.pi/2) < math.radians(2.0):
            self.get_logger().warn("Quat->Euler near gimbal lock: Euler may jump, quaternion is well-defined.")

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['yaw_joint', 'pitch_joint', 'roll_joint']
        js.position = [self.yaw, self.pitch, self.roll]
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = JointController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
