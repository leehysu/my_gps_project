#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as RosTime

def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

class TfPub(Node):
    def __init__(self):
        super().__init__('tf_pub')
        qos = QoSProfile(depth=10)

        self.br = TransformBroadcaster(self)

        self.create_subscription(Float64, '/azimuth_f9r_deg', self.cb_f9r, qos)
        self.create_subscription(Float64, '/azimuth_f9p_deg', self.cb_f9p, qos)

    def publish_tf(self, yaw_deg, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'        # 부모 프레임
        t.child_frame_id = child_frame         # 자식 프레임
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def cb_f9r(self, msg: Float64):
        self.publish_tf(msg.data, 'f9r_frame')

    def cb_f9p(self, msg: Float64):
        self.publish_tf(msg.data, 'f9p_frame')

def main(args=None):
    rclpy.init(args=args)
    node = TfPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
