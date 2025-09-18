#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam)*math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

class AzimuthPub(Node):
    def __init__(self):
        super().__init__('azimuth_pub')

        # Publisher 2개 (f9r용, f9p용)
        self.pub_f9r = self.create_publisher(Float64, '/azimuth_f9r_deg', 10)
        self.pub_f9p = self.create_publisher(Float64, '/azimuth_f9p_deg', 10)

        # 구독자 2개 (f9r, f9p)
        self.sub_f9r = self.create_subscription(NavSatFix, '/f9r/fix', self.on_f9r, 10)
        self.sub_f9p = self.create_subscription(NavSatFix, '/f9p/fix', self.on_f9p, 10)

        self.prev_r = None
        self.prev_p = None

    def on_f9r(self, m: NavSatFix):
        if self.prev_r is None:
            self.prev_r = (m.latitude, m.longitude); return
        lat1, lon1 = self.prev_r; lat2, lon2 = m.latitude, m.longitude
        if abs(lat2-lat1) < 1e-12 and abs(lon2-lon1) < 1e-12: return
        msg = Float64()
        msg.data = bearing_deg(lat1, lon1, lat2, lon2)
        self.pub_f9r.publish(msg)
        self.prev_r = (lat2, lon2)

    def on_f9p(self, m: NavSatFix):
        if self.prev_p is None:
            self.prev_p = (m.latitude, m.longitude); return
        lat1, lon1 = self.prev_p; lat2, lon2 = m.latitude, m.longitude
        if abs(lat2-lat1) < 1e-12 and abs(lon2-lon1) < 1e-12: return
        msg = Float64()
        msg.data = bearing_deg(lat1, lon1, lat2, lon2)
        self.pub_f9p.publish(msg)
        self.prev_p = (lat2, lon2)

def main(args=None):
    rclpy.init(args=args)
    node = AzimuthPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
