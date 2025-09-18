import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class AzimuthIndependent(Node):
    def __init__(self):
        super().__init__('azimuth_indep')
        # 독립적인 퍼블리셔 (topic 이름: /azimuth_indep)
        self.pub = self.create_publisher(Float64, '/azimuth_indep', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행

    def timer_callback(self):
        # 여기서는 그냥 현재 시간 기반으로 sin() 값 같은 걸 넣어 테스트
        angle = math.degrees(math.sin(time.time()))  # 더미 데이터
        msg = Float64()
        msg.data = angle
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing independent azimuth: {angle:.2f} deg')

def main(args=None):
    rclpy.init(args=args)
    node = AzimuthIndependent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
