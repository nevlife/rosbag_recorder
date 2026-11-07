"""Publish the topics the launch test records, one of them 4 MiB at 10 Hz."""

import array

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

POINTS = 262144


class Publishers(Node):
    def __init__(self):
        super().__init__('test_publishers')
        self.chatter = self.create_publisher(String, '/test/chatter', 10)
        self.imu = self.create_publisher(Imu, '/test/imu', 10)
        self.mode = self.create_publisher(String, '/test/mode', 10)
        self.points = self.create_publisher(PointCloud2, '/test/points', 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static = self.create_publisher(TFMessage, '/tf_static', latched)
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.tf_static.publish(TFMessage(transforms=[t]))
        cloud = PointCloud2(height=1, width=POINTS, point_step=16, row_step=16 * POINTS)
        cloud.header.frame_id = 'lidar'
        cloud.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(['x', 'y', 'z', 'intensity'])
        ]
        cloud.data = array.array('B', bytes(16 * POINTS))
        self.cloud = cloud
        self.count = 0
        self.create_timer(0.05, self.on_chatter)
        self.create_timer(0.02, self.on_imu)
        self.create_timer(0.1, self.on_points)
        self.create_timer(2.0, self.on_mode)

    def on_chatter(self):
        self.count += 1
        self.chatter.publish(String(data=str(self.count)))

    def on_imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu'
        m.linear_acceleration.z = 9.8
        m.orientation.w = 1.0
        self.imu.publish(m)

    def on_points(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.points.publish(self.cloud)

    def on_mode(self):
        self.mode.publish(String(data='AUTO' if self.count // 40 % 2 else 'MANUAL'))


def main():
    rclpy.init()
    node = Publishers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
