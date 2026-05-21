import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuReaderNode(Node):
    def __init__(self):
        super().__init__('imu_reader_node')

        self.subscription = self.create_subscription(
            Imu,
            '/simple_quadrotor/imu',
            self.imu_callback,
            10
        )

        self.get_logger().info('IMU reader node started.')

    def imu_callback(self, msg):
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.get_logger().info(
            f'gyro = [{wx:.3f}, {wy:.3f}, {wz:.3f}], '
            f'acc = [{ax:.3f}, {ay:.3f}, {az:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()