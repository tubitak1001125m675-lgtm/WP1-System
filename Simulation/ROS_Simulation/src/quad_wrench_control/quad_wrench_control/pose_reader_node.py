import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class PoseReaderNode(Node):
    def __init__(self):
        super().__init__('pose_reader_node')

        self.subscription = self.create_subscription(
            TFMessage,
            '/world/quad_world/dynamic_pose/info',
            self.pose_callback,
            10
        )

        self.get_logger().info('Pose reader node started.')

    def pose_callback(self, msg):
        for transform in msg.transforms:
            # Print only the quadrotor transform
            if 'simple_quadrotor' in transform.child_frame_id:
                p = transform.transform.translation
                q = transform.transform.rotation

                self.get_logger().info(
                    f'pos = [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}], '
                    f'quat = [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]'
                )


def main(args=None):
    rclpy.init(args=args)
    node = PoseReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()