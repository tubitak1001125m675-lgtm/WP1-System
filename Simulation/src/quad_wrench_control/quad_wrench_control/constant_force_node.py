import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import EntityWrench, Entity


class PersistentWrenchNode(Node):
    def __init__(self):
        super().__init__('persistent_wrench_node')

        self.declare_parameter('force_z', 14.715)
        self.force_z = float(self.get_parameter('force_z').value)

        self.entity_name = 'simple_quadrotor::base_link'

        self.wrench_pub = self.create_publisher(
            EntityWrench,
            '/world/quad_world/wrench/persistent',
            10
        )

        self.clear_pub = self.create_publisher(
            Entity,
            '/world/quad_world/wrench/clear',
            10
        )

        self.done = False
        self.phase = 'clear'
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if self.phase == 'clear':
            clear_msg = Entity()
            clear_msg.name = self.entity_name
            clear_msg.type = Entity.LINK
            self.clear_pub.publish(clear_msg)

            self.get_logger().info('Old persistent wrench cleared.')
            self.phase = 'publish'
            return

        if self.phase == 'publish':
            msg = EntityWrench()
            msg.entity.name = self.entity_name
            msg.entity.type = Entity.LINK

            msg.wrench.force.z = self.force_z

            self.wrench_pub.publish(msg)

            self.get_logger().info(
                f'Persistent wrench applied: Fz = {self.force_z:.4f} N'
            )

            self.phase = 'done'
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PersistentWrenchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()