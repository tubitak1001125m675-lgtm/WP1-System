import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import EntityWrench, Entity


class HoverTestNode(Node):
    def __init__(self):
        super().__init__('hover_test_node')

        self.declare_parameter('hover_thrust', 14.715)
        self.declare_parameter('entity_name', 'simple_quadrotor::base_link')
        self.declare_parameter('mode', 'persistent')  # persistent or normal
        self.declare_parameter('publish_period', 0.001)

        self.hover_thrust = float(self.get_parameter('hover_thrust').value)
        self.entity_name = self.get_parameter('entity_name').value
        self.mode = self.get_parameter('mode').value
        self.publish_period = float(self.get_parameter('publish_period').value)

        # Select the wrench topic and timer depending on the selected mode
        if self.mode == 'persistent':
            self.wrench_topic = '/world/quad_world/wrench/persistent'
        else:
            self.wrench_topic = '/world/quad_world/wrench'

        # Define a publisher for sending clear commands to Gazebo
        self.clear_publisher = self.create_publisher(Entity,'/world/quad_world/wrench/clear',1)

        # Define a publisher for sending wrench commands to Gazebo
        self.publisher = self.create_publisher(EntityWrench, self.wrench_topic, 1)

        self.phase = 'clear'
        # First, clear old persistent wrenches. Then publish new wrench.
        self.timer = self.create_timer(0.5, self.timer_callback)
        
#        self.done = False

        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Wrench topic: {self.wrench_topic}')
        self.get_logger().info(f'Hover thrust: {self.hover_thrust:.4f} N')

    def create_clear_msg(self):
        msg = Entity()
        msg.name = self.entity_name
        msg.type = Entity.LINK
        return msg
    
    def create_wrench_msg(self):
        msg = EntityWrench()

        msg.entity.name = self.entity_name
        msg.entity.type = Entity.LINK

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = self.hover_thrust

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        return msg

    def publish_normal_wrench(self):
        self.publisher.publish(self.create_wrench_msg())

    def timer_callback(self):
        if self.phase == 'clear':
            # Send a command to clear any existing persistent wrench on the link
            self.clear_publisher.publish(self.create_clear_msg())
            self.get_logger().info('Clear command published.')

            self.phase = 'publish'
            return

        if self.phase == 'publish':
            if self.mode == 'persistent':
                # Persistent wrench must be published only once
                self.publisher.publish(self.create_wrench_msg())
                self.get_logger().info(f'Persistent wrench published: Fz = {self.hover_thrust:.4f} N')       
                self.phase = 'done'
                self.timer.cancel()
            else:
                # Switch to normal wrench loop
                self.timer.cancel()
                self.timer = self.create_timer(self.publish_period, self.publish_normal_wrench)
                self.get_logger().info(f'Normal wrench published: Fz = {self.hover_thrust:.4f} N')
            return


def main(args=None):
    rclpy.init(args=args)
    node = HoverTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()