import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import EntityWrench, Entity
from tf2_msgs.msg import TFMessage

class AltitudeController(Node):

    def __init__(self):
        super().__init__('altitude_controller')

        # Parameters
        self.declare_parameter('mass', 1.5)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('z_ref', 1.0)

        self.declare_parameter('kp', 8.0)
        self.declare_parameter('kd', 4.0)

        self.declare_parameter('entity_name', 'simple_quadrotor::base_link')
        self.declare_parameter('pose_topic', '/model/simple_quadrotor/pose')
        self.declare_parameter('wrench_topic', '/world/quad_world/wrench')
        self.declare_parameter('control_period', 0.01)

        # Read parameters
        self.mass = float(self.get_parameter('mass').value)
        self.g = float(self.get_parameter('gravity').value)
        self.z_ref = float(self.get_parameter('z_ref').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.entity_name = self.get_parameter('entity_name').value

        pose_topic = self.get_parameter('pose_topic').value
        wrench_topic = self.get_parameter('wrench_topic').value
        control_period = float(self.get_parameter('control_period').value)
        # States
        self.z = None
        self.vz = 0.0
        self.last_time = None
        # Subscriber
        self.pose_sub = self.create_subscription(TFMessage, pose_topic, self.pose_callback, 10)
        # Publisher
        self.wrench_pub = self.create_publisher(EntityWrench, wrench_topic, 10 )
        # Timer
        self.timer = self.create_timer(control_period, self.control_loop)
        self.get_logger().info('Altitude controller started.')

    def pose_callback(self, msg):

        if len(msg.transforms) == 0:
            return

        transform = msg.transforms[0]
        now = self.get_clock().now()
        z_new = transform.transform.translation.z
        if self.z is not None and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 1e-6:
                self.vz = (z_new - self.z) / dt
        self.z = z_new
        self.last_time = now

    def control_loop(self):

        if self.z is None:
            return

        # Errors
        ez = self.z_ref - self.z
        ev = -self.vz
        # PD control
        u = self.kp * ez + self.kd * ev
        # Total thrust
        fz = self.mass * self.g + u
        if fz < 0.0:
            fz = 0.0

        # Message
        msg = EntityWrench()

        msg.entity.name = self.entity_name
        msg.entity.type = Entity.LINK

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = fz

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.wrench_pub.publish(msg)

        self.get_logger().info(f'z={self.z:.3f}, 'f'vz={self.vz:.3f}, 'f'Fz={fz:.3f}', throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()