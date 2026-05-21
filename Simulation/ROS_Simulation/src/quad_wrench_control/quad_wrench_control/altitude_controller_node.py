import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import EntityWrench, Entity


class AltitudeControllerNode(Node):
    def __init__(self):
        super().__init__('altitude_controller_node')

        # Vehicle parameters
        self.mass = 1.5
        self.gravity = 9.81

        # Altitude reference
        self.z_ref = 10.0

        # Simple PD gains
        self.kp_z = 1.0
        self.kd_z = 0.3

        # State variables
        self.z = None
        self.z_prev = None
        self.vz = 0.0
        self.t_prev = None

        self.entity_name = 'simple_quadrotor::base_link'

        # Subscribe to Gazebo pose information bridged to ROS
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/quad_world/dynamic_pose/info',
            self.pose_callback,
            10
        )

        # Publish wrench command to Gazebo
        self.wrench_pub = self.create_publisher(
            EntityWrench,
            '/world/quad_world/wrench',
            10
        )

        # Control loop at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Altitude controller node started.')
        self.get_logger().info(f'Target altitude: z_ref = {self.z_ref:.2f} m')

    def pose_callback(self, msg):
        for transform in msg.transforms:
            if 'simple_quadrotor' in transform.child_frame_id:
                z_measured = transform.transform.translation.z

                now = self.get_clock().now().nanoseconds * 1e-9

                if self.z is not None and self.t_prev is not None:
                    dt = now - self.t_prev
                    if dt > 1e-6:
                        self.vz = (z_measured - self.z) / dt

                self.z_prev = self.z
                self.z = z_measured
                self.t_prev = now

    def control_loop(self):
        if self.z is None:
            return

        # Altitude error
        ez = self.z_ref - self.z

        # Desired vertical force in world frame
        fz = self.mass * self.gravity + self.kp_z * ez - self.kd_z * self.vz

        # Avoid negative thrust for this simple test
        if fz < 0.0:
            fz = 0.0

        msg = EntityWrench()

        msg.entity.name = self.entity_name
        msg.entity.type = Entity.LINK

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = float(fz)

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.wrench_pub.publish(msg)

        self.get_logger().info(
            f'z = {self.z:.3f}, vz = {self.vz:.3f}, Fz = {fz:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()