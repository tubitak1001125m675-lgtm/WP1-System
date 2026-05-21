import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
import csv
import time


class WrenchLoggerNode(Node):
    def __init__(self):
        super().__init__('wrench_logger_node')

        self.declare_parameter('wrench_topic', '/world/quad_world/wrench')
        self.declare_parameter('output_file', 'wrench_log.csv')
        self.declare_parameter('sample_frequency', 100.0)

        self.wrench_topic = self.get_parameter('wrench_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.sample_frequency = float(self.get_parameter('sample_frequency').value)

        self.sample_period = 1.0 / self.sample_frequency
        self.last_sample_time = 0.0

        self.data = []

        self.subscriber = self.create_subscription(
            EntityWrench,
            self.wrench_topic,
            self.wrench_callback,
            10
        )

        self.get_logger().info(f'Listening to: {self.wrench_topic}')
        self.get_logger().info(f'Sample frequency: {self.sample_frequency} Hz')
        self.get_logger().info(f'Output file: {self.output_file}')

    def wrench_callback(self, msg):
        now = time.time()

        if now - self.last_sample_time < self.sample_period:
            return

        self.last_sample_time = now

        row = [
            now,
            msg.entity.name,
            msg.entity.type,
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ]

        self.data.append(row)

    def save_to_file(self):
        self.get_logger().info(f'Saving {len(self.data)} samples to {self.output_file}')

        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)

            writer.writerow([
                'time',
                'entity_name',
                'entity_type',
                'force_x',
                'force_y',
                'force_z',
                'torque_x',
                'torque_y',
                'torque_z'
            ])

            writer.writerows(self.data)

        self.get_logger().info('File saved successfully.')


def main(args=None):
    rclpy.init(args=args)

    node = WrenchLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received.')
    finally:
        node.save_to_file()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()