from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='simple_uav_controller',
            executable='altitude_controller',
            name='altitude_controller',
            output='screen',

            parameters=[
                {
                    'mass': 1.5,
                    'gravity': 9.81,

                    'z_ref': 1.0,

                    'kp': 8.0,
                    'kd': 4.0,

                    'entity_name':
                        'simple_quadrotor::base_link',

                    'pose_topic':
                        '/model/simple_quadrotor/pose',

                    'wrench_topic':
                        '/world/quad_world/wrench',

                    'control_period': 0.01
                }
            ]
        )

    ])