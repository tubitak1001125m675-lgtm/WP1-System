import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    hover_thrust = float(LaunchConfiguration('hover_thrust').perform(context))
    wrench_topic = LaunchConfiguration('wrench_topic').perform(context)

    gazebo_model_path = os.path.expanduser('~/gazebo_models')
    current_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    if current_resource_path:
        resource_path = gazebo_model_path + ':' + current_resource_path
    else:
        resource_path = gazebo_model_path

    return [
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=resource_path
        ),

        ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                world
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/simple_quadrotor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/world/quad_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                f'{wrench_topic}@ros_gz_interfaces/msg/EntityWrench]ignition.msgs.EntityWrench',
                '/world/quad_world/wrench/clear@ros_gz_interfaces/msg/Entity]ignition.msgs.Entity',
            ],
            output='screen'
        ),

        Node(
            package='quad_wrench_control',
            executable='hover_test_node',
            parameters=[
                {
                    'hover_thrust': hover_thrust,
                    'wrench_topic': wrench_topic,
                }
            ],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.expanduser('~/gazebo_worlds/simple_quad_world.sdf'),
            description='Path to the Gazebo world file'
        ),

        DeclareLaunchArgument(
            'hover_thrust',
            default_value='0.0',
            description='Vertical force applied to the quadrotor in Newtons'
        ),

        DeclareLaunchArgument(
            'wrench_topic',
            default_value='/world/quad_world/wrench/persistent',
            description='Gazebo wrench topic'
        ),

        OpaqueFunction(function=launch_setup),
    ])