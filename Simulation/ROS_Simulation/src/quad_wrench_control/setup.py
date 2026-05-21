import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'quad_wrench_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaser',
    maintainer_email='yaser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_reader_node = quad_wrench_control.imu_reader_node:main',
            'pose_reader_node = quad_wrench_control.pose_reader_node:main',
            'hover_test_node = quad_wrench_control.hover_test_node:main',
            'altitude_controller_node = quad_wrench_control.altitude_controller_node:main',
            'constant_force_node = quad_wrench_control.constant_force_node:main',
            'wrench_logger_node = quad_wrench_control.wrench_logger_node:main',
        ],
    },
)
