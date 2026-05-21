import os
from glob import glob
from setuptools import setup

package_name = 'simple_uav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaser',
    maintainer_email='yaser@example.com',
    description='Simple UAV altitude controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altitude_controller = simple_uav_controller.altitude_controller:main',
        ],
    },
    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),

    (
        'share/' + package_name,
        ['package.xml'],
    ),

    (
        os.path.join(
            'share',
            package_name,
            'launch'
        ),
        glob('launch/*.launch.py'),
    ),
],
)