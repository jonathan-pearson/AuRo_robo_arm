from glob import glob
from setuptools import find_packages, setup

package_name = 'auro_robo_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanat',
    maintainer_email='sanat@example.com',
    description='Small ROS 2 helpers for an Interbotix USB-controlled robot arm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nudge_joint = auro_robo_arm.nudge_joint:main',
            'pulse_gripper = auro_robo_arm.pulse_gripper:main',
        ],
    },
)
