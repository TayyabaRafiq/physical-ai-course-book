from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vla_action_servers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VLA Team',
    maintainer_email='vla-team@example.com',
    description='ROS 2 Action Servers for VLA robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_object_server = vla_action_servers.pick_object_server:main',
            'place_object_server = vla_action_servers.place_object_server:main',
            'navigate_to_point_server = vla_action_servers.navigate_to_point_server:main',
            'inspect_object_server = vla_action_servers.inspect_object_server:main',
        ],
    },
)