import os
from glob import glob
from setuptools import setup

package_name = 'reachy_sdk_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 Humble package handling the grpc server for robot joints.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reachy_sdk_server = reachy_sdk_server.reachy_sdk_server:main',
            'camera_server = reachy_sdk_server.camera_server:main',
        ],
    },
)
