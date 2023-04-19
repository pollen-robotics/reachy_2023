import os
from glob import glob

from setuptools import setup

package_name = 'camera_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'examples'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'v4l2py>=0.6.1',
        'zoom_kurokesu>=1.1',
        ],
    zip_safe=True,
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 Humble package for Reachy 2023 camera controllers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_controllers.camera_publisher:main',
            'camera_zoom_service = camera_controllers.camera_zoom_service:main',
            'camera_focus = camera_controllers.camera_focus:main',
            'view_cam = examples.view_cam:main',
        ],
    },
)
