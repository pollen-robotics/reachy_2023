import os
from glob import glob

from setuptools import setup

package_name = 'camera_controllers'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='simsim',
    maintainer_email='32677536+simheo@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_controllers.camera_publisher:main',
            'view_cam = examples.view_cam:main',
        ],
    },
)
