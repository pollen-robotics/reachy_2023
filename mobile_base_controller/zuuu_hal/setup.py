from setuptools import setup
from glob import glob

package_name = 'zuuu_hal'
examples_package_name = 'examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, examples_package_name],

    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name, glob("launch/*_launch.py")),
        ("share/" + package_name + "/config", glob("config/*.*")),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ZUUU Hardware Abstraction Layer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_me = examples.follow_me:main',
            'hal = zuuu_hal.zuuu_hal:main',
            'teleop_keyboard = examples.zuuu_teleop_keyboard:main',
            'teleop_joy = examples.zuuu_teleop_joy:main',
            'speed_calibration = examples.zuuu_speed_calibration:main',
            'set_speed_service_test = examples.set_speed_service_test:main',
            'go_to_service_test = examples.go_to_service_test:main',
        ],
    },
)
