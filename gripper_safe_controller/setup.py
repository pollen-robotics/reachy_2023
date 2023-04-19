from setuptools import setup

package_name = 'gripper_safe_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 Humble package managing Reachy grippers to avoid overheating.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_safe_controller = gripper_safe_controller.gripper_safe_controller:main',
        ],
    },
)
