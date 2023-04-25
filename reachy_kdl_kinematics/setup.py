from setuptools import setup

package_name = 'reachy_kdl_kinematics'

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
    description='ROS2 Humble package for Reachy 2023 kinematics (URDF, arms and head FK/IK). ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reachy_kdl_kinematics = reachy_kdl_kinematics.reachy_kdl_kinematics_node:main',
        ],
    },
)
