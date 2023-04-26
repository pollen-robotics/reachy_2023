from setuptools import setup

package_name = 'reachy_fake'

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
    description='Package to generate a fake Reachy, used by simulation tools like Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera = reachy_fake.fake_camera:main',
            'fake_zoom = reachy_fake.fake_zoom:main',
        ],
    },
)
