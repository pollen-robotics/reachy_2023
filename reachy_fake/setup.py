from setuptools import setup

package_name = 'reachy_fake'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reachy',
    maintainer_email='steve.nguyen@pollen-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera = reachy_fake.fake_camera:main',
            'fake_zoom = reachy_fake.fake_zoom:main',
        ],
    },
)
