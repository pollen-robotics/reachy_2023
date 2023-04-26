#!/usr/bin/env python
"""Setup config file."""
from os import path
from setuptools import find_packages, setup

package_name = "reachy_utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        "PyYAML",
        "pypot",
        "reachy-sdk",
    ],
    extras_require={
        "doc": ["sphinx"],
    },
    entry_points={
        "console_scripts": [
            "orbita-zero-hardware = reachy_utils.orbita_zero_hardware:main",
            "reachy-discovery = reachy_utils.discovery:scan",
            "reachy-udev-rule-right-arm = reachy_utils.udev_rules:write_udev_rules_right_arm",
            "reachy-udev-rule-left-arm = reachy_utils.udev_rules:write_udev_rules_left_arm",
            "reachy-udev-rule-head = reachy_utils.udev_rules:write_udev_rules_head",
            "reachy-identify-zuuu-model = reachy_utils.config:get_zuuu_version",
        ],
    },
    maintainer='Pollen Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='Tools used for Reachy 2023.',
    license='Apache-2.0',
)
