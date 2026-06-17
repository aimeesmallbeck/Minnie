#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

from setuptools import setup
import os
from glob import glob

package_name = 'aimee_behaviors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='Arduino SRL',
    maintainer_email='software@arduino.cc',
    description='Idle and talking animation behaviors for the AIMEE Robot base',
    license='MPL-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'animation_node = aimee_behaviors.animation_node:main',
            'marker_localization_node = aimee_behaviors.marker_localization_node:main',
        ],
    },
)
