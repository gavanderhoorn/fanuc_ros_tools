#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright (c) 2015, TU Delft Robotics Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# author: G.A. vd. Hoorn (TU Delft Robotics Institute)
#

from setuptools import setup, find_packages

setup(
    name='fanuc_ros_tools',
    version='0.0.5',
    packages=find_packages(),
    author='G.A. vd. Hoorn',
    author_email='g.a.vanderhoorn@tudelft.nl',
    maintainer='G.A. vd. Hoorn',
    maintainer_email='g.a.vanderhoorn@tudelft.nl',
    description=('Utilities for working with Fanuc robots in ROS-Industrial.'),
    long_description=('A set of Python (2) helper scripts for working with '
                       'Fanuc Robots (and related file formats) in ROS-Industrial.'),
    license='Apache License, Version 2.0',
    scripts=[
        'bin/frw2mscene',
        'bin/frw2xacro'
    ],
    url='https://github.com/gavanderhoorn/fanuc_ros_tools',
    install_requires=['transforms3d']
)
