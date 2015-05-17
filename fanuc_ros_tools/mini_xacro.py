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

import os

def gen_xacro_macro(name, links, joints):
    """
    Generates (as a string) the complete urdf element sequence for a simple
    ROS-Industrial xacro macro that defines geometry (links, joints). It takes
    a single argument ``prefix`` that should be used when instantiating the
    macro in a composite parent scene.

    Note that the ``links`` and ``joints`` sequences should already be strings.
    The ``gen_link(..)`` and ``gen_joint_fixed(..)`` macros may be used for that.

    :param name: Name of the macro, ``str``
    :param links: Sequence containing all the links that should be defined by macro, ``seq(str)``
    :param joints: Sequence containing all the joints that should be defined by the macro, ``seq(str)``
    :returns: urdf element sequence for a xacro macro, ``str``
    """
    links_str  = ''.join(links)
    joints_str = ''.join(joints)
    return '<xacro:macro name="{name}" params="prefix">{links}{joints}</xacro:macro>'.format(
        name=name, links=links_str, joints=joints_str)


def gen_xacro_robot(macro):
    """
    Generates (as a string) the complete urdf element sequence for a xacro
    robot definition. This is essentially a string concatenation operation.

    Note that the ``macro`` sequence should already be a string.

    :param macro: The xacro macro to embed, ``str``
    :returns: urdf element sequence for a xacro robot, ``str``
    """
    return '<robot xmlns:xacro="http://ros.org/wiki/xacro">{}</robot>'.format(macro)


def gen_xacro(robot):
    """
    Top level method that adds the xml pre-amble to the given xacro robot
    definition. This is essentially a string concatenation operation.

    Note that the ``robot`` sequence should already be a string.

    :param robot: The xacro robot to embed, ``str``
    :returns: String representation of a complete xacro file (including xml pre-amble), ``str``
    """
    return '<?xml version="1.0" ?>' + robot + os.linesep
