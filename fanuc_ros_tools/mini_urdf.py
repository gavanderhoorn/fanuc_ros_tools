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

def gen_urdf_box(size):
    """
    :param size: Three element sequence containing x, y and z dimensions (meters) of box, ``seq(float)``
    :returns: urdf element sequence for a box geometry, ``str``
    """
    return '<geometry><box size="{0} {1} {2}" /></geometry>'.format(*size)


def gen_urdf_sphere(radius):
    """
    :param radius: Radius of the urdf sphere (meters), ``float``
    :returns: urdf element sequence for a sphere geometry, ``str``
    """
    return '<geometry><sphere radius="{0}" /></geometry>'.format(radius)


def gen_urdf_cylinder(length, radius):
    """
    :param length: Length of the urdf cylinder (meters), ``float``
    :param radius: Radius of the urdf cylinder (meters), ``float``
    :returns: urdf element sequence for a cylinder geometry, ``str``
    """
    return '<geometry><cylinder length="{0}" radius="{1}" /></geometry>'.format(length, radius)


def gen_urdf_mesh(filename):
    """
    :param filename: Absolute or relative path to a mesh resource file, suitable for inclusion in a urdf, ``str``
    :returns: urdf element sequence for a mesh geometry, ``str``
    """
    return '<geometry><mesh filename="{0}" /></geometry>'.format(filename)


def gen_urdf_origin(pose):
    """
    :param pose: Six element sequence representing position (meters) and orientation (radians) of a box, ``seq(float)``
    :returns: urdf element sequence for an origin, ``str``
    """
    return '<origin xyz="{0} {1} {2}" rpy="{3} {4} {5}" />'.format(*pose)


def gen_urdf_material(color_rgba):
    """
    :param color_rgba: Four element sequence (0 to 1) encoding an rgba colour tuple, ``seq(float)``
    :returns: urdf element sequence for an anonymous material definition containing just a color element, ``str``
    """
    return '<material name=""><color rgba="{0} {1} {2} {3}"/></material>'.format(*color_rgba)


def gen_urdf_visual(geom, material, origin):
    """
    Generates (as a string) the complete urdf element sequence for a `visual` child
    of a `link` element. This is essentially a string concatenation operation.

    :param geom: urdf element sequence for the geometry child of a visual element, ``str``
    :param material: urdf element sequence for the material child of a visual element, ``str``
    :param origin: urdf element sequence for the origin child of a visual element, ``str``
    :returns: urdf element sequence for the `visual` child of a `link` element, ``str``
    """
    return '<visual>{0}{1}{2}</visual>'.format(geom, material, origin)


def gen_urdf_collision(geom, material, origin):
    """
    Generates (as a string) the complete urdf element sequence for a `collision` child
    of a `link` element. This is essentially a string concatenation operation.

    :param geom: urdf element sequence for the geometry child of a collision element, ``str``
    :param material: urdf element sequence for the material child of a collision element, ``str``
    :param origin: urdf element sequence for the origin child of a collision element, ``str``
    :returns: urdf element sequence for the `collision` child of a `link` element, ``str``
    """
    return '<collision>{0}{1}{2}</collision>'.format(geom, material, origin)


def gen_link(name, visual=None, collision=None):
    """
    Generates (as a string) the complete urdf element sequence for a `link`
    element. This is essentially a string concatenation operation.

    If `collision` is not provided, a duplicate of `visual` will be used
    instead.

    :param name: Name of the link, ``str``
    :param visual: urdf element sequence for the visual child, ``str``
    :param collision: urdf element sequence for the collision child, ``str``
    :returns: urdf element sequence for a `link`, ``str``
    """
    if visual is None:
        visual = ''
    if collision is None:
        collision=visual.replace('visual', 'collision')
    return '<link name="{0}">{1}{2}</link>'.format(name, visual, collision)


def gen_joint_fixed(name, parent, child, origin):
    """
    Generates (as a string) the complete urdf element sequence for a `joint`
    element with `type` set to `fixed`.

    :param name: Name of the joint, ``str``
    :param parent: Name of parent link, ``str``
    :param child: Name of child link, ``str``
    :param origin: Transform from `parent` to `child`, ``str``
    :returns: urdf element sequence for a fixed `joint`, ``str``
    """
    return '<joint name="{0}" type="fixed"><parent link="{1}" /><child link="{2}" />{3}</joint>'.format(
        name, parent, child, origin)
