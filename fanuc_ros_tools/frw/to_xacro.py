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

import math
import os
import logging

from ..mini_urdf import *
from ..mini_xacro import *

from frfformats.frw import FrwShapeKind


class UnknownFanucFrwObjectKind(Exception):
    pass

_ZERO_ORIGIN=[0, 0, 0, 0, 0, 0]


def convert_frw_box(box):
    """
    :param box: The frw box geometry to convert to its urdf counter part, ``FrwCellObject``
    :returns: urdf element sequence for a box geometry, ``str``
    """
    return gen_urdf_box(size=[box.scale.x, box.scale.y, box.scale.z])

def convert_frw_sphere(sphere):
    """
    :param sphere: The frw sphere geometry to convert to its urdf counter part, ``FrwCellObject``
    :returns: urdf element sequence for a sphere geometry, ``str``
    """
    return gen_urdf_sphere(radius=sphere.scale.x)

def convert_frw_cylinder(cylinder):
    """
    :param cylinder: The frw cylinder geometry to convert to its urdf counter part, ``FrwCellObject``
    :returns: urdf element sequence for a cylinder geometry, ``str``
    """
    # scale.x is divided by two as Roboguide stores cylinder radius
    return gen_urdf_cylinder(length=cylinder.scale.y, radius=cylinder.scale.x/2.0)

def convert_frw_cad_obj(filename):
    """
    :param filename: Path to CAD file used in frw for a CAD geometry object, ``str``
    :returns: urdf element sequence for a mesh geometry, ``str``
    """
    return gen_urdf_mesh(filename=filename)

def convert_frw_triad(triad):
    """
    :param triad: Instance of a frw triad to convert into a ROS pose, ``FrwTriad``
    :returns: A ROS compatible pose (6 floats: xyz (meter), rpy (radians)) that is a direct conversion of the frw triad, ``seq(float)``
    """
    return [
        # convert position to meters first
        triad.x / 1000.0, triad.y / 1000.0, triad.z / 1000.0,
        # TODO: check order of rotation axes
        math.radians(triad.w), math.radians(triad.p), math.radians(triad.r)]


def convert_frw_cell_obj(obj, links, joints, pkg, name_prefix):
    origin = convert_frw_triad(obj.location)
    geom_origin = _ZERO_ORIGIN[:]
    # bare link name, xacro prefixing is done later
    link_name = 'link_{0}_{1}'.format(name_prefix, obj.name)

    if obj.kind == FrwShapeKind.BOX:
        geom = convert_frw_box(obj)
        # origin of box in Roboguide is on top plane. ROS has origin
        # at box centre. We want frame location to be identical to Roboguide,
        # as well as appearance. To achieve that, we offset the geometry by
        # half the geometry height.
        # TODO: do this in a nicer way
        geom_origin[2] -= (obj.scale.z / 2.0)

    elif obj.kind == FrwShapeKind.SPHERE:
        geom = convert_frw_sphere(obj)

    elif obj.kind == FrwShapeKind.CYLINDER:
        geom = convert_frw_cylinder(obj)
        # the geometry of cylinders in Roboguide is rotated PI/-2 over the
        # X-axis compared to ROS. To keep joint frame pose identical, we
        # compensate the geometry origin by rotating that by PI/-2.
        geom_origin[3] = math.pi/-2.0

    elif obj.kind == FrwShapeKind.CAD:
        mesh_file = os.path.basename(obj.cad_file.cached_file.replace('\\', '/'))
        mesh_file = '{0}.stl'.format(os.path.splitext(mesh_file)[0])
        geom = convert_frw_cad_obj(filename='package://{0}/{1}'.format(pkg, mesh_file))

    else:
        raise UnknownFanucFrwObjectKind("Cannot convert '{0}' as '{1}' is "
            "unsupported".format(obj.name, obj.kind.name))

    link = gen_link(
        name='${prefix}' + link_name,
        visual=gen_urdf_visual(
            geom=geom,
            material=gen_urdf_material(obj.colour.as_rgba_tuple()),
            origin=gen_urdf_origin(geom_origin))
    )

    joint = gen_joint_fixed(
        name='${prefix}base_link-' + link_name,
        parent='${prefix}base_link',
        child='${prefix}' + link_name,
        origin=gen_urdf_origin(origin)
    )

    links.append(link)
    joints.append(joint)


def convert_frw_cell_objs(objs, links, joints, pkg, prefix):
    for obj in objs:
        try:
            convert_frw_cell_obj(obj, links, joints, pkg, prefix)
        except Exception, e:
            logging.warning("Skipping '%s', error processing: %s" % (obj.name, e))


def convert_frw_uframes(ctrlrs, links, joints):
    for ctrlr in ctrlrs:
        for grp in ctrlr.robot_groups:

            # add frame for motion group. Note: this should coincide with
            # pose of robot itself
            grp_link_name = 'c{cidx}_g{gid}_{name}'.format(
                cidx=ctrlr.id,
                gid=grp.id,
                name=grp.name)
            grp_frame_origin = convert_frw_triad(grp.location)

            links.append(gen_link(name="${prefix}" + grp_link_name))
            joints.append(gen_joint_fixed(
                name='${prefix}base_link-' + grp_link_name,
                parent='${prefix}base_link',
                child='${prefix}' + grp_link_name,
                origin=gen_urdf_origin(grp_frame_origin)
            ))


            for uframe in grp.user_frames:
                # TODO: for now we use 'all zeros' to check for undefined user frames
                if uframe.location.all_zeros():
                    continue

                uframe_name = 'c{cidx}_g{gid}_u{uid}_{name}'.format(
                    cidx=ctrlr.id,
                    gid=grp.id,
                    uid=uframe.num,
                    name=uframe.name)
                uframe_origin = convert_frw_triad(uframe.location)
                # TODO: user frames are relative to robot world frame, which we
                # don't have access to at this time. Figure out some way to
                # fix this.

                # empty link
                links.append(gen_link(name="${prefix}" + uframe_name))
                joints.append(gen_joint_fixed(
                    name='${prefix}base_link-' + uframe_name,
                    parent='${prefix}' + grp_link_name,
                    child='${prefix}' + uframe_name,
                    origin=gen_urdf_origin(uframe_origin)
                ))


def convert_frw(frw_cell, macro_name, pkg, include_uframes=False):
    links = []
    joints = []

    # cell base link
    links.append(gen_link(name="${prefix}base_link"))

    # convert all supported cell objects
    convert_frw_cell_objs(frw_cell.obstacles, links, joints, pkg, 'obstacle')
    convert_frw_cell_objs(frw_cell.fixtures, links, joints, pkg, 'fixture')

    if include_uframes:
        convert_frw_uframes(frw_cell.robot_controllers, links, joints)

    # generate rest of file and return
    return gen_xacro(
        gen_xacro_robot(
            gen_xacro_macro(macro_name, joints=joints, links=links)))
