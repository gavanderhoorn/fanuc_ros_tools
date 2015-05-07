#!/usr/bin/python
# Software License Agreement (Apache License)
#
# Copyright (c) 2014, 2015, TU Delft Robotics Institute
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
#
# Converts information about fixtures in a Roboguide Work Cell into
# a MoveIt Scene description file. Only supports geometric primitive
# fixtures (box, cylinder, sphere) for now.
#
# References:
#  - http://moveit.ros.org/wiki/Scene_Format
#
#
# author: G.A. vd. Hoorn (TU Delft Robotics Institute)
#
import sys
import math

from transforms3d.euler import euler2quat

from frfformats import frw_reader
from frfformats.frw import FrwShapeKind





class UnknownFanucFrwObjectKind(Exception):
    pass


_verbose_g = False
def log(msg):
    sys.stderr.write(msg + '\n')
def logv(msg):
    if _verbose_g:
        log(msg)


def gen_moveit_scene_box(fixt):
    return "1\nbox\n{sx} {sy} {sz}".format(
        sx=fixt.scale.x,
        sy=fixt.scale.y,
        sz=fixt.scale.z)


def gen_moveit_scene_cylinder(fixt):
    return "1\ncylinder\n{radius} {length}".format(
        radius=fixt.scale.x / 2.0,
        length=fixt.scale.y)


def gen_moveit_scene_sphere(fixt):
    return "1\nsphere\n{radius}".format(radius=fixt.scale.x)


def fanuc_wpr_to_quaternion(fixt):
    return euler2quat(
        math.radians(fixt.location.w),  # R_oll
        math.radians(fixt.location.p),  # P_itch
        math.radians(fixt.location.r)   # ya_W
    )



def gen_moveit_scene_shape(fixt, offset):
    # dispatch based on object type
    fkind = fixt.kind

    if fkind == FrwShapeKind.BOX:
        inner = gen_moveit_scene_box(fixt)
    elif fkind == FrwShapeKind.SPHERE:
        inner = gen_moveit_scene_sphere(fixt)
    elif fkind == FrwShapeKind.CYLINDER:
        inner = gen_moveit_scene_cylinder(fixt)
    else:
        raise UnknownFanucFrwObjectKind("'%s' is an "
            "unsupported fixture kind for '%s'" % (fkind, fixt.name))

    # calc quaternion from WPR
    quat = fanuc_wpr_to_quaternion(fixt)

    # calc rgba colour
    colour = fixt.colour.as_rgba_tuple()

    # Roboguide places origin of boxes on the top, whereas ROS / MoveIt
    # has them in the centre of the object. Compensate.
    tz = fixt.location.z
    if fixt.kind == FrwShapeKind.BOX:
        # scale needs to be multiplied by 1000, since it is in 'meters'
        tz = fixt.location.z - ((fixt.scale.z * 1000.0) / 2.0)

    fmt = """* {object_name}
{shape_geometry}
{tx} {ty} {tz}
{qx} {qy} {qz} {qw}
{cr} {cg} {cb} {ca}
"""
    return fmt.format(
        object_name=fixt.name,
        shape_geometry=inner,
        tx=(fixt.location.x / 1000.0) + offset[0],
        ty=(fixt.location.y / 1000.0) + offset[1],
        tz=(tz / 1000.0) + offset[2],
        qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3],
        cr=colour[0], cg=colour[1], cb=colour[2], ca=colour[3]
    )


def gen_moveit_scene_object(fixt, offset):
    elem_name = fixt.name

    # see if this fixture is a geometric primitve (crude)
    try:
        return gen_moveit_scene_shape(fixt, offset)
    except Exception, e:
        log("Skipping '%s', error processing: %s" % (elem_name, e.message))
    return ''


def gen_moveit_scene(fixts, scene_name="noname", offset=(0, 0, 0)):
    res = '%s\n' % scene_name

    logv('Exporting %d fixtures to MoveIt scene format' % len(fixts))

    for fixt in fixts:
        logv("  Found '%s'" % fixt.name)
        res += gen_moveit_scene_object(fixt, offset)

    res += '.\n'
    return res


def main():
    import argparse

    # parse command line opts
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true',
            dest='verbose', help='Be verbose')
    parser.add_argument('--scene-name', type=str, metavar='NAME',
            default='noname', dest='scene_name', help='Name of MoveIt scene '
            '(default: %(default)s)')
    parser.add_argument('--offset', type=str, metavar='OFFSET',
            default="0 0 0", dest='offset', help="Translate objects in FRW "
            "by OFFSET before exporting to MoveIt scene. Format: 'x y z' "
            "(m, float, default: %(default)s)")
    parser.add_argument('file_input', type=str, metavar='INPUT',
            help="Roboguide Work Cell file (*.frw)")
    parser.add_argument('file_output', type=str, metavar='OUTPUT',
            nargs='?', help="Output file (if not specified, "
            "stdout will be used)")

    # handle all arguments
    args = parser.parse_args()
    global _verbose_g
    _verbose_g = args.verbose

    # check
    # TODO: http://stackoverflow.com/a/9979169
    if len(args.offset.split(' ')) != 3 or ',' in args.offset:
        log("Invalid offset specified. Format: 'x y z' (no commas).\n")
        return

    # load the cell description from the file
    cell = frw_reader.load_file(args.file_input)

    if len(cell.fixtures) == 0:
        log("No fixtures defined in file, exiting")
        return
    logv('Found %d fixtures in cell' % len(cell.fixtures))

    offsets = [float(x) for x in args.offset.split(' ')]
    if not all(offset == 0 for offset in offsets):
        logv('Using offsets (x, y, z): (%.3f; %.3f; %.3f)' % tuple(offsets))

    moveit_scene_str = gen_moveit_scene(cell.fixtures, scene_name=args.scene_name,
        offset=offsets)

    logv('Saving result')

    if args.file_output:
        with open(args.file_output, 'w') as f:
            f.write(moveit_scene_str)
    else:
        # only print lines if not redirected
        if sys.stdout.isatty():
            log('\n-----------------------------cut-here---')
        sys.stdout.write(moveit_scene_str)
        if sys.stdout.isatty():
            log('-----------------------------cut-here---')

    logv('Done')
    log('')


if __name__ == '__main__':
    main()
