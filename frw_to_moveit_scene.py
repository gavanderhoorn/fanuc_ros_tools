#!/usr/bin/python
# Software License Agreement (Apache License)
#
# Copyright (c) 2014, TU Delft Robotics Institute
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
import xml.etree.ElementTree as ET

from transformations import quaternion_from_euler




class UnknownFanucFrwObjectKind(Exception):
    pass


_verbose_g = False
def log(msg):
    sys.stderr.write(msg + '\n')
def logv(msg):
    if _verbose_g:
        log(msg)


def gen_moveit_scene_box(elem):
    return "1\nbox\n{sx} {sy} {sz}".format(
        sx=float(elem.get('ScaleX', 1.0)),
        sy=float(elem.get('ScaleY', 1.0)),
        sz=float(elem.get('ScaleZ', 1.0))
    )


def gen_moveit_scene_cylinder(elem):
    return "1\ncylinder\n{radius} {length}".format(
        radius=float(elem.get('ScaleX', 0)) / 2.0,
        length=float(elem.get('ScaleY', 0))
    )


def gen_moveit_scene_sphere(elem):
    return "1\nsphere\n{radius}".format(
        radius=float(elem.get('ScaleX', 0))
    )


def vb_color_to_rgba(vb_color):
    clr = int(vb_color[2:], 16)
    # bgr
    b = ((clr >> (2 * 8)) & 0xFF)
    g = ((clr >> (1 * 8)) & 0xFF)
    r = ((clr >> (0 * 8)) & 0xFF)
    # rgba
    return (r / 255.0, g / 255.0, b / 255.0, 1.0)


def fanuc_wpr_to_quaternion(elem):
    w = float(elem.get('LocationW', 0))
    p = float(elem.get('LocationP', 0))
    r = float(elem.get('LocationR', 0))

    #sys.stderr.write("wpr: %.4f, %.4f, %.4f\n" % (w, p, r))

    return quaternion_from_euler(
        math.radians(r),  # R_oll
        math.radians(p),  # P_itch
        math.radians(w)   # ya_W
    )


FRW_FIXT_BOX      = 0
FRW_FIXT_SPHERE   = 2
FRW_FIXT_CYLINDER = 3

def gen_moveit_scene_shape(elem, offset):
    # dispatch based on object type
    fkind = int(elem.get('Kind'))

    if fkind == FRW_FIXT_BOX:
        inner = gen_moveit_scene_box(elem)
    elif fkind == FRW_FIXT_SPHERE:
        inner = gen_moveit_scene_sphere(elem)
    elif fkind == FRW_FIXT_CYLINDER:
        inner = gen_moveit_scene_cylinder(elem)
    else:
        raise UnknownFanucFrwObjectKind("Kind '%d' is an "
            "unknown fixture kind" % fkind)

    # calc quaternion from WPR
    quat = fanuc_wpr_to_quaternion(elem)

    # calc rgba colour
    colour = vb_color_to_rgba(elem.get('Color'))

    fmt = """* {object_name}
{shape_geometry}
{tx} {ty} {tz}
{qx} {qy} {qz} {qw}
{cr} {cg} {cb} {ca}
"""
    return fmt.format(
        object_name=elem.get('Name'),
        shape_geometry=inner,
        tx=(float(elem.get('LocationX', 0)) / 1000.0) + offset[0],
        ty=(float(elem.get('LocationY', 0)) / 1000.0) + offset[1],
        tz=(float(elem.get('LocationZ', 0)) / 1000.0) + offset[2],
        qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3],
        cr=colour[0], cg=colour[1], cb=colour[2], ca=colour[3]
    )


def gen_moveit_scene_object(elem, offset):
    elem_name = elem.get('Name')

    # see if this fixture is a geometric primitve (crude)
    try:
        if 'Kind' in elem.attrib:
            # yes, generate
            return gen_moveit_scene_shape(elem, offset)

        # no, something else, unsupported
        if _verbose_g:
            log("    Non-geometric fixtures unsupported, skipping")
        else:
            log("Fixture '%s' is not a geometric shape, skipping" % elem_name)

    except Exception, e:
        log("Skipping '%s', error processing: %s" % (elem_name, e.message))

    return ''


def gen_moveit_scene(elem_fixtures, scene_name="noname", offset=(0, 0, 0)):
    res = '%s\n' % scene_name

    logv('Exporting %d fixtures to MoveIt scene format' % len(elem_fixtures))

    for fixt in elem_fixtures.iter('Fixture'):
        logv("  Found '%s'" % fixt.get('Name'))
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

    # open the file
    with open(args.file_input, 'r') as f:
        # get frw contents
        root = ET.fromstring(f.read())

    frw_fixts = root.find('Fixtures')
    if frw_fixts is None:
        log("Couldn't find 'Fixtures' tag in file, aborting")
        return

    if len(frw_fixts) == 0:
        log("No fixtures defined in file, exiting")
        return

    logv('Found %d fixture tags' % len(frw_fixts))

    offsets = [float(x) for x in args.offset.split(' ')]
    if not all(offset == 0 for offset in offsets):
        logv('Using offsets (x, y, z): (%.3f; %.3f; %.3f)' % tuple(offsets))

    moveit_scene_str = gen_moveit_scene(frw_fixts, scene_name=args.scene_name,
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
