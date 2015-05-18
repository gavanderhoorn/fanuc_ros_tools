# fanuc_ros_tools


## Overview

A small collection of tools that can be useful when using a Fanuc robot with ROS-Industrial.


## Tools

### frw2mscene

![Roboguide and MoveIt scene](https://raw.github.com/gavanderhoorn/fanuc_ros_tools/screenshots/imgs/frw2mscene_example.png)

This tool can convert Fixtures defined in a Fanuc Roboguide Workcell XML file (`.frw`) into objects in a MoveIt scene (`.scene`). This is convenient when you use Roboguide for your kinematics simulation, and don't want to have to recreate the workcell in the MoveIt motion planning plugin.

Only geometric Fixtures are supported for now (ie: boxes, spheres and cylinders).

#### Example invocation

```shell
frw2mscene -v \
  --scene-name my_workcell \
  /path/to/fanuc/workcell/file.frw \
  /path/to/output/my_workcell.scene
```

The resulting `.scene` file can be imported using the *Import From Text* button on the *Scene Objects* tab of the MoveIt plugin in RViz.

#### Status / know limitations

 - no support for mesh resources (no CAD files)
 - no support for Obstacles, Machines, Parts, Dress-outs, or anything else but Fixtures
 - no support for unicode object names (diacritics, accents, ligatures, etc)

### frw2xacro

![Roboguide and RViz](https://raw.github.com/gavanderhoorn/fanuc_ros_tools/screenshots/imgs/frw2xacro_example.png)

This tool attempts to convert a Fanuc Roboguide Workcell XML file (`.frw`) into a corresponding ROS compatible xacro macro. Currently, only Roboguide Obstacles and Fixtures are supported. Contrary to `frw2mscene`, this tool also supports work cell objects that consist of CAD files.

Note: actual conversion of meshes should still be done manually, as the tool will only include the required urdf element sequences referencing the mesh.

Note also that the generated xacro generates a macro definition _only_. The macro will have to be called / instantiated in another file first, before it can be loaded into a `robot_description` parameter or similar.

#### Example invocation

```shell
frw2xacro -v \
  --macro-name my_workcell \
  --package my_workcell_pkg \
  /path/to/fanuc/workcell/file.frw \
  /path/to/output/my_workcell_macro.xacro
```

#### Status / know limitations

 - no support for multi-file CAD objects
 - no automatic conversion of (csb) mesh resources
 - no support for Machines, Parts, Dress-outs, or anything else but Fixtures and Obstacles
 - no support for unicode object names (diacritics, accents, ligatures, etc)


## Installation

The scripts in this repository depend on the following packages:

 - [transforms3d][]
 - [frfformats][]

Installation with `pip` will take care of all dependencies.

Note that `transforms3d` may also be available for installation through the package manager of your OS, in which case you might want to install that first. If not, it will be installed automatically from pypi by `pip`.

In a terminal window:

```shell
(sudo -H) pip install -r https://raw.githubusercontent.com/gavanderhoorn/fanuc_ros_tools/master/requirements.txt
(sudo -H) pip install git+https://github.com/gavanderhoorn/fanuc_ros_tools.git
```

Specific released versions may be installed with:

```shell
cd /some/tmp/dir
wget https://github.com/gavanderhoorn/fanuc_ros_tools/archive/0.0.5.zip
unzip 0.0.5.zip

(sudo -H) pip install -r fanuc_ros_tools-0.0.5/requirements.txt
(sudo -H) pip install fanuc_ros_tools-0.0.5
```

In all cases the utility scripts will be placed on the path, so should be usable from anywhere.




[transforms3d]: https://github.com/matthew-brett/transforms3d
[frfformats]: https://github.com/gavanderhoorn/frfformats
