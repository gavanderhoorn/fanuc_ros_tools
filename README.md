# fanuc_ros_tools

A small collection of tools that can be useful when using a Fanuc robot with ROS.


## Overview

### frw_to_moveit_scene

![Roboguide and MoveIt scene](https://raw.github.com/gavanderhoorn/fanuc_ros_tools/screenshots/imgs/frw2mscene_example.png)

This tool can convert fixtures defined in a Fanuc Roboguide Workcell XML file (`.frw`) into objects in a MoveIt scene (`.scene`). This is convenient when you use Roboguide for your kinematics simulation, and don't want to have to recreate the workcell in the MoveIt motion planning plugin.

Only geometric fixtures are supported for now (ie: boxes, spheres and cylinders). The resulting `.scene` file can be imported using the *Import From Text* button on the *Scene Objects* tab.


## Installation

The scripts in this repository depend on the following packages:

 - [transforms3d][]
 - [frfformats][]

Both of these can be installed using `pip` ([transforms3d][] may also be available for installation through the package manager of your OS):

```
(sudo -H) pip install git+https://github.com/gavanderhoorn/frfformats.git
(sudo -H) pip install transforms3d
```

Now just clone the `fanuc_ros_tools` repository and run the scripts directly. No further steps are required.



[transforms3d]: https://github.com/matthew-brett/transforms3d
[frfformats]: https://github.com/gavanderhoorn/frfformats
