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
import sys


def list_frw_contents(cell_path, cell):
    fixts = ['{0} ({1})'.format(o.name, o.kind.name) for o in cell.fixtures]
    obs   = ['{0} ({1})'.format(o.name, o.kind.name) for o in cell.obstacles]
    rcs   = [o.name for o in cell.robot_controllers]
    machs = [o.name for o in cell.machines]

    sys.stdout.write("""Cell metadata:
  file            : {file} ({path})
  name            : {name}
  description     : {desc}
  version         : {version}

""".format(
    file=os.path.basename(cell_path),
    path=os.path.dirname(os.path.abspath(cell_path)),
    name=cell.name,
    desc=cell.description,
    version=cell.version))

    sys.stdout.write("""Cell objects - summary:
  controllers ({cn}) : {clist}
  fixtures    ({fn}) : {flist}
  obstacles   ({on}) : {olist}
  machines    ({mn}) : {mlist}
  parts       ({pn}) : {plist}

""".format(
    cn=len(rcs), clist=', '.join(rcs),
    fn=len(fixts), flist=', '.join(fixts),
    on=len(obs), olist=', '.join(obs),
    mn=len(machs), mlist=', '.join(machs),
    pn=0, plist='[not implemented]'
    ))
