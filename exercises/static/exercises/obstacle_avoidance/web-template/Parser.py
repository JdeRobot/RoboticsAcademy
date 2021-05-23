#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Eduardo Perdices <eperdices@gsyc.es>
#

from interfaces.pose3d import Pose3d
import json
from Target import Target

class Parser:
    def __init__(self,filename):
        with open(filename) as data_file:
            self.data = json.load(data_file)

    def getTargets(self):
        targets = []
        for t in self.data["targets"]:
            pose = Pose3d()
            pose.x = t["x"]; pose.y = t["y"]
            targets.append(Target(t["name"],pose,False,False))
        return targets




