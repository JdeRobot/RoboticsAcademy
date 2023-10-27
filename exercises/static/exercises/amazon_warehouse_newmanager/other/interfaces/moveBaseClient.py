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
#       Shyngyskhan Abilkassov <s.abilkassov@gmail.com>
#

import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy
from std_srvs.srv import Empty

## Use this if after moving pallet, there are obstacles left on costmaps, which prevent proper movement after moving and dropping pallet
def clearCostmaps():
    rospy.wait_for_service('/amazon_warehouse_robot/move_base/clear_costmaps')
    clear_costmaps = rospy.ServiceProxy('/amazon_warehouse_robot/move_base/clear_costmaps', Empty)

    try:
        print("Clearing costmap")
        clear_costmaps()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

class MoveBaseClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/amazon_warehouse_robot/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.data = [0, 0]
        self.goal = None

    def sendGoalToClient(self, posX, posY, yaw = 0):

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = posX
        self.goal.target_pose.pose.position.y = posY

        orientation_q = quaternion_from_euler(0, 0, yaw)

        self.goal.target_pose.pose.orientation.x = orientation_q[0]
        self.goal.target_pose.pose.orientation.y = orientation_q[1]
        self.goal.target_pose.pose.orientation.z = orientation_q[2]
        self.goal.target_pose.pose.orientation.w = orientation_q[3]

        self.client.send_goal(self.goal)

        self.isFinished = self.client.wait_for_result(rospy.Duration(0.5))
        
    def getResultFromClient(self):
        if (self.goal):
            return self.client.get_result()
        else:
            return None