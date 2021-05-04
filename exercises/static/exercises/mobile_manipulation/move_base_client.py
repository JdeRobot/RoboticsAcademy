#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import yaml

class Movebase_Client:
    def __init__(self):
        # rospy.init_node('movebase_client')
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.target_pose = {}
        filename = ('./navigation.yaml')
        with open(filename) as file:
            navigation_params = yaml.load(file)
            stop_pose = navigation_params["stop_pose"]
            target_names = stop_pose.keys()
            for target_name in target_names:
                pose = stop_pose[target_name]
                self.target_pose[target_name] = [pose["x"], pose["y"], pose["theta"]]

    def get_target_pose(self, target_name):
        return self.target_pose[target_name]

    def send_goal_to_client(self, pose):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        quat = quaternion_from_euler(0,0,pose[2])
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.client.send_goal(goal)

    def get_result_from_client(self):
        wait = self.client.wait_for_result()
        return wait
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     print("self.client.get_result()",self.client.get_result())
        #     return self.client.get_result()   

if __name__ == '__main__':
    # try:
    #    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Goal execution done!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")

    test = Movebase_Client()
    test.sendGoalToClient(4,0,0)