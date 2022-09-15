import rospy
import random
from geometry_msgs.msg import Twist
import gazebo_msgs.msg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class Turtlebot():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.sleep(0.1)
        self.msg = Twist()
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self.turtlebot_pose)
        self.play = 0
        self.tb_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tb_state_update_callback)
        

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    def turtlebot_pose(self,data):
        if self.play == 1:
            self.x_v = random.uniform(0.0,0.2)
            self.y_v= random.uniform(0.0,0.2) if self.x_v<=0.15 else 0.05
    
    def tb_state_update_callback(self, event= None):
        if self.play == 1:
            self.x_v = random.uniform(0.0,0.2)
            self.y_v= random.uniform(0.0,0.2) if self.x_v<=0.15 else 0.05
            self.msg.linear.x = self.x_v
            self.msg.linear.y = self.y_v
        elif self.play == 0:
            self.msg.linear.x = 0.0
            self.msg.linear.y = 0.0            
        else:
            pass
        self.tb_vel_pub.publish(self.msg)
            
    def start_turtlebot(self):
        self.play = 1
    

    def stop_turtlebot(self):
        self.play = 0


    def reset_turtlebot(self):
        self.play = 2
        req = ModelState()
        req.model_name = "turtlebot3"
        req.twist.linear.x = 0.0
        req.twist.linear.y = 0.0
        req.twist.linear.z = 0.0
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = 0.0
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 0.0
        self.set_state(req)

