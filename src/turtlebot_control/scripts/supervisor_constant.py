#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.cmd_mode_pub = rospy.Publisher('/turtlebot_controller/command_mode', String, queue_size=0)
        self.nav_pub = rospy.Publisher('turtlebot_controller/nav_goal', Float32MultiArray, queue_size=10)
        #self.trans_listener = tf.TransformListener()
        #self.trans_broad = tf.TransformBroadcaster()
        #self.has_tag_location = False
        #self.state = "INITIAL_STATE"
        #self.epsilon = 0.32
        #self.agent_x = 0.0
        #self.agent_y = 0.0
        #self.agent_theta = 0.0
        self.nav_cmd = Float32MultiArray()    
        self.nav_cmd.data = [-2.0, 3.0, 0.0]
        self.COMMAND_MODE = "POSITION_COMMAND"

    def loop(self):
        self.nav_pub.publish(self.nav_cmd)
        self.cmd_mode_pub.publish(self.COMMAND_MODE)
        
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
