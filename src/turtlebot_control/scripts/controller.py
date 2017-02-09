#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.Subscriber('/turtlebot_control/position_goal', Float32MultiArray, self.pos_callback)
        rospy.Subscriber('/turtlebot_control/velocity_goal', Float32MultiArray, self.vel_callback)
        rospy.Subscriber('/turtlebot_control/command_mode', String, self.mode_callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_g = 1.0
        self.y_g = 1.0
        self.th_g = 1.0
        self.cmd_mode = "VELOCITY_MODE"
        self.V_cmd = 0.0
        self.om_cmd = 0.0

    def mode_callback(self, data):
        self.cmd_mode = data.data

    def pos_callback(self, data):
        self.x_g = data.data[0]
        self.y_g = data.data[1]
        self.th_g = data.data[2]
        # etc

    def vel_callback(self, data):
        self.V_cmd = data.data[0]
        self.om_cmd = data.data[1]

    def callback(self, data):
        pose = data.pose[data.name.index("mobile_base")]
        twist = data.twist[data.name.index("mobile_base")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        cmd_x_dot = 0.0 # forward velocity
        cmd_theta_dot = 0.0

        # goal values
        #x_g = 1.0
        #y_g = 1.0
        #th_g = 0.0
        
        def wrapToPi(a):
            if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
                return [(b + np.pi) % (2*np.pi) - np.pi for b in a]
            return (a + np.pi) % (2*np.pi) - np.pi

        Dx = self.x_g - self.x
        Dy = self.y_g - self.y
        rho = np.sqrt(Dx**2 + Dy**2)
        alpha = wrapToPi(np.arctan2(Dy,Dx) - self.theta)
        delta = wrapToPi(self.theta + alpha - self.th_g)

        #Define control inputs (V,om) - without saturation constraints
        k1 = 0.8 / np.pi
        k2 = 1.25 / np.pi
        k3 = 1.0
        
        V = k1 * rho * np.cos(alpha)
        om = k2 * alpha + k1 * (np.sin(alpha)*np.cos(alpha) / alpha) * ( alpha + k3 * delta)
        #om = 1.0
        # Apply saturation limits
        if self.cmd_mode == "VELOCITY_MODE":
            V = self.V_cmd
            om = self.om_cmd
        
        cmd_x_dot = np.sign(V)*min(0.5, np.abs(V))
        cmd_theta_dot = np.sign(om)*min(1, np.abs(om))

        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
