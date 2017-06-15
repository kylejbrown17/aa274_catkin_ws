#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.pos_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
        self.cmd_mode_pub = rospy.Publisher('/turtlebot_controller/command_mode', String, queue_size=0)
        self.vel_sp_pub = rospy.Publisher('/turtlebot_controller/velocity_goal', Float32MultiArray, queue_size=10)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()
        self.has_tag_location = False
        self.state = "INITIAL_STATE"
        self.epsilon = 0.32
        self.agent_x = 0.0
        self.agent_y = 0.0
        self.agent_theta = 0.0
        self.pos_cmd = Float32MultiArray()
        self.vel_cmd = Float32MultiArray()    
        self.pos_cmd.data = [0.0, 0.0, 0.0]
        self.vel_cmd.data = [0.0, 0.0]
        self.COMMAND_MODE = "VELOCITY_COMMAND"
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

    #def gazebo_callback(self, data):
    #    pose = data.pose[data.name.index("mobile_base")]
    #    self.agent_x = pose.position.x
    #    self.agent_y = pose.position.y
    #    quaternion = (pose.orientation.x,
    #                  pose.orientation.y,
    #                  pose.orientation.z,
    #                  pose.orientation.w)
    #    self.trans_broad.sendTransform((pose.position.x,pose.position.y,0),
    #                  quaternion,
    #                  rospy.Time.now(),
    #                  "mobile_base",
    #                  "world")

    def loop(self):
        try:
            # tf knows where the tag is
            #(translation, rotation) = self.trans_listener.lookupTransform("/world", "/tag_0", rospy.Time(0))
            (translation, rotation) = self.trans_listener.lookupTransform("/map", "/tag_0", rospy.Time(0))
            self.has_tag_location = True
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # tf doesn't know where the tag is
            translation = (0,0,0)
            rotation = (0,0,0,1)
            self.has_tag_location = False

        
            ######################################
            # Your state machine logic goes here #
            ######################################
#        pos_cmd = Float32MultiArray()
#        vel_cmd = Float32MultiArray()    
#        pos_cmd.data = [0.0, 0.0, 0.0]
#        vel_cmd.data = [0.0, 0.0]
#        COMMAND_MODE = "VELOCITY_COMMAND"
        
            ########## Initial State ##########
        if self.state == "INITIAL_STATE":
            rospy.loginfo(self.state)
            self.state = "LOOKING_FOR_STATION"
                
            ####### LookingForStation #########
        if self.state == "LOOKING_FOR_STATION":
            rospy.loginfo(self.state)
            self.COMMAND_MODE = "VELOCITY_COMMAND"
            V = 0.0
            om = 0.3
            #vel_cmd = Float32MultiArray()
            self.vel_cmd.data = [V, om]
            
            if self.has_tag_location == True:
                self.state = "MOVING_TOWARD_STATION" # switch states
            else:
                self.state = "LOOKING_FOR_STATION" # stay
                    
            ###### MovingTowardStation #######
        if self.state == "MOVING_TOWARD_STATION":
            rospy.loginfo(self.state)
            self.COMMAND_MODE = "POSITION_COMMAND"
            # set values of x_g, y_g, th_g - goal in world frame
            x_g = translation[0]
            y_g = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            th_g = euler[2]

            self.pos_cmd.data = [x_g, y_g, th_g]

            #rospy.loginfo(pos_cmd.data)
            try:
                (agent_translation,agent_rotation) = self.trans_listener.lookupTransform("/map","/base_footprint",rospy.Time(0))
                self.agent_x = agent_translation[0]
                self.agent_y = agent_translation[1]
                euler = tf.transformations.euler_from_quaternion(agent_rotation)
                self.agent_theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
            dist_to_charger = (self.agent_x - x_g)**2 + (self.agent_y - y_g)**2
            #rospy.loginfo("dist_to_charger", dist_to_charger)
            
            if dist_to_charger < self.epsilon**2:
                self.state = "CHARGING"            
            elif self.has_tag_location == True:
                self.state = "MOVING_TOWARD_STATION" # stay
            else:
                self.state = "LOOKING_FOR_STATION" # switch states

                
            ############# Charging ############
        if self.state == "CHARGING":
            rospy.loginfo(self.state)
            self.COMMAND_MODE = "VELOCITY_COMMAND"
            self.vel_cmd.data = [0.0, 0.0]


        self.cmd_mode_pub.publish(self.COMMAND_MODE)
        self.pos_sp_pub.publish(self.pos_cmd)
        self.vel_sp_pub.publish(self.vel_cmd)

        
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
