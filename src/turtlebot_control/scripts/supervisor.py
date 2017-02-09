#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.pos_sp_pub = rospy.Publisher('/turtlebot_control/position_goal', Float32MultiArray, queue_size=10)
        self.cmd_mode_pub = rospy.Publisher('/turtlebot_control/command_mode', String, queue_size=0)
        self.vel_sp_pub = rospy.Publisher('/turtlebot_control/velocity_goal', Float32MultiArray, queue_size=10)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()
        self.has_tag_location = False
        self.state = "INITIAL_STATE"
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

    def gazebo_callback(self, data):
        pose = data.pose[data.name.index("mobile_base")]
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        self.trans_broad.sendTransform((pose.position.x,pose.position.y,0),
                      quaternion,
                      rospy.Time.now(),
                      "mobile_base",
                      "world")

    def loop(self):
        try:
            # tf knows where the tag is
            (translation, rotation) = self.trans_listener.lookupTransform("/world", "/tag_0", rospy.Time(0))
            self.has_tag_location = True
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # tf doesn't know where the tag is
            translation = (0,0,0)
            rotation = (0,0,0,1)
            self.has_tag_location = False

        
            ######################################
            # Your state machine logic goes here #
            ######################################
        pos_cmd = Float32MultiArray()
        vel_cmd = Float32MultiArray()    
        pos_cmd.data = [0.0, 0.0, 0.0]
        vel_cmd.data = [0.0, 0.0]
        COMMAND_MODE = "VELOCITY_COMMAND"
        
            ########## Initial State ##########
        if self.state == "INITIAL_STATE":
            self.state = "LOOKING_FOR_STATION"
                
            ####### LookingForStation #########
        if self.state == "LOOKING_FOR_STATION":
            COMMAND_MODE = "VELOCITY_COMMAND"
            V = 0.0
            om = 0.1
            #vel_cmd = Float32MultiArray()
            vel_cmd.data = [V, om]
            if self.has_tag_location == True:
                self.state = "MOVING_TOWARD_STATION" # switch states
            else:
                self.state = "LOOKING_FOR_STATION" # stay
                    
            ###### MovingTowardsStation #######
        if self.state == "MOVING_TOWARD_STATION":
            COMMAND_MODE = "POSITION_COMMAND"
            #rospy.loginfo(COMMAND_MODE)
            # set values of x_g, y_g, th_g - HOW?
            x_g = translation[0]
            y_g = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            th_g = euler[2]
            #pos_cmd = Float32MultiArray()
            #pos_cmd.data = np.array([x_g, y_g, th_g])
            pos_cmd.data = [x_g, y_g, th_g]
            #rospy.loginfo(COMMAND_MODE)
            #rospy.loginfo(pos_cmd)
            if self.has_tag_location == True:
                self.state = "MOVING_TOWARD_STATION" # stay
            else:
                self.state = "LOOKING_FOR_STATION" # switch states
                #if current_position - destination < epsilon:
                #    state = "CHARGING"
                
            ############# Charging ############
        if self.state == "CHARGING":
            COMMAND_MODE = "VELOCITY_COMMAND"
            #vel_cmd.data = np.array([0.0, 0.0])
            vel_cmd.data = [0.0, 0.0]
            print "done"

        self.cmd_mode_pub.publish(COMMAND_MODE)
        self.pos_sp_pub.publish(pos_cmd)
        self.vel_sp_pub.publish(vel_cmd)

        
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
