"""
@file  : drl_gazebo_interface.py
@date  : 2021-11-30
@brief : Interface script to use Gazebo simulation for DRL.
@author: USRG @ KAIST
"""

import rospy

from tf.transformations import euler_from_quaternion, reflection_from_matrix

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, RCOut
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Empty

import copy
import math
import numpy as np

class Interface():
    def __init__(self):
        # Member variables
        self.rate = rospy.Rate(20) # Default 20 Hz, overwritten in the main function
        self.last_request = rospy.Time.now()
        self.current_state = State() # drone mode state
        self.current_odom = Odometry() # pose and velocity
        self.current_pwm = RCOut()
        
        self.rc_channels_num = 4

        self.init_pose = PoseStamped()
        self.init_pose.pose.position.z    = 2
        self.init_pose.pose.orientation.w = 1
        self.cmd_pose = copy.deepcopy(self.init_pose)
        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.z = 0.5
        

        # Subscribers
        self.current_odom_sub = rospy.Subscriber('ground_truth/state', Odometry, self.current_odom_sub_callback) # map frame velocity
        # self.current_odom_sub = rospy.Subscriber('mavros/local_position/odom', Odometry, self.current_odom_sub_callback) # map frame velocity
        self.current_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.current_pose_sub_callback)
        self.current_velocity_body_sub = rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.current_velocity_body_sub_callback) # body frame velocity
        self.current_velocity_odom_sub = rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.current_velocity_odom_sub_callback) # map frame velocity
        self.current_pwm_sub = rospy.Subscriber('mavros/rc/out', RCOut, self.current_pwm_sub_callback)

        # Publishers
        self.local_pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1) # map frame
        self.cmd_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1) # map frame

        # Services
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.reset_world_client = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        self.stuck_count = 0
        self.last_stuck_count = 0

    def run(self):
        """
        Main algorithm
        """
        self.reset_world()

    def reset_world(self):
        """
        Reset Gazebo world including the drone
        TODO Test this function yourselves.
        """
        if self.get_reset_condition(self.current_odom, self.current_pwm):
            while not rospy.is_shutdown():
                if  rospy.Time.now() - self.last_request > rospy.Duration(1):
                    self.reset_world_client()
                    self.last_request = rospy.Time.now()
                    """
                    TODO Add disarm and re-arming if needed
                    """
                    rospy.loginfo("Resetting Gazebo world.")
                elif self.is_at_origin(self.current_odom.pose.pose.position) is True:
                    self.stuck_count = 0
                    break
                self.rate.sleep() # TODO set rate?

    def get_reset_condition(self, odom, rcout):
        """
        Define your reset condition
        """
        reset_condition = self.is_flipped(odom.pose.pose.orientation) \
                            or self.is_stuck(odom.twist.twist.linear, rcout.channels[:self.rc_channels_num])
        return reset_condition

    def is_flipped(self, orientation):
        roll, pitch, _ = self.get_rotation(orientation)
        return abs(roll) > math.pi/2.0 or abs(pitch) > math.pi/2.0

    def is_stuck(self, velocity, pwm):
        """
        Considered to be stuck if almost zero speed with a lot of effort
        on only some of the rotors for a period of time
        """
        # "count_thres/self.rate" seconds
        count_thres = 200 
        speed_sq = velocity.x ** 2 + velocity.y**2 + velocity.z**2 # speed squared
        if len(pwm) > 0:
            if max(pwm) > 1800 and min(pwm) < 1300 and speed_sq < 0.2:
                # stuck count is clipped between 0 to count_thres
                self.stuck_count = np.clip(self.stuck_count + 1, 0, count_thres)
            else:
                self.stuck_count = np.clip(self.stuck_count - 1, 0, count_thres)
            if not self.stuck_count == self.last_stuck_count:
                rospy.loginfo("stuck count: "+str(self.stuck_count))
            self.last_stuck_count = self.stuck_count
            if self.stuck_count >= count_thres:
                return True
            else:
                return False
        else:
            return False

    def is_at_origin(self, position):
        """
        Return true if the drone is close to the origin
        """
        x0 = 0.0
        y0 = 0.0
        z0 = 0.0
        distance_thres = 2.4 # 2.4 m

        x = position.x
        y = position.y
        z = position.z

        distance_to_origin_sq = (x - x0)**2 + (y - y0)**2 + (z - z0)**2 # distance squared

        if distance_to_origin_sq > distance_thres**2:
            return False
        else:
            return True

    def get_rotation(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return roll, pitch, yaw

    def current_odom_sub_callback(self, msg):
        self.current_odom = msg

    def current_pose_sub_callback(self, msg):
        pass

    def current_velocity_body_sub_callback(self, msg):
        pass

    def current_velocity_odom_sub_callback(self, msg):
        pass

    def current_pwm_sub_callback(self, msg):
        self.current_pwm = msg

if __name__ == '__main__':
    rospy.init_node('drl_gazebo_interface',anonymous=False)
    interface = Interface()


    rospy.loginfo("DRL Gazebo interface node ready")

    # TODO set rate?
    interface.rate = rospy.Rate(20) # 20 hz or whatever
    while not rospy.is_shutdown():
        interface.run()
        interface.rate.sleep()

