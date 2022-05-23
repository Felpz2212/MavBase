from inspect import modulesbyfile
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
import numpy as np
import math
import time


TOL = 0.5

#Subscriber Publisher and Services
mavros_arm = '/mavros/cmd/arming'
mavros_set_mode = '/mavros/set_mode'
mavros_local_atual = '/mavros/setpoint_position/local'
mavros_state_sub = 'mavros/state'
mavros_local_position_sub = '/mavros/local_position/pose'
mavros_battery_state_sub = '/mavros/battery'
mavros_set_global_pub = '/mavros/setpoint_position/global'
mavros_velocity_pub = '/mavros/setpoint_velocity/cmd_vel'
mavros_local_position_pub = '/mavros_local_position_pub'


#mavros_local_position_pub    = '/mavros/setpoint_position/local'
#mavros_velocity_pub          = '/mavros/setpoint_velocity/cmd_vel'
#mavros_local_atual           = '/mavros/local_position/pose'
#mavros_state_sub             = '/mavros/state'
#mavros_arm                   = '/mavros/cmd/arming'
#mavros_set_mode              = '/mavros/set_mode'
#mavros_battery_sub           = '/mavros/battery'
#extended_state_sub           = '/mavros/extended_state'
#mavros_global_position_sub   = '/mavros/global_position/global'
#mavros_pose_target_sub       = '/mavros/setpoint_raw/local'
#mavros_set_global_pub        = '/mavros/setpoint_position/global'

class MAV:
    rospy.init_node("GOLAND")
    def __init__(self, mav_name):
        self.rate = rospy.Rate(60)
        self.battery_status = BatteryState()
        self.drone_pose = PoseStamped()
        self.drone_goal_pose = PoseStamped()
        self.drone_state = State()
        self.goal_vel = TwistStamped()

        #services
        self.arm = rospy.ServiceProxy(mavros_arm, CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        #publishers
        self.velocity_pub = rospy.Publisher(mavros_velocity_pub, TwistStamped, queue_size=10)
        self.set_global_pub = rospy.Publisher(mavros_set_global_pub, PoseStamped, queue_size=10)
        self.local_position_pub = rospy.Publisher(mavros_local_position_pub, PoseStamped, queue_size=15)

        #subscribers
        self.local_position_sub = rospy.Subscriber(mavros_local_position_sub, PoseStamped, self.local_callback)
        self.state_sub = rospy.Subscriber(mavros_state_sub, State, self.state_callback)
        self.battery_state_sub = rospy.Subscriber(mavros_battery_state_sub, BatteryState, self.battery_callback)

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z
    
    def state_callback(self, state_data):
        self.drone_state = state_data

    def battery_callback(self, bat_data):
        self.battery_status = bat_data

    def set_position(self, x, y, z):
        self.drone_goal_pose.pose.position.x = x
        self.drone_goal_pose.pose.position.y = y
        self.drone_goal_pose.pose.position.z = z

        self.local_position_pub.publish(self.drone_goal_pose)


    def set_vel(self, x, y, z, roll=0, pitch=0, yaw=0):
        self.goal_vel.twist.linear.x = x
        self.goal_vel.twist.linear.y = y
        self.goal_vel.twist.linear.z = z

        self.goal_vel.twist.angular.x = roll
        self.goal_vel.twist.angular.y = pitch
        self.goal_vel.twist.angular.z = yaw
        
        self.velocity_pub.publish(self.goal_vel)
    

    def set_points(self):
        x = float(input("Digita a posição em X: "))
        y = float(input("Digite a posição em Y: "))
        z = float(input("Digite a posição em Z: "))

        self.drone_goal_pose.pose.position.x = x
        self.drone_goal_pose.pose.position.y = y
        self.drone_goal_pose.pose.position.z = z

    def drone_set_mode(self):
        for i in range(100):
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()

        last_request = rospy.Time.now()
        if(self.drone_state.mode != "OFFBOARD"):
            while not rospy.is_shutdown() and self.drone_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
                self.set_mode(0, "OFFBOARD")
            rospy.loginfo("Drone está no modo OFFBOARD")
        else:
            rospy.loginfo("Drone já está no modo OFFBOARD")
        
    
    def takeoff(self):
        self.arm(True)

        if(not self.drone_state.armed):
            while(not self.drone_state.armed):
                self.arm(True)
            rospy.loginfo("Drone Armado")
        else:
            rospy.loginfo("Drone já está armado")
        
        self.set_global_pub.publish(self.drone_goal_pose)

    def chegou(self):
        if(not rospy.is_shutdown and abs(self.drone_goal_pose - self.drone_pose) > TOL):
            return True
        else:
            return False

if __name__ == '__main__':
    mav = MAV("jorge")
    mav.set_points()
    mav.drone_set_mode()
    mav.takeoff()





    


    
        
