from tkinter import Menu
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


TOL = 0.1

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
        self.rate = rospy.Rate(20)
        self.battery_status = BatteryState()
        self.drone_pose = PoseStamped()
        self.drone_home = PoseStamped()
        self.drone_goal_pose = PoseStamped()
        self.current_state = State()
        self.goal_vel = TwistStamped()

        #services
        self.arm = rospy.ServiceProxy(mavros_arm, CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        #publishers
        self.velocity_pub = rospy.Publisher(mavros_velocity_pub, TwistStamped, queue_size=10)
        self.set_global_pub = rospy.Publisher(mavros_set_global_pub, GeoPoseStamped, queue_size=10)
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        #subscribers
        self.local_position_sub = rospy.Subscriber(mavros_local_position_sub, PoseStamped, self.local_callback)
        self.state_sub = rospy.Subscriber(mavros_state_sub, State, self.state_callback)
        self.battery_state_sub = rospy.Subscriber(mavros_battery_state_sub, BatteryState, self.battery_callback)

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z 
    
    def state_callback(self, state_data):
        self.current_state = state_data

    def battery_callback(self, bat_data):
        self.battery_status = bat_data


    #def set_position_target(self, type_mask, x_position=0, y_position=0, z_position=0, x_velocity=0, y_velocity=0, z_velocity=0, x_aceleration=0, y_aceleration=0, z_aceleration=0, yaw=0, yaw_rate=0, coordinate_frame = PositionTarget.FRAME_LOCAL_NED):
    #    self.pose_target.coordinate_frame = coordinate_frame #Use PositionTarget.FRAME_LOCAL_NED para movimento relativo ao corpo do drone  
    #    self.pose_target.type_mask = type_mask
    #    #https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK

    #    self.pose_target.position.x = x_position
    #    self.pose_target.position.y = y_position
    #    self.pose_target.position.z = z_position

    #    self.pose_target.velocity.x = x_velocity
    #    self.pose_target.velocity.y = y_velocity
    #    self.pose_target.velocity.z = z_velocity

    #    self.pose_target.acceleration_or_force.x = x_aceleration
    #    self.pose_target.acceleration_or_force.y = y_aceleration
    #    self.pose_target.acceleration_or_force.z = z_aceleration

    #    self.pose_target.yaw = yaw
    #    self.pose_target.yaw_rate = yaw_rate

    #    self.target_pub.publish(self.pose_target)

    def set_position(self):
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
        self.drone_goal_pose.pose.position.x = float(input("Digita a posição em X: "))
        self.drone_goal_pose.pose.position.y = float(input("Digita a posição em Y: "))
        self.drone_goal_pose.pose.position.z = 5

        print("X: ", self.drone_goal_pose.pose.position.x)
        print("Y: ", self.drone_goal_pose.pose.position.y)
        print("Z: ", self.drone_goal_pose.pose.position.z)

    def drone_set_mode_offboard(self):
        for i in range(100):
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()
        
        last_request = rospy.Time.now()
        if (self.current_state.mode != "OFFBOARD"):
            self.result = self.set_mode(0, "OFFBOARD")
            while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
                self.result = self.set_mode(0, "OFFBOARD")
            rospy.loginfo("Drone em modo OFFBOARD")
        else:
            rospy.loginfo("Drone já está em modo OFFBOARD")
    
    def armar(self):
        self.arm(True)
        if not rospy.is_shutdown() and not self.current_state.armed:
            while not rospy.is_shutdown() and not self.current_state.armed:
                self.arm(True)
            rospy.loginfo("Drone Armado")
        else:
            rospy.loginfo("Drone já está armado")
        

    def desarmar(self):
        self.arm(False)
        if not rospy.is_shutdown() and self.current_state.armed:
            while not rospy.is_shutdown() and self.current_state.armed:
                self.arm(False)
            rospy.loginfo("Drone Desarmado")
        else:
            rospy.loginfo("Drone já está Desarmado")

    def chegou(self):
        if(not rospy.is_shutdown and abs(self.drone_goal_pose - self.drone_pose) > TOL):
            return False
        else:
            return True

    def mission(self):

        resp = Menu()
        quadrado = float(input("Digite a area do quadrado: "))

        for i in range (5):
            self.goal_vel.twist.linear.x = 0
            self.goal_vel.twist.linear.y = 0
            self.goal_vel.twist.linear.z = 0

            self.goal_vel.twist.angular.x = 0
            self.goal_vel.twist.angular.y = 0
            self.goal_vel.twist.angular.z = 0
            self.velocity_pub = rospy.Publisher(mavros_velocity_pub, TwistStamped, queue_size=10)

            self.drone_set_mode_offboard()
            self.armar()

            if i == 0:
                print("POSICAO INICIAL")
                self.set_points()
                self.set_position()
            else:
                self.square(i, quadrado, resp)

            
            while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.z - self.drone_pose.pose.position.z) > TOL:
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()

            rospy.loginfo("Esperando - Z")
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()

            while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.y - self.drone_pose.pose.position.y) > TOL:
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()
            
            rospy.loginfo("Esperando - Y")
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()

            while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.x - self.drone_pose.pose.position.x) > TOL:
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()
            
            rospy.loginfo("Esperando - X")
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
                self.local_position_pub.publish(self.drone_goal_pose)
                self.rate.sleep()


            if self.chegou():
                rospy.loginfo("Drone Descendo")
                if (self.current_state.mode != "AUTO.LAND"):
                    self.result = self.set_mode(0, "AUTO.LAND")
                    rospy.loginfo("Alterando para modo Land")
                    while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
                        self.result = self.set_mode(0, "AUTO.LAND")
                        rospy.loginfo("Drone em modo Land")
                    else:
                        rospy.loginfo("Drone já está em modo Land")
                
            self.desarmar()
            self.rate.sleep()

    def square(self, cont, metro, resp):
        rospy.loginfo("Calculando novo trajento")
        if resp == 1:
            if cont == 1:
                self.drone_goal_pose.pose.position.x -= metro
            elif cont == 2:
                self.drone_goal_pose.pose.position.y += metro
            elif cont == 3:
                self.drone_goal_pose.pose.position.x += metro
            elif cont == 4:
                self.drone_goal_pose.pose.position.y -= metro

        elif resp == 2:
            if cont == 1:
                self.drone_goal_pose.pose.position.x += metro
            elif cont == 2:
                self.drone_goal_pose.pose.position.y += metro
            elif cont == 3:
                self.drone_goal_pose.pose.position.x -= metro
            elif cont == 4:
                self.drone_goal_pose.pose.position.y -= metro

        elif resp == 3:
            if cont == 1:
                self.drone_goal_pose.pose.position.x -= metro
            elif cont == 2:
                self.drone_goal_pose.pose.position.y -= metro
            elif cont == 3:
                self.drone_goal_pose.pose.position.x += metro
            elif cont == 4:
                self.drone_goal_pose.pose.position.y += metro

        elif resp == 4:
            if cont == 1:
                self.drone_goal_pose.pose.position.x += metro
            elif cont == 2:
                self.drone_goal_pose.pose.position.y -= metro
            elif cont == 3:
                self.drone_goal_pose.pose.position.x -= metro
            elif cont == 4:
                self.drone_goal_pose.pose.position.y += metro

    def menu(self):
        teste = 0
        while teste == 0:
            print(" 1)  ....  |  ....  2)")
            print("     :  :  |  :  :")
            print("     '''X  |  X'''")
            print("-----------------------")
            print(" 3)  ...X  |  X...  4)")
            print("     :  :  |  :  :")
            print("     ''''  |  ''''")

            resp = input("Escolha: ")
            if not resp.isnumeric():
                print("Apenas numeros")
            else:
                resp = int(resp)
                if resp > 0 and resp < 5:
                    return resp
                else:
                    print("Opção invalida!")

    def RTL(self):
        self.drone_goal_pose.pose.position.x = self.drone_home.pose.position.x
        self.drone_goal_pose.pose.position.y = self.drone_home.pose.position.y
        self.drone_goal_pose.pose.position.z = 5

        self.goal_vel.twist.linear.x = 0
        self.goal_vel.twist.linear.y = 0
        self.goal_vel.twist.linear.z = 0

        self.goal_vel.twist.angular.x = 0
        self.goal_vel.twist.angular.y = 0
        self.goal_vel.twist.angular.z = 0
        self.velocity_pub = rospy.Publisher(mavros_velocity_pub, TwistStamped, queue_size=10)

        self.drone_set_mode_offboard()
        self.armar()
        self.local_position_pub.publish(self.drone_goal_pose)
        
        rospy.loginfo("Drone Retornando para posição inicial")

        while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.z - self.drone_pose.pose.position.z) > TOL:
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()

        rospy.loginfo("Esperando")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()

        while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.y - self.drone_pose.pose.position.y) > TOL:
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()
        
        rospy.loginfo("Esperando")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()

        while not rospy.is_shutdown() and abs(self.drone_goal_pose.pose.position.x - self.drone_pose.pose.position.x) > TOL:
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()
        
        rospy.loginfo("Esperando")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
            self.local_position_pub.publish(self.drone_goal_pose)
            self.rate.sleep()
        

        if self.chegou():
            if (self.current_state.mode != "AUTO.LAND"):
                self.result = self.set_mode(0, "AUTO.LAND")
                rospy.loginfo("Alterando para modo Land")
                while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
                   self.result = self.set_mode(0, "AUTO.LAND")
                rospy.loginfo("Drone em modo Land")
            else:
                rospy.loginfo("Drone já está em modo Land")

        self.desarmar()


if __name__ == '__main__':
    mav = MAV("robson")

    mav.drone_goal_pose.pose.position.x = 0
    mav.drone_goal_pose.pose.position.y = 0
    mav.drone_goal_pose.pose.position.z = 0

    mav.drone_home.pose.position.x = mav.drone_pose.pose.position.x
    mav.drone_home.pose.position.y = mav.drone_pose.pose.position.y
    mav.drone_home.pose.position.z = mav.drone_pose.pose.position.z

    mav.mission()
    mav.RTL()