#!/usr/bin/env python3

from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint, GoHome
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from math import *




class DobotControl:
    port = list_ports.comports()[0].device 
    device = Dobot(port=port, verbose=False)
    
    def __init__(self):
        self.move_point_service = rospy.Service('SetPoint', NotePoint, self.SetPoint)
        self.home_init_service = rospy.Service('HomeInit', GoHome, self.HomeInit)
        self.pub = rospy.Publisher("/Point", Point, queue_size=10)
        self.pub_pose = rospy.Publisher("/robot_pose", Point, queue_size=10)
        self.pub_suck = rospy.Publisher("/suck_on", Bool, queue_size=2)
        period = rospy.Duration(0.01)
        #self.timer = rospy.Timer(period, self.robot_pose)

        self.l1 = 80
        self.l2 = 0
        self.l3 = 135
        self.l4 = 147
        self.l5 = 55
        self.l6 = 70

        self.x = 0
        self.y = 0
        self.z = 0

        self.init_joint_states = [-1.57, 0.462, 0.462]

    def SetPoint(self, data):

        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5
        l6 = self.l6

        X = data.x
        Y = data.y
        Z = data.z
        alpha1 = atan2(Y,X)
        if X != 0:
            x = X/cos(alpha1) - l2 - l5
        else:
            x = abs(Y) - l2 - l5
        z = Z + l6 - l1
        d = sqrt(x*x+z*z)

        if (d != 0):
            gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
            beta = gamma + atan(z/x)
            alpha2 = pi/2 - beta
            gamma1 = acos((l3*l3+l4*l4-d*d)/(2*l3*l4))
            alpha3 = gamma1 - alpha2

            msg = Point()
            msg_suck = Bool()
            msg.x = alpha1
            msg.y = alpha2
            msg.z = -(alpha3-pi/2)
            msg_suck.data = data.suck

            self.pub.publish(msg)
            self.device.move_to(X, Y, Z, 0, wait=True)
            self.device.suck(data.suck)
            self.pub_suck.publish(msg_suck)
            return True
        else:
            return False

    def HomeInit(self, data):
        try:
            (x, y, z, r, j1, j2, j3, j4) = self.device.pose()
            self.device.move_to(x, y, 0, 0, wait=False)
            self.device._set_home_cmd()
            self.device.move_to(0, -200, 0, 0, wait=False)
            msg_init = Point()
            msg_init.x = self.init_joint_states[0]
            msg_init.y = self.init_joint_states[1]
            msg_init.z = self.init_joint_states[2]
            self.pub.publish(msg_init)
            return True
        except ConnectionError as e:
            print("Home init failed: %s"%e)
            return False


    def robot_pose(self, event):
        self.x, self.y, z, r, j1, j2, j3, j4 = self.device.pose()
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self.pub_pose.publish(msg)



if __name__=='__main__':

    rospy.init_node('DobotControl')
    node = DobotControl()
    rospy.spin()

