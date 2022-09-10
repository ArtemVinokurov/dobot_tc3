#!/usr/bin/env python3

from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint, GoHome
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import *


l1 = 138 
l2 = 0
l3 = 135
l4 = 147
l5 = 55
l6 = 40



class DobotControl:
    port = list_ports.comports()[0].device 
    device = Dobot(port=port, verbose=False)
    
    def __init__(self):
        self.move_point_service = rospy.Service('SetPoint', NotePoint, self.SetPoint)
        self.home_init_service = rospy.Service('HomeInit', GoHome, self.HomeInit)
        self.pub = rospy.Publisher("/Point", Point, queue_size=10)



    def SetPoint(self, data):
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
            msg.x = alpha1
            msg.y = alpha2
            msg.z = -(alpha3-pi/2)
            rospy.loginfo("j1: %s", alpha1)
            rospy.loginfo("j2: %s", alpha2)
            rospy.loginfo("j3: %s", -(alpha3-pi/2))
            self.pub.publish(msg)
            self.device.move_to(X, Y, Z, 0, wait=True)
            self.device.suck(data.suck)
            time.sleep(2)
            return True
        else:
            return False

    def HomeInit(self, data):
        try:
            (x, y, z, r, j1, j2, j3, j4) = self.device.pose()
            self.device.move_to(x, y, 0, 0, wait=False)
            self.device._set_home_cmd()
            self.device.move_to(0, -200, 0, 0, wait=False)
            return True
        except ConnectionError as e:
            print("Home init failed: %s"%e)
            return False




if __name__=='__main__':

    rospy.init_node('DobotControl')
    node = DobotControl()
    rospy.spin()

