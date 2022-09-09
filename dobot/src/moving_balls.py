#!/usr/bin/env python3

from requests import request
from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint
from geometry_msgs.msg import Point


def mov(X, Y, Z, suck=False):
    rospy.wait_for_service('SetPoint')
    try:
        set_point = rospy.ServiceProxy('SetPoint', NotePoint)
        result = set_point(X, Y, Z, suck)
        if result == False:
            rospy.loginfo(result)
        time.sleep(1.0)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    # time.sleep(5)

if __name__ == "__main__":
    
    points = [[-120, 120, 0],[120, 120, 0],[200, 120, 0],[-200, 120, 0]]
    suck = False
    for i in range (len(points)):
        X = points[i][0]
        Y = points[i][1]
        Z = points[i][2]
        mov(X, Y, Z)
        if i % 2 == 0:
            mov(X, Y, 50, True)
            mov(X, Y, Z, True)
        else:
            mov(X, Y, 50, False)
            mov(X, Y, Z)

        

            

            
        


