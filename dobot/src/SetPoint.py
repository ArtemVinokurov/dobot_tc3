#!/usr/bin/env python3

import rospy
from math import *
from dobot.srv import NotePoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import time
pub = rospy.Publisher('/Point', Point, queue_size=10)
l1 = 138 
l2 = 0
l3 = 135
l4 = 147
l5 = 55
l6 = 40


def SetPoint(data):
    
    X = data.x
    Y = data.y
    Z = data.z
    alpha1 = atan2(Y,X)
    if X != 0:
        x = X/cos(alpha1) - l2 - l5
    else:
        x = abs(Y) - l2 - l5
    # else:
    #     alpha1 = -pi/2

    z = Z + l6 - l1

    d = sqrt(x*x+z*z)

    # if (k != 0 and d != 0):
    if (d != 0):
        gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
        beta = gamma + atan(z/x)
        alpha2 = pi/2 - beta
        gamma1 = acos((l3*l3+l4*l4-d*d)/(2*l3*l4))
        alpha3 = gamma1 - alpha2
        # if ((alpha1 < 2.108 and (alpha1 > -2.395) and (alpha2 > -0.184) and (alpha2 < 2) and (alpha3 > -0.3403392) and (alpha3 < 2))):
        msg = Point()
        msg.x = alpha1
        msg.y = alpha2
        
        msg.z = -(alpha3-pi/2)
        rospy.loginfo("j1: %s", alpha1)
        rospy.loginfo("j2: %s", alpha2)
        rospy.loginfo("j3: %s", -(alpha3-pi/2))
        pub.publish(msg)
        time.sleep(0.1)

        return True
        # else:
        #     return False

    else:
        return ("Error Point")
    # else:
    #    return("Point outside of working zone")


rospy.init_node('SetPoint')

service = rospy.Service('SetPoint', NotePoint, SetPoint)

rospy.spin()