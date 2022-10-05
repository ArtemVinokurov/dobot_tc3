#!/usr/bin/env python3

import rospy
from math import *
from dobot.srv import NotePoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import Bool

pub = rospy.Publisher('/Point', Point, queue_size=10)
pub_suck = rospy.Publisher("/suck_on", Bool, queue_size=2)

l1 = 80 
l2 = 0
l3 = 135
l4 = 147
l5 = 60
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
        msg_suck.data = bool(data.suck)
        pub.publish(msg)
        time.sleep(2)
        pub_suck.publish(msg_suck)

        return True

    else:
        return ("Error Point")
    # else:
    #    return("Point outside of working zone")


rospy.init_node('SetPoint')

service = rospy.Service('SetPointSim', NotePoint, SetPoint)

rospy.spin()