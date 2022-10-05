#!/usr/bin/env python3
from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint
from geometry_msgs.msg import Point
pub = rospy.Publisher('/Point', Point, queue_size=10)
port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)
l1 = 138 
l2 = 0
l3 = 135.013 
l4 = 147.085 
l5 = 31 
l6 = 13 




def SetPoint(data):
    X = data.x
    Y = data.y
    Z = data.z
    if X == 0:
        if Y > 0:
            alpha1 = m.pi/2
        if Y < 0:
            alpha1 = -m.pi/2
    else:
        alpha1 = m.atan2(Y, X)

    x = X - (l2 + l5)*m.cos(alpha1)
    y = Y - (l2 + l5)*m.sin(alpha1)
    z = Z + l6 - l1
    k = m.sqrt(m.pow(x, 2)+m.pow(y, 2))
    d = m.sqrt(m.pow(k, 2)+m.pow(z, 2))
    if (k != 0 and d != 0):
        alpha2 = m.pi/2 - \
            m.atan2(z, k) - m.acos((m.pow(d, 2) +
                                    m.pow(l3, 2)-m.pow(l4, 2))/(2*d*l3))
        alpha3 = (
            m.pi - m.acos((m.pow(l3, 2)+m.pow(l4, 2)-m.pow(d, 2))/(2*l4*l3))) - alpha2
        if(True):
            msg = Point()
            msg.x = alpha1
            msg.y = alpha2
            msg.z = alpha3
            pub.publish(msg)
            
            device.move_to(X, Y, Z, 0, wait=True)
            device.suck(data.suck)

            return True
        else:
            return False


rospy.init_node('SetPoint')

service = rospy.Service('SetPoint', NotePoint, SetPoint)

rospy.spin()
