#!/usr/bin/env python3
from email.header import Header
from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint, GoHome
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


l1 = 138 
l2 = 0
l3 = 135.013 
l4 = 147.085 
l5 = 31 
l6 = 13 



class DobotControl:
    port = list_ports.comports()[0].device 
    device = Dobot(port=port, verbose=False)
    
    def __init__(self):
        self.move_point_service = rospy.Service('SetPoint', NotePoint, self.SetPoint)
        self.home_init_service = rospy.Service('HomeInit', GoHome, self.HomeInit)
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.dobot_joint_state_pub)
        self.calibration = False





    def SetPoint(self, data):
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
            # if ((alpha1 < 2.108 and (alpha1 > -2.395) and (alpha2 > -0.184) and (alpha2 < 2) and (alpha3 > -0.3403392) and (alpha3 < 2))):
                # msg = Point()
                # msg.x = alpha1
                # msg.y = alpha2
                # msg.z = alpha3
                # self.pub.publish(msg)
                
                self.device.move_to(X, Y, Z, 0, wait=True)
                self.device.suck(data.suck)
                time.sleep(2)

                return True
            else:
                return False

    def HomeInit(self, data):
        self.calibration = True
        try:
            (x, y, z, r, j1, j2, j3, j4) = self.device.pose()
            self.device.move_to(x, y, 0, wait=False)
            self.device.set_home_cmd()
            time.sleep()
            self.device.move_to(0, -200, 0, wait=False)
            self.calibration = False
            return True
        except ConnectionError as e:
            print("Home init failed: %s"%e)
            return False


    def dobot_joint_state_pub(self, event):
        if not self.calibration:
            (x, y, z, r, j1, j2, j3, j4) = self.device.pose()
            j1 = float(j1)
            j2 = float(j2)
            j3 = float(j3)
            joints_arr = [j1, j2, j3]
        
            msg=JointState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.name = ['0-1','1-2','1-2_1','2-3','2_1-triangle','triangle-3_1','3_1-4']
        
            msg.position.append(j1) #0-1
            msg.position.append(-j2+1) #1-2
            msg.position.append(-j2+1) #1-2_1
            msg.position.append(-j3+0.65) #2-3
            msg.position.append(-msg.position[2]) #2_1_triangle
            msg.position.append(msg.position[3]+j2) #triangle-3_1
            msg.position.append(-msg.position[5]) #3_1-4
            print(joints_arr)
            self.pub.publish(msg)


if __name__=='__main__':

    rospy.init_node('DobotControl')
    rate = rospy.Rate(100)
    node = DobotControl()
    rospy.spin()

