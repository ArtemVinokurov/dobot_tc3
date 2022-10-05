#!/usr/bin/python3


import time
from aruco_msgs import msg
from blobs_msgs.msg import BlobArray
from geometry_msgs.msg import Point
import cv2
import rospy
import numpy as np
from dobot.srv import NotePoint, MoveBall
import time
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String



class MvBall:
    def __init__(self):
        self.sub = rospy.Subscriber('/blobs', BlobArray, self.callback_func)
        self.set_ball_service = rospy.Service('MoveBall', MoveBall, self.set_ball_move)
        self.pub = rospy.Publisher('/Point', Point, queue_size=10)
        self.pub_ball_pose = rospy.Publisher("/ball_pos", Point, queue_size=2)
        self.pub_ball_color = rospy.Publisher("/ball_color", String, queue_size=1)

        self.blobs = BlobArray()
        self.XYZ = np.zeros(3)

        self.cam_mat = np.array([[1403.995, 0, 703.401], [0, 1399.73, 535.85], [0, 0, 1]], dtype=np.float)

        dist_coeff = np.array([0.006755, 0.46762, -0.001648, -0.0029868, -2.04127], dtype=np.float)

        self.newcam_mat, roi = cv2.getOptimalNewCameraMatrix(self.cam_mat, dist_coeff, (480, 480), 1, (480,480))

        self.inv_newcam_mtx = np.linalg.inv(self.newcam_mat)

        world_points = np.array([[223, -53.27, 0], [223.62, -9.51, 0], [224, 34.46, 0], [224, 76.85, 0], [221.8, 121.72, 0], 
                                     [271, -53, 0], [273, -9.47, 0], [272, 33.17, 0], [272, 77, 0], [272, 120.96, 0]], dtype=np.float)


        image_points = np.array([[388, 259], [340, 258], [293, 258], [245, 258], [198, 256],
                                 [387, 202], [340, 201], [292, 201], [245, 201], [199, 200]], dtype=np.float)
        

        retval, rvec, self.tvec = cv2.solvePnP(world_points, image_points, self.newcam_mat, dist_coeff)

        self.R_mtx, jac = cv2.Rodrigues(rvec)

        self.inv_R_mtx = np.linalg.inv(self.R_mtx)

        self.z = 0.42

    def set_ball_move(self, data):
        id = data.id
        for i in range(3):
            if self.blobs.blobs[i].id == id:
                u = self.blobs.blobs[i].pose.x
                v = self.blobs.blobs[i].pose.y

                msg_color = String()
                if id == 0:
                    msg_color.data = 'r'
                elif id == 1:
                    msg_color.data = 'g'
                elif id == 2:
                    msg_color.data = 'b'

                self.pub_ball_color.publish(msg_color)
                XYZ = self.calc_XYZ(u, v)
                rospy.loginfo("Ball position id=%s: x=%s y=%s", id, int(XYZ[0]), int(XYZ[1]))
                
                if data.move:
                    self.move_ball(XYZ)
                    rospy.loginfo("Move ball done!")
                return True

        rospy.loginfo("Error: id not found")
        return False

    def calc_XYZ(self, u, v, z = 0.0):
        uv = np.array([[u,v,1]], dtype=np.float)
        uv = np.transpose(uv)
        tempMat = np.matmul(np.matmul(self.inv_R_mtx, self.inv_newcam_mtx), uv)
        tempMat2 = np.matmul(self.inv_R_mtx, self.tvec)

        s = (0.42 + tempMat2[2, 0]) / tempMat[2, 0]

        XYZ = np.matmul(self.inv_R_mtx, (np.matmul(s * self.inv_newcam_mtx, uv) - self.tvec))
        
        msg_pos = Point()
        msg_pos.x = XYZ[0]
        msg_pos.y = XYZ[1]
        msg_pos.z = XYZ[2]
        self.pub_ball_pose.publish(msg_pos)
        
        return XYZ

    def move_manip(self, X, Y, Z, suck):
        rospy.wait_for_service('SetPoint')
        try:
            set_point = rospy.ServiceProxy('SetPoint', NotePoint)
            result = set_point(X, Y, Z, suck)
            if result == False:
                rospy.loginfo(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        
        
    def callback_func(self, msg):
        self.blobs = msg


    def move_ball(self, XYZ):
        self.move_manip(int(XYZ[0]), int(XYZ[1]), 0, False)
        self.move_manip(int(XYZ[0]), int(XYZ[1]), -70, True)
        self.move_manip(int(XYZ[0]), int(XYZ[1]), 0, True)
        self.move_manip(150, -155, 0, True)
        self.move_manip(150, -155, -40, False)
        self.move_manip(150, -155, 0, False)
        self.move_manip(0, -200, 0, False)

    


if __name__ == '__main__':
    rospy.init_node('MoveBall')
    node = MvBall()
    rospy.spin()

        

            

            
        


