#!/usr/bin/python3
import time
import rospy
from math import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class JointsCrt:
	def __init__(self):

		self.q0 = [0.0, 0.0, 0.0]
		
		self.sub = rospy.Subscriber("/Point", Point, self.callback)
		self.pub = rospy.Publisher("/joints", Point, queue_size=10)

		self.q_dot_max = 1.57 / 2

	def callback(self,data):
		q = [data.x, data.y, data.z]
		print(q)

		time_move = self.calc_time(q)
		print("time move: %s", time_move)

		delta_t = 0.001
		t = 0
		coeff_a = self.calc_coeff(q, time_move)
		q_curr = self.q0
		
		msg=Point()
		while t < time_move:
			t += delta_t
			for i in range(3):
				q_curr[i] = coeff_a[i][0] + coeff_a[i][1] * t + coeff_a[i][2] * pow(t, 2) + coeff_a[i][3] * pow(t, 3)
			msg.x = q_curr[0]
			msg.y = q_curr[1]
			msg.z = q_curr[2]
			self.pub.publish(msg)
		
			time.sleep(delta_t)

		self.q0 = q_curr	

	def calc_time(self, q):
		time_move = 0
		for i in range(3):
			time_move_new = 3 * (abs(q[i] - self.q0[i])) / (2 * self.q_dot_max)
			if time_move_new > time_move:
				time_move = time_move_new
		return time_move



	def calc_coeff(self, q, time_move):
		mat_coeff = []
		for i in range(3):
			a0 = self.q0[i]
			a1 = 0
			a2 = 3 * (q[i] - self.q0[i]) / pow(time_move, 2)
			a3 = -2 * (q[i] - self.q0[i]) / pow(time_move, 3)
			row = [a0, a1, a2, a3]
			mat_coeff.append(row)
		return mat_coeff
			

if __name__ == '__main__':
    rospy.init_node('TrajPlan')
    JointsCrt()
    rospy.spin()
