#!/usr/bin/env python3

from serial.tools import list_ports
import time
from pydobot import Dobot
import rospy
import math as m
from dobot.srv import NotePoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
import tf


rospy.init_node('rviz_marker')

marker = Marker()
marker.header.frame_id = "base"
suck_on = False
color = ''

def spawn_marker(data):
    
    marker.type = 2
    marker.id = 0
    marker.scale.x = 0.035
    marker.scale.y = 0.035
    marker.scale.z = 0.035

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    if color == "r":
        marker.color.r = 1.0
    elif color == "g":
        marker.color.g = 1.0
    elif color == "b":
        marker.color.b = 1.0

    marker.pose.position.x = -data.x / 1000
    marker.pose.position.y = -data.y / 1000
    marker.pose.position.z = 0.02

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)

def suck_control(msg):
    global suck_on
    if msg.data:
        suck_on = True
    else:
        marker.pose.position.z = 0.015
        pub.publish(marker)
        suck_on = False
    
def move_marker(event):
    if suck_on:
        trans, rot = listener.lookupTransform("base", "gripp", rospy.Time(0))
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]-0.02
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)


def set_color(msg):
    global color
    color = msg.data

sub = rospy.Subscriber("/ball_pos", Point, spawn_marker)
sub_suck = rospy.Subscriber("/suck_on", Bool, suck_control)
sub_color = rospy.Subscriber("/ball_color", String, set_color)
pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
timer = rospy.Timer(rospy.Duration(0.001), move_marker)
listener = tf.TransformListener()


while not rospy.is_shutdown():
    rospy.spin()




        

            

            
        


