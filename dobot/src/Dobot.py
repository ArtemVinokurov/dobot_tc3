#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
class JointStatePub:
    def __init__(self):
        self.pub = rospy.Publisher("/joint_states_new", JointState, queue_size=10)
        self.number_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback_js)
    def callback_js(self, msg):
        new_msg=JointState()
        new_msg.header = Header()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.name = msg.name
        new_msg.position.append(msg.position[0])
        new_msg.position.append(msg.position[1])
        new_msg.position.append(msg.position[1])
        new_msg.position.append(-msg.position[1]+msg.position[3])
        new_msg.position.append(-new_msg.position[2])
        new_msg.position.append(new_msg.position[3]+msg.position[1])
        new_msg.position.append(-new_msg.position[5])

        new_msg.velocity = msg.velocity
        new_msg.effort = msg.effort
        
        self.pub.publish(new_msg)
if __name__ == '__main__':
    rospy.init_node('ParallelsJointState')

    JointStatePub()
    rospy.spin()