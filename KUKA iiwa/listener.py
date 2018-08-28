#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import numpy as np

def publisher():
    global count
    global xmlat
    global ymlat
    global zmlat

    # Preparing data for publishing to KUKA iiwa controller
    count = 0
    xset = xmlat
    yset = ymlat
    zset = zmlat

    # Initialising objects used to communicate with KUKA iiwa controller
    pub = rospy.Publisher('/iiwa_2/command/JointPosition', JointPosition, queue_size=10)
    joint = JointPosition()

    # Assigning the coordinates meant for sending to KUKA iiwa controller
    joint.header.seq = count
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ""
    joint.position.a2 = -45.0 * 3.1415/180
    joint.position.a3 = 0.0 * 3.1415/180
    joint.position.a4 = 90.0 * 3.1415/180
    joint.position.a5 = 0.0 * 3.1415/180
    joint.position.a7 = 0.0 * 3.1415/180
    joint.position.a6 = 45.0 * 3.1415/180

    # Coordinate responsible for rotation in horizontal plane
    joint.position.a1 = np.arctan2(yset/xset)

    # Sending coordinates to KUKA iiwa controller
    rospy.loginfo(joint)
    pub.publish(joint)
    count += 1

def callback3(data):
    global xmlat
    global ymlat
    global zmlat

    # Recording coordinates published by localisation program
    xmlat = data.data[0]
    ymlat = data.data[1]
    zmlat = data.data[2]

    # Printing the coordinates in terminal
    rospy.loginfo('callback3: %f, %f, %f', data.data[0], data.data[1], data.data[2])
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    # Initialising node
    rospy.init_node('subscriberMLAT', anonymous=True)
    rospy.Subscriber("position", Float64MultiArray, callback3)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
