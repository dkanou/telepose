#!/usr/bin/env python

##########################
__author__  = "Emily-Jane Rolley-Parnell"
__email__  = "erolleyp@gmail.com"
__credits__ = ["Emily-Jane Rolley-Parnell"]
__version__ = "1.0.0"
__date__ = "31/08/18"
__usage__ = "Outdated"


# Executable code that republishes the left and right wrists from keypoints3d node
# Input = /keypoints ROS topic
# Output = /openpose/right_arm
#          /openpose/left_arm
##########################

import rospy
import numpy
import math
import actionlib
import geometry_msgs.msg as geomsg
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from openpose_ros_msgs.msg import Persons
COCONAME = ["nose", "neck","right_shoulder","right_elbow","right_wrist","left_shoulder","left_elbow","left_wrist","right_hip","right_knee","right_ankle","left_hip","left_knee","left_ankle","right_eye","left_eye","right_ear","left_ear","background"]

COCO18 = {i : s for s,i in enumerate(COCONAME)}


def newMessageReceived(msg):
    rospy.loginfo("Message CallBack")
    WristRight = Pose()
    WristLeft = Pose()
    if len(msg.person) == 0:
        rospy.loginfo("No people detected")
    else:
        WristRight = msg.person[0].body_part.poses[COCO18["right_wrist"]]
        WristLeft = msg.person[0].body_part.poses[COCO18["left_wrist"]]

    try:
        
        rospy.loginfo("Right Wrist x,y,z:" + str(WristRight.position.x)+ " , " + str(WristRight.position.y) + " , " + str(WristRight.position.z))
        rospy.loginfo("Left Wrist x,y,z :" + str(WristLeft.position.x) + " , " + str(WristLeft.position.y) + " , " + str(WristLeft.position.z))
        
        #Publish left and right
        pubR.publish(WristRight)
        pubL.publish(WristLeft)
          
    except (rospy.ROSException):
        rospy.logwarn('ROS Exception')

if __name__ == '__main__':

  try:
    # Initialize node
    rospy.init_node("optoros")
    
    # Create publisher and subscriber: [reference]
    inputTopic = rospy.resolve_name("/keypoints")
    sub = rospy.Subscriber(inputTopic,Persons, newMessageReceived, queue_size=5)

    #Resolve topic names
    outputTopicRight = rospy.resolve_name('/openpose/right_arm')
    outputTopicLeft = rospy.resolve_name('/openpose/left_arm')
    
    pubR = rospy.Publisher(outputTopicRight, Pose, queue_size=5)
    pubL = rospy.Publisher(outputTopicLeft, Pose, queue_size=5)
    # Spin ROS
    rospy.spin()
    
  except rospy.ROSInterruptException:
    print("program interrupted before completion")
