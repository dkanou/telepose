#!/usr/bin/env python


##########################
__author__  = "Emily-Jane Rolley-Parnell"
__email__  = "erolleyp@gmail.com"
__credits__ = ["Emily-Jane Rolley-Parnell"]
__version__ = "1.0.0"
__date__ = "31/08/18"
__usage__ = "Not Working"

# Executable code that republishes the left and right wrists, left and right elbows, and left and right hand arrays from keypoints3d node
# Input = /keypoints ROS topic
# Output = /openpose/right_arm
#          /openpose/left_arm
#          /openpose/right_elbow
#          /openpose/left_elbow
#          /openpose/right_hand
#          /openpose/left_hand
##########################

import rospy
import numpy
import math
import geometry_msgs.msg as geomsg
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from openpose_ros_msgs.msg import Persons
COCONAME =["nose","neck","right_shoulder","right_elbow","right_wrist","left_shoulder","left_elbow","left_wrist","right_hip","right_knee","right_ankle","left_hip","left_knee","left_ankle","right_eye","left_eye","right_ear","left_ear","background"]

COCO18 = {i : s for s,i in enumerate(COCONAME)}
H_key_num = 20

def newMessageReceived(msg):
    rospy.loginfo("Message CallBack")
    WristRight = Pose()
    WristLeft = Pose()
    ElbowRight = Pose()
    ElbowLeft = Pose()
    HandRight = PoseArray()
    HandLeft = PoseArray()

    if len(msg.person) == 0:
        rospy.loginfo("No people detected")
    else:
        WristRight = msg.person[0].body_part.poses[COCO18["right_wrist"]]
        WristLeft = msg.person[0].body_part.poses[COCO18["left_wrist"]]
        ElbowRight = msg.person[0].body_part.poses[COCO18["right_elbow"]]
        ElbowLeft = msg.person[0].body_part.poses[COCO18["left_elbow"]]
        if len(msg.person[0].left_hand_part.poses) != 0:
            for i in range(H_key_num):
                HandLeft.poses.append(msg.person[0].left_hand_part.poses[i])
        else:
            rospy.loginfo('Left hand not detected')
            
        if len(msg.person[0].right_hand_part.poses) != 0:            
            for i in range(H_key_num):
                HandRight.poses.append(msg.person[0].right_hand_part.poses[i])
        else:
            rospy.loginfo('Right hand not detected')

    try:
      
        pubR.publish(WristRight)
        pubL.publish(WristLeft)

        pubRHand.publish(HandRight)
        pubLHand.publish(HandLeft)

        pubRElbow.publish(ElbowRight)
        pubLElbow.publish(ElbowLeft)
          
    except (rospy.ROSInterruptException):
        rospy.logwarn('Ros interrupted')

if __name__ == '__main__':

  try:
    # Initialize node
    rospy.init_node("optoros_hand")
    
    # Create publisher and subscriber: [reference]
    inputTopic = rospy.resolve_name("/keypoints")
    sub = rospy.Subscriber(inputTopic,Persons, newMessageReceived, queue_size=5)

    outputTopicRight = rospy.resolve_name('/openpose/right_arm')
    outputTopicLeft = rospy.resolve_name('/openpose/left_arm')
    outputElbowRight = rospy.resolve_name('/openpose/right_elbow')
    outputElbowLeft = rospy.resolve_name('/openpose/left_elbow')
    outHandRight = rospy.resolve_name('/openpose/right_hand')
    outHandLeft = rospy.resolve_name('/openpose/left_hand')
    
    pubRElbow = rospy.Publisher(outputElbowRight, Pose, queue_size=5)
    pubLElbow = rospy.Publisher(outputElbowLeft, Pose, queue_size=5)
    pubR = rospy.Publisher(outputTopicRight, Pose, queue_size=5)
    pubL = rospy.Publisher(outputTopicLeft, Pose, queue_size=5)
    pubRHand = rospy.Publisher(outHandRight, PoseArray, queue_size=5)
    pubLHand = rospy.Publisher(outHandLeft, PoseArray, queue_size=5)
    # Spin ROS
    rospy.spin()
    
  except rospy.ROSInterruptException:
    print("program interrupted before completion")
