#!/usr/bin/env python

##########################
__author__  = "Emily-Jane Rolley-Parnell"
__email__  = "erolleyp@gmail.com"
__credits__ = ["Emily-Jane Rolley-Parnell"]
__version__ = "1.0.0"
__date__ = "31/08/18"
__usage__ = "outdated"


# Executable code that republishes the left and right wrists from an optoros node to the inverse kinematics system Cartesian Interface 
# Input =
#optoros outputs for left and right
# /openpose/right_arm
# /openpose/left_arm
#Left and right states of robot
# /xbotcore/cartesian/arm2_7/state
# /xbotcore/cartesian/arm1_7/state

#Output = 
#Right arm Reference
# /xbotcore/cartesian/arm2_7/reference   
#Left Arm Reference
#/xbotcore/cartesian/arm1_7/reference
##########################
  
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

def startPos(poseCurrent,initPose, count):
    #For taking and average of the first 5 positions of human
    if count == 5:
        initPose.position.x = initPose.position.x/count
        initPose.position.y = initPose.position.y/count
        initPose.position.z = initPose.position.z/count
        return  initPose
    else:
        initPose.position.x = initPose.position.x + poseCurrent.position.x
        initPose.position.y = initPose.position.y + poseCurrent.position.y
        initPose.position.z = initPose.position.z + poseCurrent.position.z
        return initPose

def diffCalcR(posePrev, poseCurrent):
    global posePreR, idxR
    poseDiff = Pose()
    poseDiff.position.x = poseCurrent.position.x - posePrev.position.x
    poseDiff.position.y = poseCurrent.position.y - posePrev.position.y
    poseDiff.position.z = poseCurrent.position.z - posePrev.position.z

    dist = math.sqrt((poseDiff.position.x*poseDiff.position.x) + (poseDiff.position.y*poseDiff.position.y) + (poseDiff.position.z*poseDiff.position.z))
    rospy.loginfo("Euclidian Distance Right: " + str(dist))
    rospy.loginfo("Right differences" + str(poseDiff.position))
    #If the distance is too small, stay and add to cumulation
    if (0.01 <= dist <= 0.55):
        posePreR = poseCurrent
        idxR = 0
    else:
        if (poseCurrent.position.x == 0) : 
            rospy.loginfo('Right Joint NaN')
        else:
            rospy.loginfo('Right Joint Out of Range')
            idxR += 1

        poseDiff.position.x = 0
        poseDiff.position.y = 0
        poseDiff.position.z = 0

        if idxR >= 1:
            posePreR = poseCurrent
            idxR = 0

    return poseDiff

def diffCalcL(posePrev, poseCurrent):
    global posePreL, idxL
    poseDiff = Pose()
    poseDiff.position.x = poseCurrent.position.x - posePrev.position.x
    poseDiff.position.y = poseCurrent.position.y - posePrev.position.y
    poseDiff.position.z = poseCurrent.position.z - posePrev.position.z

    dist = math.sqrt((poseDiff.position.x*poseDiff.position.x) + (poseDiff.position.y*poseDiff.position.y) + (poseDiff.position.z*poseDiff.position.z))
    rospy.loginfo("Euclidian Distance Left: " + str(dist))
    #If the distance is too small, stay and add to cumulation
    rospy.loginfo("Left Differences" + str(poseDiff.position))
    if (0.01 <= dist <= 0.55):
        posePreL = poseCurrent
        idxL = 0
    else:
        if (poseCurrent.position.x == 0) : 
            rospy.loginfo('Left Joint NaN')
        else:
            rospy.loginfo('Left Joint Out of Range')
            idxL += 1

        poseDiff.position.x = 0
        poseDiff.position.y = 0
        poseDiff.position.z = 0
        
        if idxL >= 1:
            posePreL = poseCurrent
            idxL = 0

    return poseDiff

def updateGoal(state, poseDiff):
    goal = PoseStamped()
    goal.pose.position.x = state.pose.position.x + poseDiff.position.x
    goal.pose.position.y = state.pose.position.y + poseDiff.position.y
    goal.pose.position.z = state.pose.position.z + poseDiff.position.z
    goal.pose.orientation.x = state.pose.orientation.x # 1
    goal.pose.orientation.y = state.pose.orientation.y
    goal.pose.orientation.z = state.pose.orientation.z
    goal.pose.orientation.w = state.pose.orientation.w
    return goal

def stateCallbackR(msg):
    global robotStateR, robot_initR, first_pubR
    robotStateR = msg
    if first_pubR == 1:
        robot_initR = msg

def stateCallbackL(msg):
    global robotStateL, robot_initL, first_pubL
    robotStateL = msg
    if first_pubL == 1:
        robot_initL = msg
def poseCorrect(pose):
    temp_x = pose.position.x
    temp_y = pose.position.y
    temp_z = pose.position.z
    pose.position.x = -(temp_z)
    pose.position.y = temp_x
    pose.position.z = -(temp_y)
    return pose
def initRobotStates():

     initLeft, initRight = PoseStamped(), PoseStamped()
 
     print("Do we do this?")
      
     initLeft.header.frame_id = 'world'
     initRight.header.frame_id = 'world'
     initLeft.pose.position.x = 0.524205398657
     initLeft.pose.position.y = 0.189681723327
     initLeft.pose.position.z = 1.213720707
   
     initLeft.pose.orientation.x = -0.0842139378405
     initLeft.pose.orientation.y = -0.93840858675
     initLeft.pose.orientation.z = -0.2382381388
     initLeft.pose.orientation.w = 0.23566910321
 
     initRight.pose.position.x =  0.524205398657
     initRight.pose.position.y = -0.189681723327
     initRight.pose.position.z =  1.213720707
   
     initRight.pose.orientation.x = 0.0842139378405
     initRight.pose.orientation.y = -0.93840858675
     initRight.pose.orientation.z = 0.2382381388
     initRight.pose.orientation.w = 0.23566910321
     
     return initLeft, initRight


def newRightMessageReceived(msg):
    global countR, first_pubR, posePreR, human_initR, robotStateR
    # Define the goal pose to reach with the robot
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    poseCur = poseCorrect(msg)
    poseD = Pose()
    if countR < 6 :
        human_initR = startPos(poseCur, human_initR, countR)
        rospy.loginfo("Init number Right {}".format(countR))
        if poseCur.position.x != 0 or poseCur.position.y != 0 or poseCur.position.z != 0:
            countR += 1
    else:
        try:
            # calculate the distance of the human in the moving robot world frame
            if first_pubR == 0 :

                poseD = diffCalcR(posePreR, poseCur)

            if first_pubR == 1 :
          
                #calculate difference
                poseD = diffCalcR(human_initR, poseCur)
                # not first time to run the loop
                first_pubR = 0

            goal = updateGoal(robotStateR, poseD)
             
            dist = math.sqrt((poseD.position.x*poseD.position.x) + (poseD.position.y*poseD.position.y) + (poseD.position.z*poseD.position.z))    
            # Pub the difference
            pubR.publish(goal) # pub [reference]            

        except (rospy.ROSInterruptException):
            rospy.logwarn('ROS Interrupted')

def newLeftMessageReceived(msg):
    global countL, first_pubL, posePreL, human_initL, robotStateL
    # Define the goal pose to reach with the robot
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    poseCur = poseCorrect(msg)
    poseD = Pose()
    if countL < 6 :
        human_initL = startPos(poseCur, human_initL, countL)
        rospy.loginfo("Init number Left {}".format(countL))
        if poseCur.position.x != 0 or poseCur.position.y != 0 or poseCur.position.z != 0:
            countL += 1
        
    else:
        # Transfrom poses to the world_odom (fixed in the world) frame
        try:
            # calculate the distance of the human in the moving robot world frame
            if first_pubL == 0 :

                #calculate difference
                poseD = diffCalcL(posePreL, poseCur)

            if first_pubL == 1 :
          
                #calculate difference
                poseD = diffCalcL(human_initL, poseCur)
                # not first time to run the loop
                first_pubL = 0

            goal = updateGoal(robotStateL, poseD)

            dist = math.sqrt((poseD.position.x*poseD.position.x) + (poseD.position.y*poseD.position.y) + (poseD.position.z*poseD.position.z))    
            # Pub the difference
            pubL.publish(goal) # pub [reference]            

        except (rospy.ROSInterruptException):
            rospy.logwarn('ROS Interrupted')


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("KeyToXbot")
                 
        # Create publisher for robot right arm
        outputRTopic = rospy.resolve_name('/xbotcore/cartesian/arm2_7/reference')    
        pubR = rospy.Publisher(outputRTopic, PoseStamped, queue_size=10)
        
        # Create publisher for robot Left arm
        outputLTopic = rospy.resolve_name('/xbotcore/cartesian/arm1_7/reference')    
        pubL = rospy.Publisher(outputLTopic, PoseStamped, queue_size=10)

        # Global variables
        posePreR = Pose()
        countR = 0
        idxR = 0
        first_pubR = 1
        posePreL = Pose()
        countL = 0
        idxL = 0
        first_pubL = 1
        # Average starting position
        human_initR = Pose()
        human_initL = Pose()

        robot_initL, robot_initR = initRobotStates()
        robotStateR = PoseStamped()
        robotStateL = PoseStamped()
        
        pubR.publish(robot_initR)
        pubL.publish(robot_initL)
        


        # Create subscriber [Right Wrist location]
        wristRTopic = rospy.resolve_name("/openpose/right_arm")
        subWR = rospy.Subscriber(wristRTopic, Pose, newRightMessageReceived, queue_size=10)
        # Create subscriber [Left Wrist location]
        wristLTopic = rospy.resolve_name("/openpose/left_arm")
        subWL = rospy.Subscriber(wristLTopic, Pose, newLeftMessageReceived, queue_size=10)
        # Create subscriber [Robot right arm current]
        currentRState = rospy.resolve_name('/xbotcore/cartesian/arm2_7/state')
        stateSubR = rospy.Subscriber(currentRState, PoseStamped, stateCallbackR, queue_size=10)      
        # Create subscriber [Robot left arm current]
        currentLState = rospy.resolve_name('/xbotcore/cartesian/arm1_7/state')
        stateSubL = rospy.Subscriber(currentLState, PoseStamped, stateCallbackL, queue_size=10)      

        rospy.loginfo("Hand topics: %s , %s to robot cartesian_interface topics: %s , %s " % (wristRTopic, wristLTopic, outputRTopic, outputLTopic))
        # Spin ROS
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
