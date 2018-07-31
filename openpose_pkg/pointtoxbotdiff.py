#!/usr/bin/env python


##########################
__author__  = "Emily-Jane Rolley-Parnell"
__email__  = "erolleyp@gmail.com"
__credits__ = ["Emily-Jane Rolley-Parnell"]
__version__ = "1.0.0"
__date__ = "31/08/18"
__usage__ = "CURRENT"

# Executable code that republishes the output to Cartesian Interface based on the difference from the initial point at the point that the code runs.
# Input = 
#optoros outputs for left and right wrist and hand
# /openpose/right_arm
# /openpose/left_arm

#Output = 
#Cartesian interface feedback of state
# /xbotcore/cartesian/arm1_8/state
# /xbotcore/cartesian/arm2_8/state

#Cartesian interface reference
# /xbotcore/cartesian/arm1_8/reference
# /xbotcore/cartesian/arm2_8/reference
##########################
  
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, PoseArray


def startPos(poseCurrent,initPose, count):
    #For taking and average of the first 5 positions of human
    if count == 5:
        initPose.position.x = initPose.position.x/count
        initPose.position.y = initPose.position.y/count
        initPose.position.z = initPose.position.z/count
        rospy.loginfo(initPose.position)
        return  initPose
    else:
        initPose.position.x = initPose.position.x + poseCurrent.position.x
        initPose.position.y = initPose.position.y + poseCurrent.position.y
        initPose.position.z = initPose.position.z + poseCurrent.position.z
        return initPose

def diffCalc(poseCurrent, side):
    global posePreL, human_init_L, idxL, robotStateL, posePreR, human_init_R, idxR, robotStateR
    #In this case poseDiff is the vector between init and current
    poseDiff = Pose()

    if side == "left" :
        poseDiff.position.x = poseCurrent.position.x - posePreL.position.x
        poseDiff.position.y = poseCurrent.position.y - posePreL.position.y
        poseDiff.position.z = poseCurrent.position.z - posePreL.position.z

        dist = math.sqrt((poseDiff.position.x*poseDiff.position.x) + (poseDiff.position.y*poseDiff.position.y) + (poseDiff.position.z*poseDiff.position.z))
        depth = abs(poseCurrent.position.x - posePreL.position.x)
   
        #If the distance is too small, stay and add to cumulation
        if (0.01 <= dist <= 0.17):
            posePreL = poseCurrent
            idxL = 0
            poseDiff.position.x = poseCurrent.position.x - human_init_L.position.x
            poseDiff.position.y = poseCurrent.position.y - human_init_L.position.y
            poseDiff.position.z = poseCurrent.position.z - human_init_L.position.z
        elif depth > 0.3 :
            poseDiff.position.x = posePreL.position.x - human_init_L.position.x
            poseDiff.position.y = posePreL.position.y - human_init_L.position.y
            poseDiff.position.z = posePreL.position.z - human_init_L.position.z
        else:
            rospy.loginfo('Joint lost')
            poseDiff.position.x = posePreL.position.x - human_init_L.position.x
            poseDiff.position.y = posePreL.position.y - human_init_L.position.y
            poseDiff.position.z = posePreL.position.z - human_init_L.position.z
            #idxL += 1
            if idxL == 2:
                posePreL = poseCurrent
                idxL = 0

        return poseDiff


    if side == "right" :
        poseDiff.position.x = poseCurrent.position.x - posePreR.position.x
        poseDiff.position.y = poseCurrent.position.y - posePreR.position.y
        poseDiff.position.z = poseCurrent.position.z - posePreR.position.z

        dist = math.sqrt((poseDiff.position.x*poseDiff.position.x) + (poseDiff.position.y*poseDiff.position.y) + (poseDiff.position.z*poseDiff.position.z))
        depth = abs(poseCurrent.position.x - posePreR.position.x)
   
        #If the distance is too small, stay and add to cumulation
        if (0.01 <= dist <= 0.17):
            posePreR = poseCurrent
            idxR = 0
            poseDiff.position.x = poseCurrent.position.x - human_init_R.position.x
            poseDiff.position.y = poseCurrent.position.y - human_init_R.position.y
            poseDiff.position.z = poseCurrent.position.z - human_init_R.position.z
        elif depth > 0.3 :
            poseDiff.position.x = posePreR.position.x - human_init_R.position.x
            poseDiff.position.y = posePreR.position.y - human_init_R.position.y
            poseDiff.position.z = posePreR.position.z - human_init_R.position.z
        else:
            rospy.loginfo('Joint lost')
            poseDiff.position.x = posePreR.position.x - human_init_R.position.x
            poseDiff.position.y = posePreR.position.y - human_init_R.position.y
            poseDiff.position.z = posePreR.position.z - human_init_R.position.z
            #idxR += 1
            if idxR == 2:
                posePreR = poseCurrent
                idxR = 0

        return poseDiff
    

def updateGoal(state, poseDiff, side):
    global robot_init_L, robot_init_R
    goal = PoseStamped()
    if side == "left" :
        goal.pose.position.x = robot_init_L.pose.position.x + poseDiff.position.x
        goal.pose.position.y = robot_init_L.pose.position.y + poseDiff.position.y
        goal.pose.position.z = robot_init_L.pose.position.z + poseDiff.position.z
        goal.pose.orientation.x = state.pose.orientation.x
        goal.pose.orientation.y = state.pose.orientation.y
        goal.pose.orientation.z = state.pose.orientation.z
        goal.pose.orientation.w = state.pose.orientation.w
    
    elif side == "right" :
        goal.pose.position.x = robot_init_R.pose.position.x + poseDiff.position.x
        goal.pose.position.y = robot_init_R.pose.position.y + poseDiff.position.y
        goal.pose.position.z = robot_init_R.pose.position.z + poseDiff.position.z
        goal.pose.orientation.x = state.pose.orientation.x
        goal.pose.orientation.y = state.pose.orientation.y
        goal.pose.orientation.z = state.pose.orientation.z
        goal.pose.orientation.w = state.pose.orientation.w
    return goal

def stateLCallback(msg):
    global robotStateL, robot_init_L
    robotStateL = msg
    if first_pub_L == 1:
        robot_init_L = msg

def stateRCallback(msg):
    global robotStateR, robot_init_R
    robotStateR = msg
    if first_pub_R == 1:
        robot_init_R = msg

def poseCorrect(pose):
    temp_x = pose.position.x
    temp_y = pose.position.y
    temp_z = pose.position.z
    pose.position.x = -(temp_z)
    pose.position.y = temp_x
    pose.position.z = -(temp_y)
    return pose

def newLeftMessageReceived(msg):
    global countL, posePreL, human_init_L, robotStateL, robot_init_L, first_pub_L

    # Define the goal pose to reach with the robot
    goal = PoseStamped()
    poseCur = poseCorrect(msg)
    poseD = Pose()

    if countL < 6 :

        rospy.loginfo("Init LEFT number {}".format(countL))
        if poseCur.position.x != 0 or poseCur.position.y != 0 or poseCur.position.z != 0:
            human_init_L = startPos(poseCur, human_init_L, countL)
            countL += 1
    else:
        # Transfrom poses to the world_odom (fixed in the world) frame
        try:
            # calculate the distance of the human in the moving robot world frame

            if first_pub_L == 1 :
                posePreL = human_init_L
                first_pub_L = 0

            #calculate difference
            poseD = diffCalc(poseCur, "left")
            goal = updateGoal(robotStateL, poseD, "left")
            goal.header.frame_id = 'world'
            
            # Pub the difference
            pubL.publish(goal) # pub [reference]            

            # Info
            rospy.loginfo("Robot LEFT world:" + str(goal.pose.position))


        except (rospy.ROSInterruptException):
            rospy.logwarn('ROS Interrupted')

def newRightMessageReceived(msg):
    global countR, posePreR, human_init_R, robotStateR, robot_init_R, first_pub_R

    # Define the goal pose to reach with the robot
    goal = PoseStamped()
    poseCur = poseCorrect(msg)
    poseD = Pose()

    if countR < 6 :

        rospy.loginfo("Init RIGHT number {}".format(countR))
        if poseCur.position.x != 0 or poseCur.position.y != 0 or poseCur.position.z != 0:
            human_init_R = startPos(poseCur, human_init_R, countR)
            countR += 1
    else:
        # Transfrom poses to the world_odom (fixed in the world) frame
        try:
            # calculate the distance of the human in the moving robot world frame

            if first_pub_R== 1 :
                posePreR = human_init_R
                first_pub_R= 0

            #calculate difference
            poseD = diffCalc(poseCur, "right")
            goal = updateGoal(robotStateR, poseD, "right")
            goal.header.frame_id = 'world'
            
            # Pub the difference
            pubR.publish(goal) # pub [reference]            

            # Info
            rospy.loginfo("Robot RIGHT world:" + str(goal.pose.position))


        except (rospy.ROSInterruptException):
            rospy.logwarn('ROS Interrupted')

if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("KeyToXbot")
       
        # Create subscriber [Right Wrist location]
        wristLeftTopic = rospy.resolve_name("/openpose/left_arm")
        subLW = rospy.Subscriber(wristLeftTopic, Pose, newLeftMessageReceived, queue_size=5)

        # Create subscriber [Robot right arm current]
        currentLState = rospy.resolve_name('/xbotcore/cartesian/arm1_8/state')
        stateLeftSub = rospy.Subscriber(currentLState, PoseStamped, stateLCallback, queue_size=5)      

        # Create publisher for robot
        outputLeftTopic = rospy.resolve_name('/xbotcore/cartesian/arm1_8/reference')    
        pubL = rospy.Publisher(outputLeftTopic, PoseStamped, queue_size=5)

        wristRightTopic = rospy.resolve_name("/openpose/right_arm")
        subRW = rospy.Subscriber(wristRightTopic, Pose, newRightMessageReceived, queue_size=5)

        # Create subscriber [Robot right arm current]
        currentRState = rospy.resolve_name('/xbotcore/cartesian/arm2_8/state')
        stateRightSub = rospy.Subscriber(currentRState, PoseStamped, stateRCallback, queue_size=5)      
        
        # Create publisher for robot
        outputRightTopic = rospy.resolve_name('/xbotcore/cartesian/arm2_8/reference')    
        pubR = rospy.Publisher(outputRightTopic, PoseStamped, queue_size=5)

       
        #LEFT
        # Global variables
        posePreL = Pose()
        countL = 0
        idxL = 0
        first_pub_L = 1
        # Average starting position
        human_init_L = Pose()
        robot_init_L = PoseStamped()
        robotStateL = PoseStamped()
        #RIGHT
        # Global variables
        posePreR = Pose()
        countR = 0
        idxR = 0
        first_pub_R = 1
        # Average starting position
        human_init_R = Pose()
        robot_init_R = PoseStamped()
        robotStateR = PoseStamped()
        # Spin ROS
        rospy.spin()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
