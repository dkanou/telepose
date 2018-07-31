#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Emily Rolley-Parnell (erolleyp@gmail.com)
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Frames: 
# -- ci/world_odom is the fixed frame in the world
# -- world = odom = base_footprint are moving with the robot and are in between
#    the robot's feet
  
import rospy
import math
<<<<<<< HEAD
=======
import geometry_msgs.msg as geomsg
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
from geometry_msgs.msg import PoseStamped, Pose, PoseArray


def startPos(poseCurrent,initPose, count):
    #For taking and average of the first 5 positions of human
    if count == 5:
<<<<<<< HEAD
        initPose.position.x = initPose.position.x/count
        initPose.position.y = initPose.position.y/count
        initPose.position.z = initPose.position.z/count
        return  initPose
    else:
        initPose.position.x = initPose.position.x + poseCurrent.position.x
        initPose.position.y = initPose.position.y + poseCurrent.position.y
        initPose.position.z = initPose.position.z + poseCurrent.position.z
=======
        initPose.pose.position.x = initPose.pose.position.x/count
        initPose.pose.position.y = initPose.pose.position.y/count
        initPose.pose.position.z = initPose.pose.position.z/count
        return  initPose
    else:
        initPose.pose.position.x = initPose.pose.position.x + poseCurrent.pose.position.x
        initPose.pose.position.y = initPose.pose.position.y + poseCurrent.pose.position.y
        initPose.pose.position.z = initPose.pose.position.z + poseCurrent.pose.position.z
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
        return initPose

def diffCalc(posePrev, poseCurrent):
    global posePre, human_init, idx
<<<<<<< HEAD
    poseDiff = Pose()
    poseDiff.position.x = poseCurrent.position.x - posePrev.position.x
    poseDiff.position.y = poseCurrent.position.y - posePrev.position.y
    poseDiff.position.z = poseCurrent.position.z - posePrev.position.z

    dist = math.sqrt((poseDiff.position.x*poseDiff.position.x) + (poseDiff.position.y*poseDiff.position.y) + (poseDiff.position.z*poseDiff.position.z))
    rospy.loginfo("Euclidian Distance: " + str(dist))
    #If the distance is too small, stay and add to cumulation
    if 0.01 <= dist <= 0.55:
        posePre = poseCurrent
        idx = 0
    else:
        rospy.loginfo('Joint lost')
        poseDiff.position.x = 0
        poseDiff.position.y = 0
        poseDiff.position.z = 0
        idx += 1
        if idx >= 2:
=======
    poseDiff = PoseStamped()
    poseDiff.pose.position.x = poseCurrent.pose.position.x - posePrev.pose.position.x
    poseDiff.pose.position.y = poseCurrent.pose.position.y - posePrev.pose.position.y
    poseDiff.pose.position.z = poseCurrent.pose.position.z - posePrev.pose.position.z

    dist = math.sqrt((poseDiff.pose.position.x*poseDiff.pose.position.x) + (poseDiff.pose.position.y*poseDiff.pose.position.y) + (poseDiff.pose.position.z*poseDiff.pose.position.z))
    rospy.loginfo("Euclidian Distance: " + str(dist))
    #If the distance is too small, stay and add to cumulation
    if 0.03 <= dist <= 0.5:
        posePre = poseCurrent
    else:
        rospy.loginfo('Joint lost')
        poseDiff.pose.position.x = 0
        poseDiff.pose.position.y = 0
        poseDiff.pose.position.z = 0
        idx += 1
        if idx >= 7:
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
            rospy.loginfo("Too Far, resetting")
            posePre = poseCurrent
            idx = 0

    
    return poseDiff

def updateGoal(state, poseDiff):
    goal = PoseStamped()
<<<<<<< HEAD
    goal.pose.position.x = state.pose.position.x + poseDiff.position.x
    goal.pose.position.y = state.pose.position.y + poseDiff.position.y
    goal.pose.position.z = state.pose.position.z + poseDiff.position.z
=======
    goal.pose.position.x = state.pose.position.x + poseDiff.pose.position.x
    goal.pose.position.y = state.pose.position.y + poseDiff.pose.position.y
    goal.pose.position.z = state.pose.position.z + poseDiff.pose.position.z
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
    goal.pose.orientation.x = state.pose.orientation.x
    goal.pose.orientation.y = state.pose.orientation.y
    goal.pose.orientation.z = state.pose.orientation.z
    goal.pose.orientation.w = state.pose.orientation.w
    return goal

def stateCallback(msg):
    global robotState, robot_init
    robotState = msg
    if first_pub == 1:
        robot_init = msg


def newMessageReceived(msg):
    global count, first_pub, posePre, human_init, listener, robotState
    rospy.loginfo("Message CallBack")

    # Define the goal pose to reach with the robot
    goal = PoseStamped()
    poseCur = msg
<<<<<<< HEAD
    temp_x = poseCur.position.x
    temp_y = poseCur.position.y
    temp_z = poseCur.position.z
    poseCur.position.x = -(temp_z)
    poseCur.position.y = temp_x
    poseCur.position.z = -(temp_y)
    poseD = Pose()
=======
    temp_x = poseCur.pose.position.x
    temp_y = poseCur.pose.position.y
    temp_z = poseCur.pose.position.z
    poseCur.pose.position.x = -(temp_z)
    poseCur.pose.position.y = temp_x
    poseCur.pose.position.z = -(temp_y)
    poseD = PoseStamped()
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3

    if count < 6 :

        human_init = startPos(poseCur, human_init, count)
        rospy.loginfo("Init number {}".format(count))
<<<<<<< HEAD
        if poseCur.position.x != 0 or poseCur.position.y != 0 or poseCur.position.z != 0:
=======
        if poseCur.pose.position.x != 0 or poseCur.pose.position.y != 0 or poseCur.pose.position.z != 0:
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
            count += 1
    else:
        # Transfrom poses to the world_odom (fixed in the world) frame
        try:
            # calculate the distance of the human in the moving robot world frame
<<<<<<< HEAD
            if first_pub == 0 :

                #calculate difference
                rospy.loginfo("Pose current: " + str(poseCur.position))
                rospy.loginfo("Previous Pose: " + str(posePre.position))
                poseD = diffCalc(posePre, poseCur)
                rospy.loginfo("Pose Difference: " + str(poseD.position))
=======

            
            poseD.header.stamp = rospy.Time(0)
            if first_pub == 0 :

                #calculate difference
                rospy.loginfo("Pose current: " + str(poseCur.pose.position))
                rospy.loginfo("Previous Pose: " + str(posePre.pose.position))
                poseD = diffCalc(posePre, poseCur)
                rospy.loginfo("Pose Difference: " + str(poseD.pose.position))
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3

            if first_pub == 1 :
          
                #calculate difference
<<<<<<< HEAD
                rospy.loginfo("Pose current (first):" + str(poseCur.position))
                rospy.loginfo("Pose Initial (first):" + str(human_init.position))
                poseD = diffCalc(human_init, poseCur)
                rospy.loginfo("Pose Difference (first):" + str(poseD.position))
=======
                rospy.loginfo("Pose current (first):" + str(poseCur.pose.position))
                rospy.loginfo("Pose Initial (first):" + str(human_init.pose.position))
                poseD = diffCalc(human_init, poseCur)
                rospy.loginfo("Pose Difference (first):" + str(poseD.pose.position))
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
                # not first time to run the loop
                first_pub = 0

            goal = updateGoal(robotState, poseD)
            goal.header.frame_id = 'world'
             
<<<<<<< HEAD
            dist = math.sqrt((poseD.position.x*poseD.position.x) + (poseD.position.y*poseD.position.y) + (poseD.position.z*poseD.position.z))    
=======
            #goal.pose.orientation.x = 0.0842139378405
            #goal.pose.orientation.y = -0.93840858675
            #goal.pose.orientation.z = 0.2382381388
            #goal.pose.orientation.w = 0.23566910321

            dist = math.sqrt((poseD.pose.position.x*poseD.pose.position.x) + (poseD.pose.position.y*poseD.pose.position.y) + (poseD.pose.position.z*poseD.pose.position.z))    
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
            
            # Pub the difference
            pub.publish(goal) # pub [reference]            

            # Info
            rospy.loginfo("Robot world:" + str(goal.pose.position))
            rospy.loginfo ("Distance Moved: " + str(dist))

        except (rospy.ROSInterruptException):
<<<<<<< HEAD
            rospy.logwarn('ROS Interrupted')
=======
            rospy.logwarn('ROS Interrupted'.format(poseD.header.frame_id))
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3

if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("KeyToXbot")
       
<<<<<<< HEAD
        # Create subscriber [Right Wrist location]
        wristTopic = rospy.resolve_name("/openpose/left_arm")
        subW = rospy.Subscriber(wristTopic, Pose, newMessageReceived, queue_size=5)

        # Create subscriber [Robot right arm current]
        currentState = rospy.resolve_name('/xbotcore/cartesian/arm1_7/state')
        stateSub = rospy.Subscriber(currentState, PoseStamped, stateCallback, queue_size=5)      
        

        # Create publisher for robot
        outputTopic = rospy.resolve_name('/xbotcore/cartesian/arm1_7/reference')    
        pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=5)

        rospy.loginfo("Pub the human pose in camera frame: %s to robot cartesian output: %s" % (wristTopic, outputTopic))
    
        # Global variables
        posePre = Pose()
=======
        # Create publisher and subscriber: [reference]
        inputTopic = rospy.resolve_name("/openpose/right_arm")
        sub = rospy.Subscriber(inputTopic, PoseStamped, newMessageReceived, queue_size=5)
        currentState = rospy.resolve_name('/xbotcore/cartesian/arm2_7/state')
        stateSub = rospy.Subscriber(currentState, PoseStamped, stateCallback, queue_size=5)

        outputTopic = rospy.resolve_name('/xbotcore/cartesian/arm2_7/reference')    
        pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=5)

        rospy.loginfo("Pub the human pose in camera frame: %s to robot cartesian output: %s" % (inputTopic, outputTopic))
    
        # Global variables
        posePre = PoseStamped()
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
        count = 0
        idx = 0
        first_pub = 1
        # Average starting position
<<<<<<< HEAD
        human_init = Pose()
=======
        human_init = PoseStamped()
>>>>>>> 721d3cea08cacb13c732b29d4615509123cc50b3
        robot_init = PoseStamped()
        robotState = PoseStamped()

        # Spin ROS
        rospy.spin()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
