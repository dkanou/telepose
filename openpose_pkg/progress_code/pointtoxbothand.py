#!/usr/bin/env python
##########################
__author__  = "Emily-Jane Rolley-Parnell"
__email__  = "erolleyp@gmail.com"
__credits__ = ["Emily-Jane Rolley-Parnell"]
__version__ = "1.0.0"
__date__ = "31/08/18"
__usage__ = "Not Working"


# Executable code that publishes the orientation to be given to the right hand
# Input =
#optoros outputs for right wrist and hand
# /openpose/right_arm
# /openpose/right_hand
#Right hand state of robot
# /xbotcore/cartesian/arm2_7/state
 
#Output = 
#Right arm Reference
# /xbotcore/cartesian/arm2_7/reference   
##########################
  
  
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import numpy as np
from pyquaternion import Quaternion


def updateGoal(state, poseDiff):
    goal = PoseStamped()
    goal.pose.position.x = state.pose.position.x + poseDiff.position.x
    goal.pose.position.y = state.pose.position.y + poseDiff.position.y
    goal.pose.position.z = state.pose.position.z + poseDiff.position.z
    return goal

def stateCallback(msg):
    global robotState, robot_init, q_prev
    robotState = msg
    if first_pub == 1:
        robot_init = msg
        q_prev = [robotState.pose.orientation.w, robotState.pose.orientation.x, robotState.pose.orientation.y, robotState.pose.orientation.z]

def normalise(u):
    u_mag = np.sqrt(np.square(u[0]) + np.square(u[1]) + np.square(u[2]))
    if u_mag == 0:
        u_norm = [0,0,0]
    else:
        u_norm = [u[0]/u_mag , u[1]/u_mag, u[2]/u_mag]
    return u_norm    

def vectCalc (key1, key2):
    #Calculate a vector between 2 3D points
    vectorOut = []
    vectorOut.append(key1.position.x - key2.position.x)
    vectorOut.append(key1.position.y - key2.position.y)
    vectorOut.append(key1.position.z - key2.position.z)
    return vectorOut

def handQuatUpd():
    #assign an array of a quaternion to the message output
    global q_out, q_prev, count
    pose = Pose()
    
    Q1 = Quaternion(q_out)
    Q0 = Quaternion(q_prev)

    Q05 = Quaternion.slerp(Q0, Q1, 1/(2**(1+count)))
    print(Q05)

    pose.orientation.w = Q05[0]
    pose.orientation.x = Q05[1]
    pose.orientation.y = Q05[2]
    pose.orientation.z = Q05[3]

    return pose.orientation

def poseCorrect(human):
    #Convert from human/Camera frame to Robot world frame
    robot = Pose()
    temp_x = human.position.x
    temp_y = human.position.y
    temp_z = human.position.z
    robot.position.x = -(temp_z)
    robot.position.y = temp_x
    robot.position.z = -(temp_y)
    return robot

def sizeCheck(rightHandPA):
    global poseArrayPrev
    error = 0
    if len(rightHandPA.poses) != 0:
        for i in usedpointslist:
            #rospy.loginfo("Number of fingers : {}".format(len(rightHandPA.poses)))
            if rightHandPA.poses[i].position.x == 0 or  rightHandPA.poses[i].position.y == 0 or rightHandPA.poses[i].position.z == 0 :
                error += 1

        if error > 3:
            boolean = 0
            rospy.loginfo("Too many missing, not updating")
        else:

            boolean = 1
    else:
        boolean = 0
    print error
    return boolean

def weighted_average(val1, val2, weight):
    return (val1*weight + val2*(1-weight))

def filtering(handPA, number1, number2, number3):
    global handStoreArray
    maximum = 0.0735
    minimum = 0.01

    point1 = poseCorrect(handPA.poses[number1])
    point2 = poseCorrect(handPA.poses[number2])
    point3 = poseCorrect(handPA.poses[number3])

    #Set the maximum depth between the hands as the maximum
    #Set the minimum value as the width of a finger
    
    if len(handStoreArray) > 5:
       
        xAverage = [0,0,0]
        yAverage = [0,0,0]
        zAverage = [0,0,0]
        for l,j in enumerate([number1,number2,number3]):
            for i in range(len(handStoreArray)):
                xAverage[l] += handStoreArray[i].poses[j].position.x
                yAverage[l] += handStoreArray[i].poses[j].position.y
                zAverage[l] += handStoreArray[i].poses[j].position.z
            xAverage[l] = xAverage[l] / len(handStoreArray)
            yAverage[l] = yAverage[l] / len(handStoreArray)
            zAverage[l] = zAverage[l] / len(handStoreArray)

        #if points are 0
        if point1.position.x == 0:
            point1.position.x = xAverage[0]
            point1.position.y = yAverage[0]
            point1.position.z = zAverage[0]

        if point2.position.x == 0:
            point2.position.x = xAverage[1]
            point2.position.y = yAverage[1]
            point2.position.z = zAverage[1]

        if point3.position.x == 0:        
            point3.position.x = xAverage[2]
            point3.position.y = yAverage[2]
            point3.position.z = zAverage[2]


        #find average of previous

        weight = 0.7
        point1.position.x = weighted_average(point1.position.x, xAverage[0], weight)
        point1.position.y = weighted_average(point1.position.y, yAverage[0], weight)
        point1.position.z = weighted_average(point1.position.z, zAverage[0], weight)

        point2.position.x = weighted_average(point2.position.x, xAverage[1], weight)
        point2.position.y = weighted_average(point2.position.y, yAverage[1], weight)
        point2.position.z = weighted_average(point2.position.z, zAverage[1], weight)

        point3.position.x = weighted_average(point3.position.x, xAverage[2], weight)
        point3.position.y = weighted_average(point3.position.y, yAverage[2], weight)
        point3.position.z = weighted_average(point3.position.z, zAverage[2], weight)

        handStoreArray.pop(0)
        handStoreArray.append(handPA)
        return point1, point2, point3

    else:
        handStoreArray.append(handPA)
        return point1, point2, point3


def initState():
    #initialise the values of robot state to home position for right hand
    global robotState
    robotState.pose.position.x = 0.524205398657
    robotState.pose.position.y = -0.189681723327
    robotState.pose.position.z = 1.213720707
    robotState.pose.orientation.w = 0.0842139378405
    robotState.pose.orientation.x = -0.93840858675
    robotState.pose.orientation.y = 0.2382381388
    robotState.pose.orientation.z = 0.23566910321

def surfaceNormalise(p1, p2, p3):
    V = vectCalc(p2, p1)
    W = vectCalc(p3, p1)
    N = np.cross(V,W)
    A = normalise(N)
    return A

def midpoint4(handPA, number1, number2, number3, number4):

    point1 = poseCorrect(handPA.poses[number1])
    point2 = poseCorrect(handPA.poses[number2])
    point3 = poseCorrect(handPA.poses[number3])
    point4 = poseCorrect(handPA.poses[number4])
    Px = point1.position.x + point2.position.x + point3.position.x + point4.position.x
    Py = point1.position.y + point2.position.y + point3.position.y + point4.position.y
    Pz = point1.position.z + point2.position.z + point3.position.z + point4.position.z
    return [Px/4, Py/4, Pz/4]

def midpoint3(handPA, number1, number2, number3):

    point1 = poseCorrect(handPA.poses[number1])
    point2 = poseCorrect(handPA.poses[number2])
    point3 = poseCorrect(handPA.poses[number3])

    Px = point1.position.x + point2.position.x + point3.position.x
    Py = point1.position.y + point2.position.y + point3.position.y
    Pz = point1.position.z + point2.position.z + point3.position.z
    return [Px/3, Py/3, Pz/3]
def pose2vector(pose):
    vector = [0,0,0]
    vector[0] = pose.position.x
    vector[1] = pose.position.y
    vector[2] = pose.position.z
    return vector
def triangleVectorCross(handPA, number1, number2, number3):

    #point1 = poseCorrect(handPA.poses[number1])
    #point2 = poseCorrect(handPA.poses[number2])
    #point3 = poseCorrect(handPA.poses[number3])
    point1, point2, point3 = filtering(handPA, number1, number2, number3)
    vectorCross = surfaceNormalise(point1, point2, point3)
    return vectorCross

def fourvectoraverage(vector1, vector2, vector3, vector4):
    vout = [0,0,0]
    vout[0] = (vector1[0] + vector2[0] + vector3[0] + vector4[0])/4
    vout[1] = (vector1[1] + vector2[1] + vector3[1] + vector4[1])/4
    vout[2] = (vector1[2] + vector2[2] + vector3[2] + vector4[2])/4

    return vout

def Rot_matrix(thx, thy, thz):
    Rot_Xl = [[1,0,0],
             [0, np.cos(thx),-(np.sin(thx))],
             [0, np.sin(thx), np.cos(thx)]]
    Rot_Yl = [[np.cos(thy), 0, np.sin(thy)],
             [0, 1, 0],
             [-(np.sin(thy)), 0, np.cos(thy)]]
    Rot_Zl = [[np.cos(thz),-(np.sin(thz)), 0],
             [np.sin(thz), np.cos(thz), 0],
             [0, 0, 1]]
    Rot_X = np.array(Rot_Xl)
    Rot_Y = np.array(Rot_Yl)
    Rot_Z = np.array(Rot_Zl)
    Matrix_R = Rot_X * Rot_Y * Rot_Z
    return Matrix_R

def Ycomponent(Mp, Yh):
    #Finds point on cross product to mid point and then creates a vector from the cross product to the mid point of the upper part of the triangle.
    lam = (Mp[0]*Yh[0] + Mp[1]*Yh[1] + Mp[2]*Yh[2])/(Yh[0]**2 + Yh[1]**2 + Yh[2]**2)
    C = [0,0,0]
    C[0] = lam*Yh[0]
    C[1] = lam*Yh[1]
    C[2] = lam*Yh[2]
    Cn = normalise(C)
    Y = [Mp[0] - C[0], Mp[1] - C[1], Mp[2] - C[2]]
    return Y
    

def handAverage(handPA):
    #Creates 4 cross product vectors between the predefined points
    V1 = triangleVectorCross(handPA, 1, 17, 5)
    V2 = triangleVectorCross(handPA, 1, 17, 9)
    V3 = triangleVectorCross(handPA, 0, 13, 5)
    V4 = triangleVectorCross(handPA, 0, 9, 2)
    #Human Z axis
    Zm = fourvectoraverage(V1, V2, V3, V4)
    Z = [-Zm[0], -Zm[1], -Zm[2]]
    Zh = normalise(Z)

    #Human X Axis
    Mp2 = midpoint4(handPA, 5, 9, 13, 17)
    Xm = Ycomponent(Mp2, Zh) #Yh
    X = [-Xm[0], -Xm[1], -Xm[2]]
    Xh = normalise(Xm)

    #Human Z axis
    Ym = np.cross(Xh, Zh)
    Y = [-Ym[0], -Ym[1], -Ym[2]]
    Yh = normalise(Y)

    ROT = np.array([[Xh[0], Yh[0], Zh[0]],[Xh[1], Yh[1], Zh[1]],[Xh[2], Yh[2], Zh[2]]])
   
    return ROT

def handCallback(msg):

    global first_pub, q_out, robotState, q_prev, handStore, count
    
    rightHandPA = msg
    boolean = sizeCheck(rightHandPA)

    if boolean == 1 :

        if first_pub == 1:

            q_out = [robotState.pose.orientation.w, robotState.pose.orientation.x, robotState.pose.orientation.y, robotState.pose.orientation.z]
            first_pub = 0
            q_prev = q_out
        else:
            if count == 5:
                Rotation = handAverage(rightHandPA)
                q_out = Quaternion(matrix=Rotation)
                q_prev = q_out
                handStore = rightHandPA
                count = 0
            else:

                count += 1


    if boolean == 0:
        q_out = q_prev
        rospy.loginfo("Quaternion: " + str(q_out))


def newMessageReceived(msg):
    global first_pub, robotState
    rospy.loginfo("Message CallBack")
    poseD = Pose()
    poseD.position.x = 0
    poseD.position.y = 0
    poseD.position.z = 0 
    # Define the goal pose to reach with the robot
    goal = PoseStamped()

    goal = updateGoal(robotState, poseD)
    goal.header.frame_id = 'world'

    goal.pose.orientation = handQuatUpd()
        
    # Pub the difference
    pub.publish(goal)           


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("KeyToXbot")
       
        # Create subscriber [Right Wrist location]
        wristTopic = rospy.resolve_name("/openpose/right_arm")
        subW = rospy.Subscriber(wristTopic, Pose, newMessageReceived, queue_size=5)
        
        # Create subscriber [robot right arm current state]
        currentState = rospy.resolve_name('/xbotcore/cartesian/arm2_7/state')
        stateSub = rospy.Subscriber(currentState, PoseStamped, stateCallback, queue_size=5)

        # Create subscribe [Right Hand]
        handTopic = rospy.resolve_name("/openpose/right_hand")
        subH = rospy.Subscriber(handTopic, PoseArray, handCallback, queue_size=5)
        
        # Create publisher for robot
        outputTopic = rospy.resolve_name('/xbotcore/cartesian/arm2_7/reference')    
        pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=5)

        rospy.loginfo("Pub the human pose in camera frame: %s to robot cartesian output: %s" % (wristTopic, outputTopic))
    
        # Global variables
        usedpointslist = [1,2,3,5,9,13,17]
        idx = 0
        first_pub = 1
        count = 0

        robot_init = PoseStamped()
        robotState = PoseStamped()
        handStoreArray = []
        q_held = Quaternion()
        initState()
        q_out = [robotState.pose.orientation.w, robotState.pose.orientation.x, robotState.pose.orientation.y, robotState.pose.orientation.z]
        
        # Spin ROS
        rospy.spin()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

