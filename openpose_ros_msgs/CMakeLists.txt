cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
	std_msgs
	message_generation
        geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PersonDetection.msg
   Persons.msg


)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## Catkin Specific Configuration ##
###################################
catkin_package(CATKIN_DEPENDS 
					message_generation
                                        message_runtime 
					std_msgs
                                        geometry_msgs)
