# TelePose

The final ROS Wrapper for CMU Openpose branch  f49e18421da832ae441f75477035786126357401  (31/07/18) Using the input of an RGB-D camera and Openni2 to publish the cartesian coordinates of human joints in order to be used with the Cartesian Interface module.

## Web-Page
Web-page: https://sites.google.com/view/telepose

Video: https://youtu.be/p9vv6LRn2ts

## Publications
1. Emily-Jane Rolley-Parnell, Dimitrios Kanoulas, Arturo Laurenzi, Brian Delhaisse, Leonel Rozo, Darwin G. Caldwell, Nikos G. Tsagarakis, *"Bi-Manual Articulated Robot Teleoperation using an External RGB-D Range Sensor"*, 15th International Conference on Control, Automation, Robotics and Vision, **ICARCV 2018**. [.pdf](https://www.google.com/url?q=https%3A%2F%2Fwww.researchgate.net%2Fpublication%2F327515599_Bi-Manual_Articulated_Robot_Teleoperation_using_an_External_RGB-D_Range_Sensor&sa=D&sntz=1&usg=AFQjCNHjM7b1IPhZ1OA56fh57qTjA3ECJg)

## Installing Original OpenPose
``` 
$ mkdir catkin_ws 
$ cd catkin_ws 
$ mkdir src 
$ cd src 
$ git clone https://github.com/dkanou/telepose.git 
$ cd telepose/openpose
```

* Complete this installation https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md
* It is described below:

    ```$ git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
    $ git checkout f49e18421da832ae441f75477035786126357401 
    $ mkdir build 
    $ cd build 
    $ ccmake .. 
Turn on flags for DOWNLOAD_BODY_COCO_MODEL in the cmake GUI
```bash
$ make all -j(num of cores)
```
(If the models cannot be found, attempt a `sudo make install` in the build folder)


Final Catkin_make:

``` 
$ cd [/catkin_ws location]
$ catkin_make
$ cd .. 
$ source devel/setup.bash 
```

## Running Openpose ROS
You must add your catkin_ws/devel/setup.bash to your .bashrc in the form of:

``` 
$ source [Insert the location of your workspace]/catkin_ws/devel/setup.bash 
```

To run the code run: 

```bash
$ roslaunch openni2_launch openni2.launch
```

(in the CMU OpenPose folder) 
```bash 
$ rosrun openpose_pkg keypoints3d
```

This will bring up the video output with the assigned Pose superimposed.

![Full Body OpenPose](/images/fullbody.png)

## Executables

Each of the executables can by run with the `python [file.py]` command. Here is a brief description of the available executables.

###### In the openpose_pkgs file:

- optoroshand.py - Converts the output of keypoints3d in to Pose ROS messages for the wrist and PoseArray for the right and left hand keypoints of person 0.
- pointtoxbotdiff.py - Filters and provides ROS PoseStamped messages for both left and right hands to Cartesian Interface.

###### In the openpose_pkgs/progress_code file:
- optoroselbow.py - Converts the output of keypoints3d in to Pose ROS messages for the wrists and elbows and PoseArray for the right and left hand keypoints of person 0.
- optoros.py - Converts the output of keypoints3d in to Pose ROS messages and publishes the wrist joint for the right and left wrists of person 0. 

- pointtoxbot2hand.py - Publishes the difference between keypoints over time and publishes PoseStamped messages for the Cartesian Interface node of both hands of person 0.

- pointtoxbothand.py - [WORK IN PROGRESS] - Publishes the PoseStamped messages for the Cartesian Interface node of the right hand of person 0, and uses the finger keypoints to create quaternions to give (ONLY) orientation of hand/end effector.
- pointtoxbot.py - Publishes the difference between keypoints over time and publishes PoseStamped messages for the Cartesian Interface node of the right hand of person 0.



## To Update OpenPose

Change to the CMU OpenPose folder:
```bash
$ git pull origin master 
$ rm -rf build 
$ mkdir build 
$ cd build 
$ cmake .. 
$ make all -j(num of cores) 
```

## Subscribing to topics

The current topic to be published has been selected as /openpose

As the custom messages have been written as vectors, the data can be accessed by specifying the point in the vector you would like to access with square brackets [].

e.g.

```bash
$ rostopic echo /openpose/person[0] 
```

Will print the custom message containing body_part , left_hand_part , right_hand_part and face_part, depending on what flags are turned on, for person 0.

```bash 
$ rostopic echo /openpose/person[1]/body_part[2]/pose_stamped 
```

Will print the PoseStamped message for person 1 for the keypoints of their left sholder as specified in the OpenPose output format [output.md](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md).

## Using Flags
If you would like to adjust the flags within Openpose, you must edit the flags within openpose_pkg/keypoints3d.cpp and follow the examples as you would if it was the openpose.bin and catkin_make each time.

**Useful Flags**:

- hand
- face
- camera_topic
- pointcloud_topic
- publish_topic
- num_gpu
- num_gpu_start

## DEPENDENCIES
- glog
- gflags
