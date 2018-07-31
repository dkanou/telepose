/*
*  Implementation of Openpose where the input has been replaced by a ROS 
*  sensor_msgs::ImageConstPtr topic and sensor_msgs::PointCloud2 as 
*  specified by the user. The output can print keypoints to terminal or it 
*  can publish the keypoints as combination of custom ROS messages and 
*  PoseStamped messages.
*  \author Emily Rolley-Parnell
*/

//---------------ROS Includes ---------------//

//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/transforms.h>

#include <openpose/headers.hpp>

#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/init.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <openpose_ros_msgs/PersonDetection.h>
#include <openpose_ros_msgs/Persons.h>

#include <image_transport/image_transport.h>
//#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread
#include <math.h>
// Other 3rdparty dependencies
// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include <gflags/gflags.h>
// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <openpose/pose/poseParameters.hpp>

// See all the available parameter options withe the `--help` flag. E.g. `build/examples/openpose/openpose.bin --help`
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging/Other
DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
DEFINE_bool(disable_multi_thread,       false,          "It would slightly reduce the frame rate in order to highly reduce the lag. Mainly useful"
                                                        " for 1) Cases where it is needed a low latency (e.g. webcam in real-time scenarios with"
                                                        " low-range GPU devices); and 2) Debugging OpenPose when it is crashing to locate the"
                                                        " error.");
DEFINE_int32(profile_speed,             1000,           "If PROFILER_ENABLED was set in CMake or Makefile.config files, OpenPose will show some"
                                                        " runtime statistics at this frame number.");
// Producer
DEFINE_int32(camera,                    0,             "The camera index for cv::VideoCapture. Integer in the range [0, 9]. Select a negative"
                                                        " number (by default), to auto-detect and open the first available camera.");
DEFINE_string(camera_resolution,        "640x480",        "Set the camera resolution (either `--camera` or `--flir_camera`). `-1x-1` will use the"
                                                        " default 1280x720 for `--camera`, or the maximum flir camera resolution available for"
                                                        " `--flir_camera`");
DEFINE_double(camera_fps,               30.0,           "Frame rate for the webcam (also used when saving video). Set this value to the minimum"
                                                        " value between the OpenPose displayed speed and the webcam real frame rate.");
DEFINE_string(video,                    "",             "Use a video file instead of the camera. Use `examples/media/video.avi` for our default"
                                                        " example video.");
DEFINE_string(image_dir,                "examples/media/",   "Process a directory of images. Use `examples/media/` for our default"
                                                        " example folder with 20 images. Read all standard formats (jpg, png, bmp, etc.).");
DEFINE_bool(flir_camera,                false,          "Whether to use FLIR (Point-Grey) stereo camera.");
DEFINE_string(ip_camera,                "",             "String with the IP camera URL. It supports protocols like RTSP and HTTP.");
DEFINE_uint64(frame_first,              0,              "Start on desired frame number. Indexes are 0-based, i.e. the first frame has index 0.");
DEFINE_uint64(frame_last,               -1,             "Finish on desired frame number. Select -1 to disable. Indexes are 0-based, e.g. if set to"
                                                        " 10, it will process 11 frames (0-10).");
DEFINE_bool(frame_flip,                 false,          "Flip/mirror each frame (e.g. for real time webcam demonstrations).");
DEFINE_int32(frame_rotate,              0,              "Rotate each frame, 4 possible values: 0, 90, 180, 270.");
DEFINE_bool(frames_repeat,              false,          "Repeat frames when finished.");
DEFINE_bool(process_real_time,          false,          "Enable to keep the original source frame rate (e.g. for video). If the processing time is"
                                                        " too long, it will skip frames. If it is too fast, it will slow it down.");
DEFINE_string(camera_parameter_folder,  "models/cameraParameters/flir/", "String with the folder where the camera parameters are located.");
// OpenPose
DEFINE_string(model_folder,             "models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu,                   1,             "The number of GPU devices to use. If negative, it will use all the available GPUs in your"
                                                        " machine.");
DEFINE_int32(num_gpu_start,             1,              "GPU device start number.");
DEFINE_int32(keypoint_scale,            0,              "Scaling of the (x,y) coordinates of the final pose data array, i.e. the scale of the (x,y)"
                                                        " coordinates that will be saved with the `write_keypoint` & `write_keypoint_json` flags."
                                                        " Select `0` to scale it to the original source resolution, `1`to scale it to the net output"
                                                        " size (set with `net_resolution`), `2` to scale it to the final output size (set with"
                                                        " `resolution`), `3` to scale it in the range [0,1], and 4 for range [-1,1]. Non related"
                                                        " with `scale_number` and `scale_gap`.");
DEFINE_int32(number_people_max,         1,             "This parameter will limit the maximum number of people detected, by keeping the people with"
                                                        " top scores. The score is based in person area over the image, body part score, as well as"
                                                        " joint score (between each pair of connected body parts). Useful if you know the exact"
                                                        " number of people in the scene, so it can remove false positives (if all the people have"
                                                        " been detected. However, it might also include false negatives by removing very small or"
                                                        " highly occluded people. -1 will keep them all.");
// OpenPose Body Pose
DEFINE_bool(body_disable,               false,          "Disable body keypoint detection. Option only possible for faster (but less accurate) face"
                                                        " keypoint detection.");
DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(net_resolution,           "-1x368",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
// OpenPose Body Pose Heatmaps and Part Candidates
DEFINE_bool(heatmaps_add_parts,         false,          "If true, it will fill op::Datum::poseHeatMaps array with the body part heatmaps, and"
                                                        " analogously face & hand heatmaps to op::Datum::faceHeatMaps & op::Datum::handHeatMaps."
                                                        " If more than one `add_heatmaps_X` flag is enabled, it will place then in sequential"
                                                        " memory order: body parts + bkg + PAFs. It will follow the order on"
                                                        " POSE_BODY_PART_MAPPING in `src/openpose/pose/poseParameters.cpp`. Program speed will"
                                                        " considerably decrease. Not required for OpenPose, enable it only if you intend to"
                                                        " explicitly use this information later.");
DEFINE_bool(heatmaps_add_bkg,           false,          "Same functionality as `add_heatmaps_parts`, but adding the heatmap corresponding to"
                                                        " background.");
DEFINE_bool(heatmaps_add_PAFs,          false,          "Same functionality as `add_heatmaps_parts`, but adding the PAFs.");
DEFINE_int32(heatmaps_scale,            2,              "Set 0 to scale op::Datum::poseHeatMaps in the range [-1,1], 1 for [0,1]; 2 for integer"
                                                        " rounded [0,255]; and 3 for no scaling.");
DEFINE_bool(part_candidates,            false,          "Also enable `write_json` in order to save this information. If true, it will fill the"
                                                        " op::Datum::poseCandidates array with the body part candidates. Candidates refer to all"
                                                        " the detected body parts, before being assembled into people. Note that the number of"
                                                        " candidates is equal or higher than the number of final body parts (i.e. after being"
                                                        " assembled into people). The empty body parts are filled with 0s. Program speed will"
                                                        " slightly decrease. Not required for OpenPose, enable it only if you intend to explicitly"
                                                        " use this information.");
// OpenPose Face
DEFINE_bool(face,                       false,          "Enables face keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`. Note that this will considerable slow down the performance and increse"
                                                        " the required GPU memory. In addition, the greater number of people on the image, the"
                                                        " slower OpenPose will be.");
DEFINE_string(face_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the face keypoint"
                                                        " detector. 320x320 usually works fine while giving a substantial speed up when multiple"
                                                        " faces on the image.");
// OpenPose Hand
DEFINE_bool(hand,                       true,          "Enables hand keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`. Analogously to `--face`, it will also slow down the performance, increase"
                                                        " the required GPU memory and its speed depends on the number of people.");
DEFINE_string(hand_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the hand keypoint"
                                                        " detector.");
DEFINE_int32(hand_scale_number,         1,              "Analogous to `scale_number` but applied to the hand keypoint detector. Our best results"
                                                        " were found with `hand_scale_number` = 6 and `hand_scale_range` = 0.4.");
DEFINE_double(hand_scale_range,         0.4,            "Analogous purpose than `scale_gap` but applied to the hand keypoint detector. Total range"
                                                        " between smallest and biggest scale. The scales will be centered in ratio 1. E.g. if"
                                                        " scaleRange = 0.4 and scalesNumber = 2, then there will be 2 scales, 0.8 and 1.2.");
DEFINE_bool(hand_tracking,              true,          "Adding hand tracking might improve hand keypoints detection for webcam (if the frame rate"
                                                        " is high enough, i.e. >7 FPS per GPU) and video. This is not person ID tracking, it"
                                                        " simply looks for hands in positions at which hands were located in previous frames, but"
                                                        " it does not guarantee the same person ID among frames.");
// OpenPose 3-D Reconstruction
DEFINE_bool(3d,                         false,          "Running OpenPose 3-D reconstruction demo: 1) Reading from a stereo camera system."
                                                        " 2) Performing 3-D reconstruction from the multiple views. 3) Displaying 3-D reconstruction"
                                                        " results. Note that it will only display 1 person. If multiple people is present, it will"
                                                        " fail.");
DEFINE_int32(3d_min_views,              -1,             "Minimum number of views required to reconstruct each keypoint. By default (-1), it will"
                                                        " require all the cameras to see the keypoint in order to reconstruct it.");
DEFINE_int32(3d_views,                  1,              "Complementary option to `--image_dir` or `--video`. OpenPose will read as many images per"
                                                        " iteration, allowing tasks such as stereo camera processing (`--3d`). Note that"
                                                        " `--camera_parameters_folder` must be set. OpenPose must find as many `xml` files in the"
                                                        " parameter folder as this number indicates.");
// OpenPose Rendering
DEFINE_int32(part_to_show,              0,              "Prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body"
                                                        " part heat map, 19 for the background heat map, 20 for all the body part heat maps"
                                                        " together, 21 for all the PAFs, 22-40 for each body part pair PAF.");
DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
                                                        " background, instead of being rendered into the original image. Related: `part_to_show`,"
                                                        " `alpha_pose`, and `alpha_pose`.");
// OpenPose Rendering Pose
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_int32(render_pose,               -1,             "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering"
                                                        " (slower but greater functionality, e.g. `alpha_X` flags). If -1, it will pick CPU if"
                                                        " CPU_ONLY is enabled, or GPU if CUDA is enabled. If rendering is enabled, it will render"
                                                        " both `outputData` and `cvOutputData` with the original image and desired body part to be"
                                                        " shown (i.e. keypoints, heat maps or PAFs).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");
DEFINE_double(alpha_heatmap,            0.7,            "Blending factor (range 0-1) between heatmap and original frame. 1 will only show the"
                                                        " heatmap, 0 will only show the frame. Only valid for GPU rendering.");
// OpenPose Rendering Face
DEFINE_double(face_render_threshold,    0.4,            "Analogous to `render_threshold`, but applied to the face keypoints.");
DEFINE_int32(face_render,               1,             "Analogous to `render_pose` but applied to the face. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(face_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to face.");
DEFINE_double(face_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to face.");
// OpenPose Rendering Hand
DEFINE_double(hand_render_threshold,    0.2,            "Analogous to `render_threshold`, but applied to the hand keypoints.");
DEFINE_int32(hand_render,               -1,             "Analogous to `render_pose` but applied to the hand. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(hand_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to hand.");
DEFINE_double(hand_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to hand.");
// Result Saving
DEFINE_string(write_images,             "",             "Directory to write rendered frames in `write_images_format` image format.");
DEFINE_string(write_images_format,      "png",          "File extension and format for `write_images`, e.g. png, jpg or bmp. Check the OpenCV"
                                                        " function cv::imwrite for all compatible extensions.");
DEFINE_string(write_video,              "",             "Full file path to write rendered frames in motion JPEG video format. It might fail if the"
                                                        " final path does not finish in `.avi`. It internally uses cv::VideoWriter.");
DEFINE_string(write_json,               "",             "Directory to write OpenPose output in JSON format. It includes body, hand, and face pose"
                                                        " keypoints (2-D and 3-D), as well as pose candidates (if `--part_candidates` enabled).");
DEFINE_string(write_coco_json,          "",             "Full file path to write people pose data with JSON COCO validation format.");
DEFINE_string(write_heatmaps,           "",             "Directory to write body pose heatmaps in PNG format. At least 1 `add_heatmaps_X` flag"
                                                        " must be enabled.");
DEFINE_string(write_heatmaps_format,    "png",          "File extension and format for `write_heatmaps`, analogous to `write_images_format`."
                                                        " For lossless compression, recommended `png` for integer `heatmaps_scale` and `float` for"
                                                        " floating values.");
DEFINE_string(write_keypoint,           "",             "(Deprecated, use `write_json`) Directory to write the people pose keypoint data. Set format"
                                                        " with `write_keypoint_format`.");
DEFINE_string(write_keypoint_format,    "yml",          "(Deprecated, use `write_json`) File extension and format for `write_keypoint`: json, xml,"
                                                        " yaml & yml. Json not available for OpenCV < 3.0, use `write_keypoint_json` instead.");
DEFINE_string(write_keypoint_json,      "",             "(Deprecated, use `write_json`) Directory to write people pose data in JSON format,"
                                                        " compatible with any OpenCV version.");
DEFINE_string(write_coco_foot_json,     "",             "Full file path to write people foot pose data with JSON COCO validation format.");

// Result Saving - Extra Algorithms
DEFINE_string(write_video_adam,         "",             "Experimental, not available yet. E.g.: `~/Desktop/adamResult.avi`. Flag `camera_fps`"
                                                        " controls FPS.");
DEFINE_string(write_bvh,                "",             "Experimental, not available yet. E.g.: `~/Desktop/mocapResult.bvh`.");
// UDP communication
DEFINE_string(udp_host,                 "",             "Experimental, not available yet. IP for UDP communication. E.g., `192.168.0.1`.");
DEFINE_string(udp_port,                 "8051",         "Experimental, not available yet. Port number for UDP communication.");


//Display
DEFINE_int32(display,                   0,             "Display mode: -1 for automatic selection; 0 for no display (useful if there is no X server"
                                                        " and/or to slightly speed up the processing if visual output is not required); 2 for 2-D"
                                                        " display; 3 for 3-D display (if `--3d` enabled); and 1 for both 2-D and 3-D display.");

DEFINE_bool(no_gui_verbose,             false,          "Do not write text on output images on GUI (e.g. number of current frame and people). It"
                                                        " does not affect the pose rendering.");
DEFINE_bool(fullscreen,                 false,          "Run in full-screen mode (press f during runtime to toggle).");

// Extra algorithms
DEFINE_bool(identification,             false,          "Experimental, not available yet. Whether to enable people identification across frames.");
DEFINE_int32(tracking,                  -1,             "Experimental, not available yet. Whether to enable people tracking across frames. The"
                                                        " value indicates the number of frames where tracking is run between each OpenPose keypoint"
                                                        " detection. Select -1 (default) to disable it or 0 to run simultaneously OpenPose keypoint"
                                                        " detector and tracking for potentially higher accurary than only OpenPose.");
DEFINE_int32(ik_threads,                0,              "Experimental, not available yet. Whether to enable inverse kinematics (IK) from 3-D"
                                                        " keypoints to obtain 3-D joint angles. By default (0 threads), it is disabled. Increasing"
                                                        " the number of threads will increase the speed but also the global system latency.");

/*
*\brief ROS Topic flags to be specified by the user
*\param camera_topic for OpenNI2 standard RGB Camera topic is "/camera/rgb/image_raw"
*\param pointcloud_topic for OpenNI2 standard Depth PCL Camera topic is  "/camera/depth/points"
*/

//ROS Flags
DEFINE_string(camera_topic,    "/camera/rgb/image_raw", "The ROS Topic to subscribe to that provides an RGB image.");
DEFINE_string(pointcloud_topic, "/camera/depth/points", "The ROS Topic that publishes the PointCloud2");
DEFINE_string(publish_topic,   "/keypoints"           , "The ROS Topic to which the keypoint data is published");


/*
*\brief A struct in which UserDatum is defined, to hold any variables processed by OpenPose. Includes op::Datum as a child struct 
*\param dataCloud a pointer to a pcl::PointCloud that holds the unprocessed PointCloud data of each frame
*
* If the user needs their own variables, they can inherit the op::Datum struct and add them,
* UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define
* Wrapper<UserDatum> instead of Wrapper<op::Datum>
*/

struct UserDatum : public op::Datum
{
    bool boolThatUserNeedsForSomeReason;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> dataCloud {new pcl::PointCloud<pcl::PointXYZ>};    
    UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
       boolThatUserNeedsForSomeReason{boolThatUserNeedsForSomeReason_}
   {} 
};

/*
*\brief UserInputClass is a class containing the functions for processing the PointCloud data as well as the sensor_msgs::ImageConstPtr
*\param  pOpenCVImg the pointer to the opencv image, the midpoint between the ROS message and the cv::Mat
*\param  mMessage the ROS sensor message produced by the RGB camera node 
*\param  mFrame the internal variable used for the cv::Mat of the RGB image
*\param  FramePtr std::shared_ptr to mFrame
*\param  pCloudVector a std::shared_ptr to a vector of std::shared_ptrs to PointClouds of pcl::PointXYZ
*\param  pTempCloud is a pointer to a temporary PointCloud of pcl::PointXYZ points
*\param  pointSubscriber is the ROS Subscriber that subscribes to a point cloud of type pcl::PointCloud2 
*\param  nh the node handler for ROS
*\param  pclTopic the topic as was given in the FLAGS section to subscribe to for the PointCloud2 message
*/
class UserInputClass 
{
    public:
    
        ros::NodeHandle nh;
        /*
        *\brief class constructor for UserInputClass - also subscribes to PointCloud and 
        * initialises ros::NodeHandle
        */ 
        UserInputClass()    
        { 
            const std::string pclTopic = FLAGS_pointcloud_topic;
            pointSubscriber = nh.subscribe(pclTopic, 5, &UserInputClass::callbackPCL, this);
        }
        
        /*
        *\brief setMessage takes the ROS message and copies it to the internal mMessage variable
        */
        void setMessage (const sensor_msgs::ImageConstPtr msg)
        {
	    mMessage = msg;
            op::log("Setting Message", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
        } 
        
        /*
        *\brief getFrame returns a pointer to the cv::Mat that will be used by OpenPose 
        * - converts message to OpenCV then points to the cv::Mat component of this
        */
        std::shared_ptr<cv::Mat> getFrame ()
        {
            try
            {
                //Use cv_bridge to convert from sensor_msgs:ImageConstPtr to OpenCV::Image 
                pOpenCVImg = cv_bridge::toCvCopy(mMessage, sensor_msgs::image_encodings::BGR8);
                //Convert pointer to cv::Mat 
                *FramePtr = pOpenCVImg->image;
                op::log("Returning Cv Mat.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
                return FramePtr;
                            
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("getFrame exception: %s", e.what());
            }
        }
        /*
        *\brief callbackPCL called whenever a new PointCloud2 message is received on 
        *the pointcloud topic as specified in the Flags, converts from sensor_msgs::PointCloud2
        * to pcl::PointCloud2 and emplaces it on the queue to be used
        */
        void callbackPCL (const sensor_msgs::PointCloud2& cloud_msg)
        {
            //Define temporary PointCloud2
            pcl::PCLPointCloud2 pcl2Temp;
            //Convert from sensor_msgs::PointCloud2 to pcl::PointCloud
            pcl_conversions::toPCL(cloud_msg, pcl2Temp);
            //Convert from pcl::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
            pcl::fromPCLPointCloud2(pcl2Temp,*pTempCloud);
            op::log("PointCloud Callback", op::Priority::Low);
 
            try
            {
                //Add pcl::PointCloud<pcl::PointXYZ> to the queue
                pCloudVector -> emplace_back(pTempCloud);
                op::log("PointCloud2 emplaced", op::Priority::Low);
            }
            catch(std::bad_alloc)
            {
                ROS_ERROR("PointCloud2 emplace error");
            }
        
        }

        /*
        *\brief createDatum takes the RGB Image message that has been subscribed to and adds
        * it to the datumsPtr queue
        */
        std::shared_ptr<std::vector<UserDatum>> createDatum(const sensor_msgs::ImageConstPtr msg)
        {
            //Update mMessage to be msg
            setMessage(msg);
            //Process new mMessage and update FramePtr
            getFrame();
            //Update mFrame to be the latest cv::Mat frame
            mFrame = *FramePtr;
        
            bool isReceiving = true;

            //If FramePtr is null
            if (FramePtr == nullptr)
            {
                isReceiving = false;
                op::log("OpenCv Frame Pointer is NULL", op::Priority::High);
             
                return nullptr;
            }
            //Else if the queue size is 0
            else if (pCloudVector->size() == 0)
            {
                isReceiving = false;
                op::log("Waiting for PointCloud", op::Priority::High);
             
                return nullptr;
            }
            else
            {        
                // Create new datum
                auto datumsPtr = std::make_shared<std::vector<UserDatum>>();
                datumsPtr->emplace_back();
                //Create reference at bottom of the queue
                auto& datum = datumsPtr->at(0);

                // Fill datum
                datum.dataCloud = pCloudVector->at(0);
                datum.cvInputData = mFrame;
                datum.cvOutputData = mFrame;

                // If empty frame -> return nullptr
                if (datum.cvInputData.empty())
                {
                    datumsPtr = nullptr;
                }
                return datumsPtr;
            
            }   
        }
    
    private:
        sensor_msgs::ImageConstPtr mMessage;
        cv::Mat mFrame;
        cv_bridge::CvImagePtr pOpenCVImg;
        std::shared_ptr<cv::Mat> FramePtr{new cv::Mat};
        std::shared_ptr<std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>> pCloudVector {new std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>};
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pTempCloud {new pcl::PointCloud<pcl::PointXYZ>};
        ros::Subscriber pointSubscriber;


};

/*
*\brief UserOutputClass uses the std::shared_ptr to a std::vector of UserDatum after being processed by OpenPose * and provides different fucntions that display that data.
*\param datumsPtr pointer to vector of the struct UserDatum, used as the queue of processed 
* information.
*\param datum.cvOutputData OpenCV image rendered with Pose or Heatmaps
*\param datum.PoseKeypoints Array<float> containing the keypoints of the estimated pose 
*\param datum.FaceKeypoints Array<float> It is completely analogous to poseKeypoints.  
*\param datum.HandKeypoints std::array<Array, 2> handKeypoints, where handKeypoints[0] corresponds
* to the left hand and handKeypoints[1] to the right one. Each handKeypoints[i] is analogous to 
* poseKeypoints and faceKeypoints
*\param datum.dataCloud pcl::PointCloud<pcl::PointXYZ> of the data taken in by the ROS Depth
* PointCloud topic.
*/

class UserOutputClass
{
public:

    /*
    *\brief display renders map of pose for each person detected by OpenPose
    */
    bool display(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
    {
        char key = ' ';
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
            // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
            key = (char)cv::waitKey(1);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        return (key == 27);
    }
    
    /*
    *\brief (!!UNUSED!! See "toPoseStamped" function) printKeypoints takes the datumsPtr and prints to terminal all of the keypoints for Pose, 
    *Face and Hands.
    */
    void printKeypoints(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
    {
        // If datumsPtr is not null and queue is not empty
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::log("\nKeypoints:");
            // Accesing first item in queue
            const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
            op::log("Person pose keypoints:");
            //Loop for number of people detected
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
                op::log("Person " + std::to_string(person) + " (x, y, score):");
                //Loop for number of body parts
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    std::string valueToPrint;
                    //Loop through printing x, y, confidence
                    for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                        valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                    op::log(valueToPrint);
                }
            }
            op::log(" ");
            // Alternative: just getting std::string equivalent
            op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());
            op::log("Left hand keypoints: " + datumsPtr->at(0).handKeypoints[0].toString());
            op::log("Right hand keypoints: " + datumsPtr->at(0).handKeypoints[1].toString());
            // Heatmaps
            const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
            if (!poseHeatMaps.empty())
            {
                op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                        + std::to_string(poseHeatMaps.getSize(1)) + ", "
                        + std::to_string(poseHeatMaps.getSize(2)) + "]");
                const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
                op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                        + std::to_string(faceHeatMaps.getSize(1)) + ", "
                        + std::to_string(faceHeatMaps.getSize(2)) + ", "
                        + std::to_string(faceHeatMaps.getSize(3)) + "]");
                const auto& handHeatMaps = datumsPtr->at(0).handHeatMaps;
                op::log("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                        + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                        + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                        + std::to_string(handHeatMaps[0].getSize(3)) + "]");
                op::log("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                        + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                        + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                        + std::to_string(handHeatMaps[1].getSize(3)) + "]");
            }
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    }

    /*
    *\brief PixToXYZ a function that, given pixel coordinates, returns the X, Y, Z coordinates at 
    * that pixel location, performs a comparison to prevent coordinates that go out of range
    * of the PointCloud map.
    *\param resolution sting provided by  FLAGS_camera_resolution
    *\param delimiter character that separates the two resolution parameters within the flag, 
    * in this case it is "x".
    *\param sMaxU string value of the maximum resolution width
    *\param sMaxV string value of the maximum resolution height
    *\param iMaxU int of sMaxU
    *\param iMaxV int of sMaxV
    *\param mU the integer whole of the pixel coordinate u
    *\param mV the integer whole of the pixel coordinate v
    *\param mPoint the pcl::PointXYZ container
    *\param keypointsArray(3) vector of floats size 3 containing X, Y, Z catesian coordinates
    */

    std::vector<float> PixToXYZ (float u, float v, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> dataCloud)
    {
        //Collect camera resolution as specified in FLAGS
        std::string resolution = FLAGS_camera_resolution;
        std::string delimiter = "x";
        //Collect the max U value from the string resolution flag
        std::string sMaxU = resolution.substr(0, resolution.find(delimiter));
        resolution = resolution.erase(0, resolution.find(delimiter) + delimiter.length());
        //Collect the max V value from the string resolution flag
        std::string sMaxV = resolution;
        //Convert the string values to int base 10
        int iMaxU = std::stoi(sMaxU, nullptr, 10);
        int iMaxV = std::stoi(sMaxV, nullptr, 10);
        
        //Take natural part of pixel coordinates provided by openpose
        int mU = static_cast<int>(u);
        int mV = static_cast<int>(v);

        std::vector<float> keypointsArray (3);
        //If pixel Coordinates are 0, or NaN, return 0
        if (std::isnan(mU) || std::isnan(mV) || mV == 0 || mU == 0)
        {          
            keypointsArray.at(0) = 0;
            keypointsArray.at(1) = 0;
            keypointsArray.at(2) = 0;
            op::log("Keypoint not located", op::Priority::Normal);
        }
        else
        {   
            //Ensure OpenPose pixel coordinates are within resolution bounds
            if (mU >= iMaxU)
            {
                mU = iMaxU-1;
            }
            if(mV >= iMaxV)
            {
               mV = iMaxV-1;
            }
            if(mV <= 0)
            {
                mV = 1;
            }
            if(mU <= 0)
            {
                mU = 1;
            }
            pcl::PointXYZ mPoint;
            int pixels = 0;
            // Average the nearest points 3X3 square
            for (int i = (mV - 1) ; i <= (mV + 1) ; i ++)
            {
                for (int i = (mU - 1) ; i <= (mU + 1) ; i ++)
                {
                    mPoint = dataCloud->at(mU, mV);

                    if (!(std::isnan(mPoint.x) || std::isnan(mPoint.y) || std::isnan(mPoint.z)))
                    {
                        //Sum to the average
                        keypointsArray.at(0) = keypointsArray.at(0) + mPoint.x;
                        keypointsArray.at(1) = keypointsArray.at(1) + mPoint.y;
                        keypointsArray.at(2) = keypointsArray.at(2) + mPoint.z;
 
                        pixels += 1;
                    }
                }
            }
              
            op::log("Pixels averaged: " + std::to_string(pixels), op::Priority::Low);
            if (pixels != 0)
            { 
                keypointsArray.at(0) = keypointsArray.at(0)/pixels;
                keypointsArray.at(1) = keypointsArray.at(1)/pixels;
                keypointsArray.at(2) = keypointsArray.at(2)/pixels;
                         
                op::log("keypointsArray: " + std::to_string(keypointsArray.at(0)) + ", " + std::to_string(keypointsArray.at(1)) + ", " + std::to_string(keypointsArray.at(2)), op::Priority::Low);
            }
            else
            {
                keypointsArray.at(0) = 0;
                keypointsArray.at(1) = 0;
                keypointsArray.at(2) = 0;
                  
            }
        }   
        return keypointsArray;
    }
      
    /*
    *\brief toPoseStamped takes the data from datumsPtr and returns the keypoints as a custom 
    * ROS message type. 
    *\param  individual Custom ROS message made up of different keypoint type messages
    *\param  output Custom ROS message type Persons made up of PersonDetection messages
    *\param cloudPointer shared pointer to PointCloud
    *\param joint Pose message a single joint of type geometry_msgs::Pose
    *\param bodyPart a counter variable ennumerating the number of body parts
    *\param handPart a counter variable ennumerating the number of hand parts on either the 
    * left or the right hand
    *\param facePart a counter variable ennumerating the number of face parts
    */
    
    openpose_ros_msgs::Persons toPoseStamped (const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
    {

        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::log("\nPublishing Data... (Publisher topic: " + FLAGS_publish_topic + " )");
            
            //Define ROS custom message 
            openpose_ros_msgs::Persons output;
            //Create references to keypoint variables at the bottom of the queue
            const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
            const auto& handKeypoints = datumsPtr->at(0).handKeypoints;
            const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
            const auto& depthCloud = datumsPtr->at(0).dataCloud;
            //Create pointer of depth cloud
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudPointer = depthCloud;

            std::cout << "Number of People in image :" << poseKeypoints.getSize(0) << std::endl;
            //Loop for number of people
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
              
                //Define ROS custom message 
                openpose_ros_msgs::PersonDetection individual;
                //Loop for number of body parts (Check pose model - MPI/COCO)
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    //Define Pose message to be pushed to list of body parts
                    geometry_msgs::Pose joint;
                    
                    //Create floats containing pixel coordinates of joint
                    float u = poseKeypoints[{person, bodyPart, 0}];
                    float v = poseKeypoints[{person, bodyPart, 1}];
                    std::vector<float> pointXYZ (3); 
               
                    //Create vector containing XYZ of pixel coordinate    
                    pointXYZ = PixToXYZ(u, v, cloudPointer); 
                    if (bodyPart == 4){
                        op::log("Right wrist key points : " + std::to_string(pointXYZ.at(0)) + ", " + std::to_string(pointXYZ.at(1)) + ", " + std::to_string(pointXYZ.at(2)));
                        op::log("Right wrist pixel points : " + std::to_string(u) + ", " + std::to_string(v));

                    }
                    if (bodyPart == 7){
                        op::log("Left wrist key points : " + std::to_string(pointXYZ.at(0)) + ", " + std::to_string(pointXYZ.at(1)) + ", " + std::to_string(pointXYZ.at(2)));
                        op::log("Left wrist pixel points : " + std::to_string(u) + ", " + std::to_string(v));
                    }
                    //Place data in to ROS message
                    joint.position.x = pointXYZ.at(0);
                    joint.position.y = pointXYZ.at(1);
                    joint.position.z = pointXYZ.at(2);

                    //Add joint to vector of body parts
                    individual.body_part.poses.push_back(joint);
                        
                }
     
                //If hand detection is enabled
                if (FLAGS_hand){
                    //Loop for number of keypoints in left hand
                    for (auto handPart = 0 ; handPart < handKeypoints[0].getSize(1) ; handPart++)
                    {
                        //Define Pose message to be pushed to list of body parts
                        geometry_msgs::Pose joint;
        
                        //Create floats containing pixel coordinates of joint
                        float u = handKeypoints[0][{person, handPart, 0}];
                        float v = handKeypoints[0][{person, handPart, 1}];
                        std::vector<float> pointXYZ (3); 
                        //Create vector containing XYZ of pixel coordinate    
                        pointXYZ = PixToXYZ(u, v, cloudPointer); 
                       
                        //Place data in to ROS message
                        joint.position.x = pointXYZ.at(0);
                        joint.position.y = pointXYZ.at(1);
                        joint.position.z = pointXYZ.at(2);

                        //Add hand keypoint to vector of left hand keypoints
                        individual.left_hand_part.poses.push_back(joint);
                  
 
                    }
                   
                    //Loop for number of keypoints in right hand
                    for (auto handPart = 0 ; handPart < handKeypoints[1].getSize(1) ; handPart++)
                    {
                        //Define Pose message to be pushed to list of body parts
                        geometry_msgs::Pose joint;

                        //Create floats containing pixel coordinates of joint
                        float u = handKeypoints[1][{person, handPart, 0}];
                        float v = handKeypoints[1][{person, handPart, 1}];
                       
                        std::vector<float> pointXYZ (3);
                        //Create vector containing XYZ of pixel coordinate    
                        pointXYZ = PixToXYZ(u, v, cloudPointer); 
                       
                        //Place data in to ROS message
                        joint.position.x = pointXYZ.at(0);
                        joint.position.y = pointXYZ.at(1);
                        joint.position.z = pointXYZ.at(2);

                        //Add hand keypoint to vector of left hand keypoints
                        individual.right_hand_part.poses.push_back(joint);
 
                    }
                   
                }

                //If face detection is enabled
                if(FLAGS_face)
                {
                    //Loop for number of face parts
                    for(auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                    {
                        //Define Pose message to be pushed to list of body parts
                        geometry_msgs::Pose joint;

                        //Create floats containing the pixel coordinates of the face keypoints
                        float u = faceKeypoints[{person, facePart, 0}];
                        float v = faceKeypoints[{person, facePart, 1}];
                        std::vector<float> pointXYZ (3); 
                        //Create vector containing XYZ of pixel coordinate    
                        pointXYZ = PixToXYZ(u, v, cloudPointer); 

                        //Place data in to ROS message
                        joint.position.x = pointXYZ.at(0);
                        joint.position.y = pointXYZ.at(1);
                        joint.position.z = pointXYZ.at(2);

                        
                        //Add face keypoint to vector of face parts
                        individual.face_part.poses.push_back(joint);
                    }
                }   
               
               //Push person keypoints to the output message 
               individual.person_id = person;
               output.person.push_back(individual);
                    
            }   
 
            return output;
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

    }


};

/*
*\brief OpenImageSub class containing the callback for the loading of data to and from Openpose as well
* as outputting the data using the functions ad defined in UserOutputClass

*\param  mMessage pointer to the ROS message for the RGB image
*\param  rUserInputClass a boost pointer to the input class object
*\param  rUserOutputClass a std::shared_ptr to the output class object
*\param  mOpWrapper reference to the opWrapper used for managing inputs, outputs, multithreading...
*\param  imgSubscriber subscriber to RGB image
*\param  keyPublisher publisher node for the keypoints
*/

class OpenImageSub
{ 
    public: 
    template <class UserInputClass, class UserOutputClass> 

    //Define constructor
    OpenImageSub(op::Wrapper<std::vector<UserDatum>>& opWrapper, UserInputClass inputClass, UserOutputClass outputClass):
    rUserOutputClass{outputClass},
    rUserInputClass{inputClass},
    mOpWrapper{opWrapper}
    
    {
        //Create reference to original Node Handler
        auto& nh_ = rUserInputClass->nh;
        //Define an image transporter
        image_transport::ImageTransport it_(nh_);
        //Write the flag topic for the RGB subscriber to a string
        const std::string topic_rgb = FLAGS_camera_topic ; 
        const std::string topic_publisher = FLAGS_publish_topic ; 
        
        //ROS Advertise Topic//
        keyPublisher = nh_.advertise<openpose_ros_msgs::Persons>(topic_publisher, 5);
        //ROS Subscriber//
        imgSubscriber = it_.subscribe(topic_rgb, 5, &OpenImageSub::callback, this);
    

    }
    /*
    *\brief Callback emplaces data in to the input class, waits for it to be processed, then pops it
    * from the Openpose Wrapper object to be processed by functions provided in UserOutputClass
    */
    
    void callback (const sensor_msgs::ImageConstPtr& mMessage)
    {
         
          bool userWantsToExit = false;
          //Define output ROS message
          openpose_ros_msgs::Persons dataToPublish;
           
           op::log("Start of Callback Loop. ", op::Priority::Normal);

           // Push camera frame to be turned in to the correct format
           auto datumToProcess = rUserInputClass->createDatum(mMessage);
           
           if (datumToProcess != nullptr)
           {
                //Returns true if the datumToProcess was successfully added to the stack, and the dataCloud is not a null pointer
                auto successfullyEmplaced = mOpWrapper.waitAndPush(datumToProcess) && datumToProcess->at(0).dataCloud != nullptr;
                op::log("Emplaced = " + std::to_string(successfullyEmplaced), op::Priority::Low,__LINE__, __FUNCTION__, __FILE__);
               
                //Create a vector of UserDatum for processed data
                std::shared_ptr<std::vector<UserDatum>> datumProcessed;
                //if mOpWrapper returns the processed data 
                if (successfullyEmplaced && mOpWrapper.waitAndPop(datumProcessed))
                {
                    //Output processing of data
                    userWantsToExit = rUserOutputClass->display(datumProcessed);
                    dataToPublish = rUserOutputClass->toPoseStamped(datumProcessed);
                    //rUserOutputClass->printKeypoints(datumProcessed);
                    keyPublisher.publish(dataToPublish);
                }
                else
                    op::log("Processed datum could not be emplaced.", op::Priority::High,
                            __LINE__, __FUNCTION__, __FILE__);
            
            }
           

        op::log("Callback complete", op::Priority::Normal);
    }

    private:
        sensor_msgs::ImageConstPtr mMessage;
        boost::shared_ptr<UserInputClass> rUserInputClass;
        std::shared_ptr<UserOutputClass> rUserOutputClass;
        op::Wrapper<std::vector<UserDatum>>& mOpWrapper;
        image_transport::Subscriber imgSubscriber;
        ros::Publisher keyPublisher;

};
    
     

int telePoseROS()
{

    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
   // op::Profiler::setDefaultX(FLAGS_profile_speed);

    op::log("Starting pose estimation demo.", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    // Applying user defined configuration - Google flags to program variables
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // faceNetInputSize
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
    // handNetInputSize
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // JSON saving
    const auto writeJson = (!FLAGS_write_json.empty() ? FLAGS_write_json : FLAGS_write_keypoint_json);
    if (!FLAGS_write_keypoint.empty() || !FLAGS_write_keypoint_json.empty())
        op::log("Flags `write_keypoint` and `write_keypoint_json` are deprecated and will eventually be removed."
                " Please, use `write_json` instead.", op::Priority::Max);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // >1 camera view?
    const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
    // Enabling Google Logging
    const bool enableGoogleLogging = true;
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // Configure OpenPose
    op::Wrapper<std::vector<UserDatum>> opWrapper{op::ThreadManagerMode::Asynchronous};
    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{!FLAGS_body_disable, netInputSize, outputSize, keypointScale,
                                                  FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose, multipleView),
                                                  poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                                  heatMapTypes, heatMapScale, FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold, FLAGS_number_people_max,
                                                  enableGoogleLogging};//, FLAGS_3d, FLAGS_3d_min_views};
    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face, faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range, FLAGS_hand_tracking,
                                                  op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose, (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};

    // Consumer (comment or use default argument to disable any output)
    const auto displayMode = op::DisplayMode::NoDisplay;
    const bool guiVerbose = false;
    const bool fullScreen = false;
    const op::WrapperStructOutput wrapperStructOutput{op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen,
            FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format), FLAGS_write_json,
            FLAGS_write_coco_json, FLAGS_write_coco_foot_json, FLAGS_write_images, FLAGS_write_images_format,
            FLAGS_write_video, FLAGS_camera_fps, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format,
            FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
    // Configure wrapper
    op::log("Configuring OpenPose wrapper.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    opWrapper.configure(wrapperStructPose, wrapperStructFace, wrapperStructHand, op::WrapperStructExtra{}, op::WrapperStructInput{}, wrapperStructOutput);

    // Set to single-thread running (to debug and/or reduce latency)
    if (FLAGS_disable_multi_thread)
       opWrapper.disableMultiThreading();
   
    op::log("MultiThread Disabled", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    
    // Create class objects
    boost::shared_ptr<UserInputClass> inputClass{new UserInputClass};
    std::shared_ptr<UserOutputClass> outputClass{new UserOutputClass};
    boost::shared_ptr<OpenImageSub> imageProcessPtr (new OpenImageSub(opWrapper, inputClass, outputClass));

    op::log("Initialisation complete.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    op::log("Starting ROS Spin", op::Priority::High);
/////////////////////////////////////////////////////////////////////
    opWrapper.start();
     
    while (ros::ok()){
        ros::spinOnce();
    }
    
    opWrapper.stop();
/////////////////////////////////////////////////////////////////////
    op::log("Stopping ROS Spin", op::Priority::High);  

 
    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                            * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: "
                       + std::to_string(totalTimeSec) + " seconds.";
    op::log(message, op::Priority::High);
    return 0;
}

int main(int argc, char *argv[])
{   
    //Initialise ROS
    ros::init(argc, argv, "openpose_keypoints3d");    

    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true); 

    return telePoseROS();
       
}
