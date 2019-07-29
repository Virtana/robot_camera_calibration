# real\_preprocessing

### Additional packages  

* [apriltag\_ros]
* [cv\_bridge](http://wiki.ros.org/cv_bridge)
* [tf\_conversions](http://wiki.ros.org/tf_conversions)

[OpenCV](https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/) was installed on Ubuntu 16.04 and included in CMakeLists.txt as follows

	find_package(find_package(OpenCV REQUIRED PATHS "YOUR-FOLDER-PATH")

### apriltag_ros library 
The **apriltag\_ros** library was forked and modified to publish pixel coordinates of detected tags on the /tag_detections rostopic. This was done as an alternative method to the homography concepts which used used camera intrinsics for pixel point determination. This fork can be found [here](https://github.com/DAA2310/apriltag_ros/tree/milestone_1b_pipeline) and is the assumed publisher to **corner\_detections.cpp**.

### Camera calibration
The package requires calibration of the camera for an initial guess of the camera intrinsics, to be used for OpenCV's SolvePNP function. This was done using [camera\_calibration](http://wiki.ros.org/camera_calibration). The calibration package generates a "camera.yaml" file which is used in **camera\_pose.cpp**.

### Image publishing 
**corner\_detections.cpp** simulates image capture with user input. By default, `""` /`ENTER` is the assigned trigger for image processing and YAML dumping. The YAML file for each frame detection is dumped at the working directory named "detection_n.yml" for the nth image taken. To view the video stream detection, [image\_view](http://wiki.ros.org/image_view) was the library included. Rviz can be used alternatively with some configuration.

### File handling
**corner\_detections.cpp** generates detection YAML files in the directory in which it is called. Until a launch file is implemented, **camera\_pose.cpp** must be run in the same directory such that these YAML files can be parsed. 

### Steps: 
	1.  $ roscore 
	2.  $ rosparam load ~/.ros/camera_info/camera.yaml
	3.  $ rosrun cv_camera cv_camera_node 
	4.  $ roslaunch apriltag_ros continuous_detection.launch camera_name:=/cv_camera image_topic:=image_raw 
	5.  $ rosrun image_view image_view image:=tag_detections_image
	6.  $ rosrun real_preprocessing corner_detection_node
	7.  $ rosrun real_preprocessing camera_pose_node
