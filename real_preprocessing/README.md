# real\_preprocessing

### Additional packages  

* [apriltag\_ros](http://wiki.ros.org/apriltag_ros)
* [cv\_bridge](http://wiki.ros.org/cv_bridge)
* [tf\_conversions](http://wiki.ros.org/tf_conversions)

[OpenCV](https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/) was installed on Ubuntu 16.04 and included in CMakeLists.txt as follows

	find_package(find_package(OpenCV REQUIRED PATHS "YOUR-FOLDER-PATH")

### Image publishing 
**corner\_detection.cpp** simulates image capture with user input. By default, `""` /`ENTER` is the assigned trigger for image processing and YAML dumping. The YAML file for each frame detection is dumped at the working directory. To view the video stream detection, [image\_view](http://wiki.ros.org/image_view) was the library included. Rviz can be used alternatively with some configuration.

### Camera calibration
The package requires calibration of the camera to be used. This was done using [camera\_calibration](http://wiki.ros.org/camera_calibration).  The intrinsics obtained should be recorded as it is required for detection accuracy. Your intrinsic matrix, K, must be specified in **corner\_detection.cpp**, specifically at line 18,  `intrinsic << #, #, #, #, #, #, #, #, #;`.

### Steps: 
	1.  $ roscore 
	2.  $ rosrun cv_camera cv_camera_node 
	3.  $ roslaunch apriltag_ros continuous_detection.launch camera_name:=/cv_camera image_topic:=image_raw 
	4.  $ rosrun image_view image_view image:=tag_detections_image
	5.  $ rosrun real_preprocessing corner_detection_node
