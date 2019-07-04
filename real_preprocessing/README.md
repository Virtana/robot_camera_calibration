# real\_preprocessing

### Additional packages  

* [apriltag\_ros](http://wiki.ros.org/apriltag_ros)
* [cv\_bridge](http://wiki.ros.org/cv_bridge)
* [tf\_conversions](http://wiki.ros.org/tf_conversions)

[OpenCV](https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/) was installed on Ubuntu 16.04 and included in CMakeLists.txt as follows

	find_package(find_package(OpenCV REQUIRED PATHS "YOUR-FOLDER-PATH")

### Image publishing 
**corner\_detection.cpp** simulates image capture with user input. By default, '0' is the assigned trigger for image processing and YAML dumping. This can be changed at line 16, `if(capture=="0")`. The YAML file for each frame detection is dumped at the working directory. To view the video stream detection, [image\_view](http://wiki.ros.org/image_view) was the library included. Rviz can be used alternatively with some configuration. NOTE: **corner\_detection.cpp** is currently unable to accurately process distinct,consecutive frames i.e. two frames with varying tag positions/numbers cannot be processed with repeated '0' input. Testing showed that consecutive dumps give the same data. It is suggested that images are processed with every other trigger i.e. '0' to process current frame -> change frame -> 'ENTER' or any other input -> '0' to process new frame. 

### Camera calibration
The package requires calibration of the camera to be used. This was done using [camera\_calibration](http://wiki.ros.org/camera_calibration).  The intrinsics obtained should be recorded as it is required for detection accuracy. Your intrinsic matrix, K, must be specified in **corner\_detection.cpp**, specifically at line 13,  `intrinsic << #, #, #, #, #, #, #, #, #;`.

### Steps: 
	1.  $ roscore 
	2.  $ rosrun cv_camera cv_camera 
	3.  $ roslaunch apriltag_ros continuous_detection.launch camera_name:=/cv_camera image_topic:=image_raw 
	4.  $ rosrun image_view image_view image:=tag_detections_image
	5.  $ rosrun real_preprocessing corner_detection.cpp 
