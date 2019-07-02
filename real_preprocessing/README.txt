Additional packages required for real_preprocessing: 

	apriltag_ros (http://wiki.ros.org/apriltag_ros)
	cv_bridge (http://wiki.ros.org/cv_bridge)
	tf_conversions (http://wiki.ros.org/tf_conversions)

To be included  in package.xml as dependencies and CMakelists.txt.

In package.xml, the following must be included for each package. (Dependency declaration depends on DISTRO!) 
Kinetic uses:
	<build_depend>apriltag_ros</build_depend>
	<exec_depend>apriltag_ros</exec_depend>
	<build_export_depend>apriltag_ros</build_export_depend>

In CMakeList, packages are to be included ->

	find_package( catkin REQUIRED COMPONENTS ..here..)

	catkin_package(CATKIN_DEPENDS ..here..)

Must also include OpenCV and Eigen directories. Mimic ->

	include_directories(
	  ${catkin_INCLUDE_DIRS}
	  ${OpenCV_INCLUDE_DIRS}
	  ${Eigen_INCLUDE_DIRS}
	)

OpenCV was installed on Ubuntu 16.04 using (https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/) and included in CMakeLists.txt as follows.

	find_package(find_package(OpenCV REQUIRED PATHS "YOUR-FOLDER-PATH")

Camera calibration is required. This was done using camera_calibration under image_pipeline. See (http://wiki.ros.org/camera_calibration).
The intrinsics obtained should be recorded as it is required for detection accuracy. Your intrinsic matrix, K, must be specified in corner_detection_cpp.

Specifically @ line 13
	intrinsic << #, #, #, #, #, #, #, #, #;

Steps: 
	1.  $ roscore 
	2.  Separate terminal $ rosrun cv_camera cv_camera (Runs camera driver)
	3.  Separate terminal $ roslaunch apriltag_ros continuous_detection.launch camera_name:=/cv_camera image_topic:=image_raw (Runs video stream detection with manual remapping of camera driver to detection node )
	4.  Separate terminal $ rosrun real_preprocessing corner_detection (Dumps YAML to path where this is run)


 
