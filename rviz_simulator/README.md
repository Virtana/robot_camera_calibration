# `rviz_simulator`


## Package Description
The `rviz_simulator` is an interactive 3D environment built using RViz and Interactive Markers that can be used to generate fiducial target pixel locations.  


## Installation Instructions
Requires ROS Kinectic Kane.  

Clone the repo into the `src` folder of your catkin workspace.
Run `catkin_make` in the root of your workspace to build the `rviz_simulator` package.  

### Required Dependencies
- YAML-cpp [Installation Instructions](https://github.com/jbeder/yaml-cpp)
- Eigen [Installation Instructions](http://eigen.tuxfamily.org/dox/GettingStarted.html)
- Ceres [Installation Instructions](http://ceres-solver.org/installation.html)
- [ros-perception/image_common](https://github.com/ros-perception/image_common/tree/1.11.13) 
  - Clone the `image_common` repo into your catkin workspace `src` folder. 
  - Add "`add_compile_options(-std=c++11)`" to the `camera_calibration_parsers` CMakeLists.txt

---
## Execution Instructions
---
### `simulate` Node
Edit the `initialize_simulator.yaml` file to change the simulator configuration.

Naviate to the root of your catkin workspace and run the `simulate.launch` file with the following command:  
> `roslaunch rviz_simulator simulate.launch`

`simulate.launch` launches 2 nodes in normal mode:
- `rviz`
- `simulate`

and 4 nodes in `debug_mode`:
- `rviz`
- `rqt_console`
- `rqt_logger_level`
- `simulate`

To enter `debug_mode`, run the launch file with the command line arg 
> `debug_mode:=true`

By default, the `photoneo_camera_intrinsics.yaml` file is loaded. To specify another camera intrinsics file, run the launch file with the command line arg
> `camera_intrinsics_file:=<camera_intrinsics_file_name.yaml>`  

The `simulator` node creates a new folder `"detections_ROS_timestamp"` in the `rviz_simulator` package folder. A new detections folder with a new timestamp is created each time `roslaunch` is run.

Drag around the virtual camera and multiple virtual fiducial targets.  
  
Left click on the camera to generate a `YAML` file, in the created detections folder, with the measured target locations.

---

### `synthetic_optimization` Node
Ceres optimizer for a bundle adjustment camera calibration problem. Considers the reprojection error to optimize the following parameters:
 *    world_T_camera
 *    world_T_target
 *    camera_intrinsics

Start `roscore` in a new terminal.  
Run the following command in another terminal:
> `rosrun rviz_simulator synthetic_optimization _dir_name:=<detections_directory_name>`  

where the `detections_directory_name` is a directory of camera, targets and detection yaml files in the `rviz_simulator` package directory.  

The output is written to a folder labelled "`optimized`" which is created in the `detections_directory_name` folder.
