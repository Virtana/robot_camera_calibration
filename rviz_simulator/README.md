# `rviz_simulator`

[INSERT SS HERE]

## Package Description
The `rviz_simulator` is an interactive 3D environment built using RViz and Interactive Markers that can be used to generate fiducial target pixel locations.  



## Installation Instructions
Requires ROS Kinectic Kane.  
Clone the repo in the `src` folder of your catkin workspace.
Run `catkin_make` in the root of your workspace to build the `rviz_simulator` package.

## Execution Instructions
Edit the `initialize_simulator.yaml` file to change the simulator configuration.

Naviate to the root of your catkin workspace and run the `simulate.launch` file with the following command:  
> `roslaunch rviz_simulator simulate.launch`

`simulate.launch` launches 4 nodes:
- `rviz`
- `rqt_console`
- `rqt_logger_level`
- `simulate`

The `simulator` node creates a new folder `"detections_ROS_timestamp"` in the `rviz_simulator` package folder. A new detections folder with a new timestamp is created each time `roslaunch` is run.

Drag around the virtual camera and multiple virtual fiducial targets.  
  
Left click on the camera to generate a `YAML` file, in the created detections folder, with the measured target locations.