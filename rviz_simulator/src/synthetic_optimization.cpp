/*
 * Copyright (c) 2019, Virtana TT Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christopher Sahadeo
 *
 * Ceres optimizer for a bundle adjustment camera calibration problem
 * Considers the reprojection error to optimize the following parameters:
 *    world_T_camera
 *    world_T_target
 *    camera_intrinsics
 *
 */

/* TODO:
 * 
 * fix optimize.launch
 * hardcoded for plumb-bob model
 * templating the YAML-cpp readers to read automatically/refactoring the save methods
 * Use a YAML conversions class for all other nodes
 * exception handling
 *
 */

#include "rviz_simulator/camera_calibration_optimizer.h"

std::string getDetectionsDirectoryPath(ros::NodeHandle& n)
{
  // checking if the detections directory name was passed as a cmd line arg
  std::string detections_directory_name;
  if (!n.getParam("dir_name", detections_directory_name))
  {
    ROS_ERROR("Detections directory name not specified.\n");
    ros::shutdown();
  }

  // checking existance of detections directory
  std::string package_path = ros::package::getPath("rviz_simulator");
  std::string detections_directory_path = package_path + "/detections/" + detections_directory_name;
  struct stat info;
  if (stat(detections_directory_path.c_str(), &info) != 0)
  {
    ROS_ERROR("Detections directory read error.\n");
    ros::shutdown();
  }

  return detections_directory_path;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synthetic_optimization");
  ros::NodeHandle n("~");

  std::string detections_directory_path = getDetectionsDirectoryPath(n);
  camera_calibration::CameraCalibrationOptimizer camera_calibration_optimizer(detections_directory_path);
  camera_calibration_optimizer.optimize();
  // camera_calibration_optimizer.printResultsToConsole();
  camera_calibration_optimizer.writeResultsToYAML();

  return 0;
}
