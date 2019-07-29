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

#ifndef RVIZ_SIMULATOR_CAMERA_CALIBRATION_OPTIMIZER_H
#define RVIZ_SIMULATOR_CAMERA_CALIBRATION_OPTIMIZER_H

#include <ros/ros.h>
#include <ros/package.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

// for getting a directory/file listing
#include <dirent.h>

// for making directories
#include <sys/stat.h>
#include <sys/types.h>


namespace camera_calibration
{

#define REFERENCE_T_TARGET_SIZE 6
#define OBJ_POINTS_SIZE 3
#define CORNER_POINTS_SIZE 2
#define NUM_OBJ_POINTS 4
#define NUM_CORNERS 4
#define CAMERA_INTRINSICS_SIZE 9 // currently hardcoded for the plumb bob model

// All transforms are stored as a 6 elemnt double array
// [0, 1, 2] - rodriguez angle-axis rotation
// [3, 4, 5] - translation

/// The target struct models a fiducial target (E.g. an AprilTag)
/// It stores the world_T_target transform
/// and the 3D object point coordinates in the target frame
struct Target
{
  int targetID;
  std::array<double, REFERENCE_T_TARGET_SIZE> world_T_target;
  std::array<std::array<double, OBJ_POINTS_SIZE>, NUM_OBJ_POINTS> obj_points_in_target;
};

/// A detection may occur when a camera takes a picture
/// A dectection consists of a targetID and the pixel [x, y] coordinates of the 4 corners that appear in the image
/// This targetID is looked up in the targets collection to retrieve the corresponding world_t_target
struct Detection
{
  int targetID;
  std::array<std::array<double, CORNER_POINTS_SIZE>, NUM_CORNERS> corners;
};

/// A picture consists of 0 or more detections
/// The picture struct records the world_T_camera transform which is then inverted to get camera_T_world
struct Picture
{
  std::array<double, REFERENCE_T_TARGET_SIZE> world_T_camera;
  std::array<double, REFERENCE_T_TARGET_SIZE> camera_T_world;
  std::vector<Detection> detections;
};

/// Templated functor for the Bundle Adjustment problem
struct ReProjectionResidual
{
public:
  // Constructor
  ReProjectionResidual(const double* observed_pixel_coordinates, const double* obj_point_in_target);

  // Using the openCV camera calibration model:
  // https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  template <typename T>
  bool operator()(const T* const camera_intrinsics, const T* const camera_T_world, const T* const world_T_target,
                  T* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const double* observed_pixel_coordinates, const double* obj_point_in_target);

private:
  double observed_pixel_coordinates[2];
  double obj_point_in_target[3];
};

class CameraCalibrationOptimizer
{
public:
  CameraCalibrationOptimizer(std::string detections_directory_path);

  ~CameraCalibrationOptimizer();

  // runs the ceres non linear least squares optimizer on the loaded bundle adjustment problem
  void optimize();

  // optimized results consists of 
  // - camera_intrinsics
  // - world_T_camera for each image
  // - world_T_target for each fiducial target

  // Function outputs the optimization results to the console (screen)
  void printResultsToConsole();
  
  // Function outputs the optimization results to YAML files (screen)
  void printResultsToYAML(std::string output_directory_path);

private:
  // fields
  std::string detections_directory_path;
  std::array<double, CAMERA_INTRINSICS_SIZE> camera_intrinsics;
  std::array<double, CAMERA_INTRINSICS_SIZE> initial_intrinsics;
  std::map<int, Target> targets;
  std::vector<Picture> pictures;
  ceres::Problem bundleAdjustmentProblem;

  // methods
  // gets the directory path that contains all the picture files (E.g. detection_0.yaml)
  std::string getDetectionsDirectoryPath(ros::NodeHandle& n);

  // returns an array of camera intrinsics
  std::array<double, CAMERA_INTRINSICS_SIZE> getCameraIntrinsics(std::string camera_file_path);

  // reads target data from a YAML file
  // returns a vector of targets
  std::map<int, Target> getTargets(std::string targets_directory_path);

  // returns a vector of all pictures from a given directory
  // if there are no detections in a picture, then that picture is not pushed to the vector
  std::vector<Picture> getPictures(std::string detections_directory_path);

  // Function creates a ceres::Problem object and adds residual blocks from the bundle adjustment problem
  void buildBundleAdjustmentProblem();

  ///////////////////////////////////////////////////////////////// debugging functions ////////////////
  // print vector
  template <typename T>
  void printVector(std::vector<T> data);

  // print std::array
  template <typename T, std::size_t N>
  void printStdArray(std::array<T, N> data);

  void printTargets(std::map<int, Target> targets);

  void printPictures(std::vector<Picture> pictures);
};
}


#endif