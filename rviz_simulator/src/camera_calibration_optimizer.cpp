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

#include "rviz_simulator/camera_calibration_optimizer.h"

namespace camera_calibration
{
ReProjectionResidual::ReProjectionResidual(const double* observed_pixel_coordinates, const double* obj_point_in_target)
{
  // projected image pixel coordinates (u,v)
  this->observed_pixel_coordinates[0] = observed_pixel_coordinates[0];
  this->observed_pixel_coordinates[1] = observed_pixel_coordinates[1];

  // 3D coordinates in ROSWorld with tag0 as origin (X, Y, Z)
  this->obj_point_in_target[0] = obj_point_in_target[0];
  this->obj_point_in_target[1] = obj_point_in_target[1];
  this->obj_point_in_target[2] = obj_point_in_target[2];
}

template <typename T>
bool ReProjectionResidual::operator()(const T* const camera_intrinsics, const T* const camera_T_world, const T* const world_T_target, T* residuals) const
{
  const T point[3] = { T(this->obj_point_in_target[0]), T(this->obj_point_in_target[1]),
                        T(this->obj_point_in_target[2]) };

  T p[3];

  ceres::AngleAxisRotatePoint(world_T_target, point, p);
  p[0] += world_T_target[3];
  p[1] += world_T_target[4];
  p[2] += world_T_target[5];

  ceres::AngleAxisRotatePoint(camera_T_world, p, p);
  p[0] += camera_T_world[3];
  p[1] += camera_T_world[4];
  p[2] += camera_T_world[5];

  T x_prime = p[0] / p[2];
  T x_prime_squared = x_prime * x_prime;
  T y_prime = p[1] / p[2];
  T y_prime_squared = y_prime * y_prime;

  const T& fx = camera_intrinsics[0];
  const T& fy = camera_intrinsics[1];
  const T& cx = camera_intrinsics[2];
  const T& cy = camera_intrinsics[3];
  const T& k1 = camera_intrinsics[4];
  const T& k2 = camera_intrinsics[5];
  const T& p1 = camera_intrinsics[6];
  const T& p2 = camera_intrinsics[7];
  const T& k3 = camera_intrinsics[8];
  // const T& k4 = camera_intrinsics[9];
  // const T& k5 = camera_intrinsics[10];
  // const T& k6 = camera_intrinsics[11];
  const T k4 = T(0.0);
  const T k5 = T(0.0);
  const T k6 = T(0.0);

  T r_raise_2 = x_prime_squared + y_prime_squared;
  T r_raise_4 = r_raise_2 * r_raise_2;
  T r_raise_6 = r_raise_4 * r_raise_2;

  T numerator = T(1) + k1 * r_raise_2 + k2 * r_raise_4 + k3 * r_raise_6;
  T denominator = T(1) + k4 * r_raise_2 + k5 * r_raise_4 + k6 * r_raise_6;
  T coefficient = numerator / denominator;

  T x_double_prime =
      x_prime * coefficient + T(2) * p1 * x_prime * y_prime + p2 * (r_raise_2 + T(2) * x_prime_squared);
  T y_double_prime =
      y_prime * coefficient + p1 * (r_raise_2 + T(2) * y_prime_squared) + T(2) * p2 * x_prime * y_prime;

  T u = fx * x_double_prime + cx;
  T v = fy * y_double_prime + cy;

  residuals[0] = u - T(this->observed_pixel_coordinates[0]);
  residuals[1] = v - T(this->observed_pixel_coordinates[1]);

  return true;
}

static ceres::CostFunction* Create(const double* observed_pixel_coordinates, const double* obj_point_in_target)
{
  return (new ceres::AutoDiffCostFunction<ReProjectionResidual, 2, CAMERA_INTRINSICS_SIZE, REFERENCE_T_TARGET_SIZE,
                                          REFERENCE_T_TARGET_SIZE>(
      new ReProjectionResidual(observed_pixel_coordinates, obj_point_in_target)));
}

CameraCalibrationOptimizer::CameraCalibrationOptimizer(std::string detections_directory_path)
{
  this->detections_directory_path_ = detections_directory_path;
  this->camera_intrinsics_ = this->getCameraIntrinsics(this->detections_directory_path_+"/camera.yaml");
  this->initial_intrinsics_ = this->camera_intrinsics_;
  this->targets_ = this->getTargets(this->detections_directory_path_+"/targets.yaml");
  this->pictures_ = this->getPictures(this->detections_directory_path_);
}

CameraCalibrationOptimizer::~CameraCalibrationOptimizer()
{
}


std::array<double, CAMERA_INTRINSICS_SIZE> CameraCalibrationOptimizer::getCameraIntrinsics(std::string camera_file_path)
{
  YAML::Node camera_node = YAML::LoadFile(camera_file_path);
  if (!camera_node)
  {
    ROS_ERROR("Camera intrisics file read error.");
  }

  std::array<double, CAMERA_INTRINSICS_SIZE> camera_intrinsics;

  // XXX: hardcoded for the plumb_bob distortion model
  camera_intrinsics[0] = camera_node["camera_matrix"]["data"][0].as<double>();  // fx
  camera_intrinsics[1] = camera_node["camera_matrix"]["data"][4].as<double>();  // fy
  camera_intrinsics[2] = camera_node["camera_matrix"]["data"][2].as<double>();  // cx
  camera_intrinsics[3] = camera_node["camera_matrix"]["data"][5].as<double>();  // cy

  // plumb_bob distortions
  camera_intrinsics[4] = camera_node["distortion_coefficients"]["data"][0].as<double>();  // k1
  camera_intrinsics[5] = camera_node["distortion_coefficients"]["data"][1].as<double>();  // k2
  camera_intrinsics[6] = camera_node["distortion_coefficients"]["data"][2].as<double>();  // p1
  camera_intrinsics[7] = camera_node["distortion_coefficients"]["data"][3].as<double>();  // p2
  camera_intrinsics[8] = camera_node["distortion_coefficients"]["data"][4].as<double>();  // k3
  // camera_intrinsics[9] = 0;                                                               // k4
  // camera_intrinsics[10] = 0;                                                              // k5
  // camera_intrinsics[11] = 0;                                                              // k6

  return camera_intrinsics;
}

std::array<double, 6> getReference_T_Target(const YAML::Node& node, std::string reference_T_target_name, bool inverse)
{
  std::array<double, 6> reference_T_target;

  if (inverse)
  {
    // getting rodrigues angle axis rotation and converting to Eigen::Matrix3d
    std::array<double, 3> rodrigues_angle_axis = node[reference_T_target_name]["rotation"].as<std::array<double, 3>>();
    double rotation_array[9];
    ceres::AngleAxisToRotationMatrix(rodrigues_angle_axis.data(), rotation_array);
    Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(rotation_array);

    // getting translation and converting to Eigen::Vector3d
    std::array<double, 3> translation = node[reference_T_target_name]["translation"].as<std::array<double, 3>>();
    Eigen::Vector3d translation_vector = Eigen::Map<Eigen::Vector3d>(translation.data());

    // combining rotation matrix and translation vector to Eigen::Affine3d
    Eigen::Affine3d transform;
    transform.translation() = translation_vector;
    transform.linear() = rotation_matrix;

    // inverting the transform
    Eigen::Affine3d transform_inverse = transform.inverse();

    // converting from rotation matrix back to rodriguez angle axis
    ceres::RotationMatrixToAngleAxis(transform_inverse.rotation().data(), rodrigues_angle_axis.data());

    // storing in an std::array
    reference_T_target[0] = rodrigues_angle_axis[0];
    reference_T_target[1] = rodrigues_angle_axis[1];
    reference_T_target[2] = rodrigues_angle_axis[2];

    reference_T_target[3] = transform_inverse.translation().x();
    reference_T_target[4] = transform_inverse.translation().y();
    reference_T_target[5] = transform_inverse.translation().z();
  }
  else
  {
    reference_T_target[0] = node[reference_T_target_name]["rotation"][0].as<double>();
    reference_T_target[1] = node[reference_T_target_name]["rotation"][1].as<double>();
    reference_T_target[2] = node[reference_T_target_name]["rotation"][2].as<double>();

    reference_T_target[3] = node[reference_T_target_name]["translation"][0].as<double>();
    reference_T_target[4] = node[reference_T_target_name]["translation"][1].as<double>();
    reference_T_target[5] = node[reference_T_target_name]["translation"][2].as<double>();
  }

  return reference_T_target;
}


std::map<int, Target> CameraCalibrationOptimizer::getTargets(std::string targets_directory_path)
{
  YAML::Node targets_node = YAML::LoadFile(targets_directory_path);
  if (!targets_node)
  {
    ROS_ERROR("Targets file read error.");
  }

  std::map<int, Target> targets;

  // iterating through all targets
  for (YAML::const_iterator target_iterator = targets_node["targets"].begin();
       target_iterator != targets_node["targets"].end(); ++target_iterator)
  {
    const YAML::Node& target_node = *target_iterator;
    Target target;

    // targetID
    target.targetID = target_node["targetID"].as<int>();

    // world_T_target
    target.world_T_target = getReference_T_Target(target_node, "world_T_target", false);

    // obj_points_in_target
    for (YAML::const_iterator obj_points_iterator = target_node["obj_points_in_target"].begin();
         obj_points_iterator != target_node["obj_points_in_target"].end(); ++obj_points_iterator)
    {
      int corner_index = obj_points_iterator->first.as<int>();
      target.obj_points_in_target[corner_index] = obj_points_iterator->second.as<std::array<double, OBJ_POINTS_SIZE>>();
    }

    targets.insert(std::pair<int, Target>(target.targetID, target));
  }

  return targets;
}

// returns a single detection from the YAML file
Detection getDetection(const YAML::Node& detection_node)
{
  Detection detection;
  detection.targetID = detection_node["targetID"].as<int>();
  for (YAML::const_iterator corner_iterator = detection_node["corners"].begin();
       corner_iterator != detection_node["corners"].end(); ++corner_iterator)
  {
    int corner_index = corner_iterator->first.as<int>();
    detection.corners[corner_index] = corner_iterator->second.as<std::array<double, CORNER_POINTS_SIZE>>();
  }
  return detection;
}

// returns a vector of all detections from a given YAML file
std::vector<Detection> getDetections(const YAML::Node& detections_node)
{
  std::vector<Detection> detections;
  for (YAML::const_iterator detection_iterator = detections_node.begin(); detection_iterator != detections_node.end();
       ++detection_iterator)
  {
    detections.push_back(getDetection(*detection_iterator));
  }
  return detections;
}

// returns a picture from a single YAML file
Picture getPicture(std::string picture_file_path)
{
  YAML::Node picture_node = YAML::LoadFile(picture_file_path);
  if (!picture_node)
  {
    ROS_ERROR("Picture file read error.");
  }

  Picture picture;
  picture.world_T_camera = getReference_T_Target(picture_node, "world_T_camera", false);
  picture.camera_T_world = getReference_T_Target(picture_node, "world_T_camera", true);
  picture.detections = getDetections(picture_node["detections"]);
  return picture;
}


// searches a given directory
// returns the list of detection entites (an entity is a directory or file) with the substring "detections"
std::vector<std::string> getDetectionEntityNames(std::string directory_path, unsigned char entity_type)
{
  std::vector<std::string> detection_entities;

  DIR* dir = opendir(directory_path.c_str());
  struct dirent* entry = readdir(dir);
  while (entry != NULL)
  {
    if (entry->d_type == entity_type)
    {
      std::string name = entry->d_name;
      if (name.find("detections") != std::string::npos)  // a directory
      {
        detection_entities.push_back(name);
      }
    }
    entry = readdir(dir);
  }
  return detection_entities;
}

std::vector<Picture> CameraCalibrationOptimizer::getPictures(std::string detections_directory_path)
{
  // get list of detection files
  std::vector<std::string> detection_file_names = getDetectionEntityNames(detections_directory_path, DT_REG);

  std::vector<Picture> pictures;
  for (auto detection_file_name : detection_file_names)
  {
    Picture picture = getPicture(detections_directory_path + "/" + detection_file_name);
    if (!picture.detections.empty())
    {
      pictures.push_back(picture);
      ROS_INFO_STREAM("Picture added: " << detection_file_name);
    }
    else
    {
      ROS_INFO_STREAM("No detections in picture: " << detection_file_name);
    }
  }
  return pictures;
}

void CameraCalibrationOptimizer::buildBundleAdjustmentProblem()
{
  for (int p = 0; p < this->pictures_.size(); p++)
  {
    for (int d = 0; d < this->pictures_[p].detections.size(); d++)
    {
      for (int c = 0; c < this->pictures_[p].detections[d].corners.size(); c++)
      {
        ceres::CostFunction* cost_function =
            Create(this->pictures_[p].detections[d].corners[c].data(),
                                         this->targets_.at(pictures_[p].detections[d].targetID).obj_points_in_target[c].data());

        this->bundle_adjustment_problem_.AddResidualBlock(cost_function, NULL, this->camera_intrinsics_.data(), this->pictures_[p].camera_T_world.data(),
                                 this->targets_.at(this->pictures_[p].detections[d].targetID).world_T_target.data());
      }
    }
  }
}

void CameraCalibrationOptimizer::optimize()
{
  this->buildBundleAdjustmentProblem();
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &(this->bundle_adjustment_problem_), &summary);
  ROS_INFO_STREAM(summary.FullReport());
}

void CameraCalibrationOptimizer::printResultsToConsole()
{
  ROS_INFO_STREAM("INITIAL\t\tFINAL");
  for (int i = 0; i < camera_intrinsics_.size(); i++)
  {
    ROS_INFO_STREAM(this->initial_intrinsics_[i] << "\t\t" << camera_intrinsics_[i]);
  }
  this->printTargets(this->targets_);
}

// Reads the original camera.yaml file into CameraInfo object using camera_calibration_parser
// Edits in optimized values in the object then outputs back to a YAML file
/*
void CameraCalibrationOptimizer::cameraToYAML()
{
  std::string old_camera_file_path = this->detections_directory_path_+"/camera.yaml";
  std::string camera_name;
  sensor_msgs::CameraInfo camera_info;
  if(!camera_calibration_parsers::readCalibration(old_camera_file_path, camera_name, camera_info))
  {
    ROS_ERROR_STREAM("Camera file read error: " << old_camera_file_path);
    ros::shutdown();
  }
  
  boost::array<double, CAMERA_INTRINSICS_SIZE> K = 
  {
    this->camera_intrinsics_[0], 0, this->camera_intrinsics_[1],
    0, this->camera_intrinsics_[2], this->camera_intrinsics_[3],
    0, 0, 1
  };
  camera_info.K = K;

  std::vector<double> D = 
  {
    this->camera_intrinsics_[4],
    this->camera_intrinsics_[5],
    this->camera_intrinsics_[6],
    this->camera_intrinsics_[7],
    this->camera_intrinsics_[8]
  };
  camera_info.D = D;

  std::string new_camera_file_path = this->detections_directory_path_+"/optimized/camera.yaml";
  if(!camera_calibration_parsers::writeCalibration(new_camera_file_path, camera_name, camera_info))
  {
    ROS_ERROR_STREAM("Camera file write error: " << old_camera_file_path);
    ros::shutdown();
  }
}
*/

void targetToYAML(Target target, YAML::Emitter &out)
{
  out << YAML::BeginMap; // 0
  out << YAML::Key << "targetID" << YAML::Value << target.targetID;
  out << YAML::Key << "world_T_target" << YAML::Value;

  out << YAML::BeginMap; // 1
  std::vector<double> rotation = { target.world_T_target[0], target.world_T_target[1], target.world_T_target[2] };;
  out << YAML::Key << "rotation" << YAML::Value << YAML::Flow << rotation;
  std::vector<double> translation = { target.world_T_target[3], target.world_T_target[4], target.world_T_target[5] };;
  out << YAML::Key << "translation" << YAML::Value << YAML::Flow << translation;
  out << YAML::EndMap; // 1

  out << YAML::Key << "obj_points_in_target" << YAML::Value;
  out << YAML::BeginMap; // 1
  for(int i = 0; i < target.obj_points_in_target.size(); i++)
  {
    std::vector<double> obj_points_in_target(target.obj_points_in_target[i].begin(), target.obj_points_in_target[i].end());
    out << YAML::Key << i << YAML::Value << YAML::Flow << obj_points_in_target;
  }
  out << YAML::EndMap; // 1

  out << YAML::EndMap; // 0
}

void CameraCalibrationOptimizer::targetsToYAML()
{
  YAML::Emitter targets_out;
  targets_out << YAML::BeginMap;
  targets_out << YAML::Key << "targets" << YAML::Value << YAML::BeginSeq;

  std::map<int, Target>::iterator itr;
  for(itr = this->targets_.begin(); itr != this->targets_.end(); ++itr)
  {
    targetToYAML(itr->second, targets_out);
  }
  targets_out << YAML::EndSeq;
  targets_out << YAML::EndMap;


  // writing to targets.yamls
  std::string output_file_path = this->detections_directory_path_+"/optimized/targets.yaml";
  std::ofstream fout(output_file_path);
  if (!fout)
  {
    ROS_ERROR_STREAM("Error writing to targets.yaml:  " << strerror(errno) << std::endl);
  }
  fout << targets_out.c_str();
  fout.close();
}

void CameraCalibrationOptimizer::writeResultsToYAML()
{
  // making optimized data directory
  std::string output_directory_path = this->detections_directory_path_ + "/optimized";
  if (mkdir(output_directory_path.c_str(), 0777) == -1)
  {
    ROS_WARN_STREAM("Error making output directory :  " << strerror(errno) << std::endl);
    // ros::shutdown();
  }

  // writes optimized camera info to YAML file
  // this->cameraToYAML(); 
  this->targetsToYAML();
}

///////////////////////////////////////////////////////////////// debugging functions ////////////////
// print vector
template <typename T>
void CameraCalibrationOptimizer::printVector(std::vector<T> data)
{
  for (T element : data)
  {
    ROS_INFO_STREAM(element);
  }
  ROS_INFO_STREAM(std::endl);
}

// print std::array
template <typename T, std::size_t N>
void CameraCalibrationOptimizer::printStdArray(std::array<T, N> data)
{
  for (T element : data)
  {
    ROS_INFO_STREAM(element);
  }
  ROS_INFO_STREAM(std::endl);
}

void CameraCalibrationOptimizer::printTargets(std::map<int, Target> targets)
{
  std::map<int, Target>::iterator itr;
  for(itr = targets.begin(); itr != targets.end(); ++itr){
    //  const YAML::Node& target_node = *target_iterator;
    const Target target = itr->second;
    ROS_INFO_STREAM("targetID: " << itr->first);
    ROS_INFO_STREAM("world_T_target: ");
    printStdArray(target.world_T_target);
    // ROS_INFO_STREAM("obj_points_in_target: ");
    // for (int j = 0; j < target.obj_points_in_target.size(); j++)
    // {
    //   ROS_INFO_STREAM("Corner index: " << j);
    //   printStdArray(target.obj_points_in_target[j]);
    // }
  }
}

void CameraCalibrationOptimizer::printPictures(std::vector<Picture> pictures)
{
  for (int i = 0; i < pictures.size(); i++)
  {
    ROS_INFO_STREAM("Picture #" << i);
    ROS_INFO("world_T_camera");
    printStdArray(pictures[i].world_T_camera);
    ROS_INFO("camera_T_world");
    printStdArray(pictures[i].camera_T_world);
    for (Detection detection : pictures[i].detections)
    {
      ROS_INFO_STREAM("targetID: " << detection.targetID);
      for (int j = 0; j < detection.corners.size(); j++)
      {
        ROS_INFO_STREAM("Corner #" << j << ": ");
        printStdArray(detection.corners[j]);
      }
    }
  }
  ROS_INFO("\n");
}
///////////////////////////////////////////////////////////////// debugging functions ////////////////


}