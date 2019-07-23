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
 */

/**
 * TODO: 
 * 
 * templating the YAML-cpp readers to read automatically/refactoring the save methods
 * exception handling
 * 
 */

#include <ros/ros.h>
#include <ros/package.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <yaml-cpp/yaml.h>

// getting a directory/file listing
#include <dirent.h>

// for making directories
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Dense>

#define REFERENCE_T_TARGET_SIZE 6
#define OBJ_POINTS_SIZE 3
#define CORNER_POINTS_SIZE 2
#define NUM_OBJ_POINTS 4
#define NUM_CORNERS 4
#define CAMERA_INTRINSICS_SIZE 12

struct Target
{
  std::array<double, REFERENCE_T_TARGET_SIZE> world_T_target;
  std::array<std::array<double, OBJ_POINTS_SIZE>, NUM_OBJ_POINTS> obj_points_in_target;
};

struct Detection
{
  int targetID;
  std::array<std::array<double, CORNER_POINTS_SIZE>, NUM_CORNERS> corners;
};

struct Picture
{
  std::array<double, REFERENCE_T_TARGET_SIZE> world_T_camera;
  std::array<double, REFERENCE_T_TARGET_SIZE> camera_T_world;
  std::vector<Detection> detections;
};


struct ReProjectionResidual
{
  // Constructor
  ReProjectionResidual(const double *observed_pixel_coordinates, const double *obj_point_in_target)
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
  bool operator()(const T* const camera_intrinsics, const T* const camera_T_world, const T* const world_T_target, T* residuals) 
  const
  {
    // for(int i = 0; i < 7; i++)
    //   ROS_INFO_STREAM("FROM CERES" << camera_extrinsics[i]);
    // ROS_INFO_STREAM(std::endl);

    // ROS_INFO_STREAM("FROM CERES: wTt[0] " << (world_T_target[0]) << "wTt[1]: " << (world_T_target[1]) << "wTt[2]: " << (world_T_target[2]));
    // // for(int i = 0; i < 16; i++)
    // //   ROS_INFO_STREAM(*(world_T_camera+i));

    const T point[3] = 
    {
      T(this->obj_point_in_target[0]),
      T(this->obj_point_in_target[1]),
      T(this->obj_point_in_target[2])
    };
    T p[3];
    
    ROS_INFO_STREAM("CERES: x: " << point[0] << "y: " << point[1] << "z: " << point[2]);
    // const T world_T_target_angle_axis[3] = {T(world_T_target[0]), T(world_T_target[1]), T(world_T_target[2])};

    ceres::AngleAxisRotatePoint(world_T_target, point, p);
    p[0] += world_T_target[3];
    p[1] += world_T_target[4];
    p[2] += world_T_target[5];

    ROS_INFO_STREAM("CERES: p0: " << p[0] << "p1: " << p[1] << "p2: " << p[2]);


    const T point2[3] = {p[0], p[1], p[2]};

    // const T camera_T_world_angle_axis[3] = {-T(camera_T_world[0]), -T(camera_T_world[1]), -T(camera_T_world[2])};
    ceres::AngleAxisRotatePoint(camera_T_world, point2, p);
    
    p[0] += camera_T_world[3];
    p[1] += camera_T_world[4];
    p[2] += camera_T_world[5];

    // trying eigen
    // T world_T_target_quarternion[4];
    // const T world_T_target_angle_axis[3] = {T(world_T_target[0]), T(world_T_target[1]), T(world_T_target[2])};
    // ceres::AngleAxisToQuaternion(world_T_target_angle_axis, world_T_target_quarternion);
    // Eigen::Quaternion<T> world_T_target_eigen_quarternion(world_T_target_quarternion[0],world_T_target_quarternion[1],world_T_target_quarternion[2],world_T_target_quarternion[3]);    


    ROS_INFO_STREAM("CERES: p0: " << p[0] << "p1: " << p[1] << "p2: " << p[2]);

  
    // scaling
    T x_prime = p[0] / p[2];
    T x_prime_squared = x_prime * x_prime;
    T y_prime = p[1] / p[2];
    T y_prime_squared = y_prime * y_prime;

    ROS_INFO_STREAM("CERES: x_prime: " << x_prime << "y_prime: " << y_prime);

    // rectifying the camera distortion
    const T& fx = camera_intrinsics[0];
    const T& fy = camera_intrinsics[1];
    const T& cx = camera_intrinsics[2];
    const T& cy = camera_intrinsics[3];
    const T& k1 = camera_intrinsics[4];
    const T& k2 = camera_intrinsics[5];
    const T& p1 = camera_intrinsics[6];
    const T& p2 = camera_intrinsics[7];
    const T& k3 = camera_intrinsics[8];
    const T& k4 = camera_intrinsics[9];
    const T& k5 = camera_intrinsics[10];
    const T& k6 = camera_intrinsics[11];  

    T r_raise_2 = x_prime_squared + y_prime_squared;
    T r_raise_4 = r_raise_2 * r_raise_2;
    T r_raise_6 = r_raise_4 * r_raise_2;

    T numerator = T(1) + k1 * r_raise_2 + k2 * r_raise_4 + k3 * r_raise_6;
    T denominator = T(1) + k4 * r_raise_2 + k5 * r_raise_4 + k6 * r_raise_6;
    T coefficient = numerator / denominator;

    T x_double_prime = x_prime * coefficient + T(2) * p1 * x_prime * y_prime +
                            p2 * (r_raise_2 + T(2) * x_prime_squared);
    T y_double_prime = y_prime * coefficient + p1 * (r_raise_2 + T(2) * y_prime_squared) +
                            T(2) * p2 * x_prime * y_prime;

    T u = fx * x_double_prime + cx;
    T v = fy * y_double_prime + cy;

    // // negates the distortion
    // u = fx*x_prime + cx;
    // v = fy*y_prime + cy;

    ROS_INFO_STREAM("FROM CERES: u: " << u << " v: " << v);

    // residuals
    residuals[0] = u - T(this->observed_pixel_coordinates[0]);
    residuals[1] = v - T(this->observed_pixel_coordinates[1]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction *Create(const double *observed_pixel_coordinates, const double *obj_point_in_target)
  {
    return (new ceres::AutoDiffCostFunction<ReProjectionResidual, 3, CAMERA_INTRINSICS_SIZE, REFERENCE_T_TARGET_SIZE, REFERENCE_T_TARGET_SIZE>(
            new ReProjectionResidual(observed_pixel_coordinates, obj_point_in_target)));
  }

private:
  double observed_pixel_coordinates[2];
  double obj_point_in_target[3];
};


// print vector
template <typename T>
void printVector(std::vector<T> data)
{
  for (auto i = data.begin(); i != data.end(); i++)
  {
    ROS_INFO_STREAM(*i);
  }
  ROS_INFO_STREAM(std::endl);

}

// print std::array
template <typename T, std::size_t N>
void printStdAarray(std::array<T, N> data)
{
  for (auto i = data.begin(); i != data.end(); i++)
  {
    ROS_INFO_STREAM(*i);
  }
  ROS_INFO_STREAM(std::endl);

}

// print array
template <typename T>
void printArray(T array[], int n)
{
  for(int i = 0; i < n; i++)
  {
    ROS_INFO_STREAM(array[i]);
  }
  ROS_INFO_STREAM(std::endl);
}

void printTargets(std::vector<Target> targets)
{
  for(int i = 0; i < targets.size(); i++)
  {
    ROS_INFO_STREAM("targetID: " << i);
    ROS_INFO_STREAM("world_T_target: "); printStdAarray(targets[i].world_T_target);
    ROS_INFO_STREAM("obj_points_in_target: ");
    for(int j = 0; j < targets[i].obj_points_in_target.size(); j++)
    {
      ROS_INFO_STREAM("Corner index: " << j); printStdAarray(targets[i].obj_points_in_target[j]);
    }
  }
}

// gets the list of detection entites (an entity is a directory or file) with the substring "detections"
std::vector<std::string> getDetectionEntityNames(std::string directory_path, unsigned char entity_type)
{
  std::vector<std::string> detection_entities;
  
  DIR *dir = opendir(directory_path.c_str());
  struct dirent *entry = readdir(dir);
  while(entry != NULL)
  {
    if(entry->d_type == entity_type)
    {
      std::string name = entry->d_name;
      if(name.find("detections") != std::string::npos) // a directory
      {
        detection_entities.push_back(name);
      }
    }
    entry = readdir(dir);
  }
  return detection_entities;
}

std::array<double, 6> getReference_T_Target(const YAML::Node &node, std::string reference_T_target_name, bool inverse)
{
  std::array<double, 6> reference_T_target;
  
  if(inverse)
  {
    std::array<double, 3> rodrigues_angle_axis = node[reference_T_target_name]["rotation"].as<std::array<double, 3>>();
    double rotation_array[9];
    ceres::AngleAxisToRotationMatrix(rodrigues_angle_axis.data(), rotation_array);
    Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(rotation_array);
    rotation_matrix = rotation_matrix.inverse().eval();
    ceres::RotationMatrixToAngleAxis(rotation_matrix.data(), rodrigues_angle_axis.data());

    reference_T_target[0] = rodrigues_angle_axis[0];
    reference_T_target[1] = rodrigues_angle_axis[1];
    reference_T_target[2] = rodrigues_angle_axis[2];

    reference_T_target[3] = -1*node[reference_T_target_name]["translation"][0].as<double>();
    reference_T_target[4] = -1*node[reference_T_target_name]["translation"][1].as<double>();
    reference_T_target[5] = -1*node[reference_T_target_name]["translation"][2].as<double>();
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

Detection getDetection(const YAML::Node &detection_node)
{
  Detection detection;
  detection.targetID = detection_node["targetID"].as<int>();
  for(YAML::const_iterator corner_iterator = detection_node["corners"].begin();
      corner_iterator != detection_node["corners"].end();
      ++corner_iterator)
  {
    int corner_index = corner_iterator->first.as<int>();
    detection.corners[corner_index] = corner_iterator->second.as<std::array<double, CORNER_POINTS_SIZE>>();
  }
  return detection;
}

std::vector<Detection> getDetections(const YAML::Node &detections_node)
{
  std::vector<Detection> detections;
  for(YAML::const_iterator detection_iterator = detections_node.begin();
      detection_iterator != detections_node.end();
      ++detection_iterator)
  {
    detections.push_back(getDetection(*detection_iterator));
  }
  return detections;
}

Picture getPicture(std::string picture_file_path)
{
  YAML::Node picture_node = YAML::LoadFile(picture_file_path);
  if(!picture_node)
  {
    ROS_ERROR("Picture file read error.");
  }

  Picture picture;
  picture.world_T_camera = getReference_T_Target(picture_node, "world_T_camera", false);
  picture.camera_T_world = getReference_T_Target(picture_node, "world_T_camera", true);
  picture.detections = getDetections(picture_node["detections"]);
  return picture;
}

std::vector<Picture> getPictures(std::string detections_directory_path)
{
  // get list of detection files
  std::vector<std::string> detection_file_names = getDetectionEntityNames(detections_directory_path, DT_REG);
  // std::sort(detection_file_names.begin(), detection_file_names.end());

  std::vector<Picture> pictures;
  for(auto detection_file_name : detection_file_names)
  {
    // ROS_INFO_STREAM("FILE: " << detection_file_name.c_str());
    pictures.push_back(getPicture(detections_directory_path+"/"+detection_file_name));
  }
  return pictures;
}

void printPictures(std::vector<Picture> pictures)
{
  for(int i = 0; i < pictures.size(); i++)
  {
    ROS_INFO_STREAM("Picture #" << i);
    ROS_INFO("world_T_camera"); printStdAarray(pictures[i].world_T_camera);
    ROS_INFO("camera_T_world"); printStdAarray(pictures[i].camera_T_world);
    for(Detection detection : pictures[i].detections)
    {
      ROS_INFO_STREAM("targetID: " << detection.targetID);
      for(int j = 0; j < detection.corners.size(); j++)
      {
        ROS_INFO_STREAM("Corner #" << j << ": "); printStdAarray(detection.corners[j]);
      }
    }
  }
  ROS_INFO("\n");
}

std::vector<Target> getTargets(std::string targets_directory_path)
{
  YAML::Node targets_node = YAML::LoadFile(targets_directory_path);
  if(!targets_node)
  {
    ROS_ERROR("Targets file read error.");
  }

  std::vector<Target> targets;

  // iterating through all targets
  for(YAML::const_iterator target_iterator = targets_node["targets"].begin();
      target_iterator != targets_node["targets"].end();
      ++target_iterator)
  {    
    const YAML::Node& target_node = *target_iterator;
    Target target;

    // world_T_target
    target.world_T_target = getReference_T_Target(target_node, "world_T_target", false);

    // obj_points_in_target
    for(YAML::const_iterator obj_points_iterator = target_node["obj_points_in_target"].begin();
        obj_points_iterator != target_node["obj_points_in_target"].end();
        ++obj_points_iterator)
    {
      int corner_index = obj_points_iterator->first.as<int>();
      target.obj_points_in_target[corner_index] = obj_points_iterator->second.as<std::array<double, OBJ_POINTS_SIZE>>();
    }

    targets.push_back(target);
  }

  return targets;
}


std::array<double, CAMERA_INTRINSICS_SIZE> getCameraIntrinsics(std::string camera_file_path)
{
  YAML::Node camera_node = YAML::LoadFile(camera_file_path);
  if(!camera_node)
  {
    ROS_ERROR("Camera intrisics file read error.");
  }

  std::array<double, CAMERA_INTRINSICS_SIZE> camera_intrinsics;
  // XXX: hardcoded for the plumb_bob distortion model
  camera_intrinsics[0] = camera_node["camera_matrix"]["data"][0].as<double>();              // fx
  camera_intrinsics[1] = camera_node["camera_matrix"]["data"][4].as<double>();              // fy
  camera_intrinsics[2] = camera_node["camera_matrix"]["data"][2].as<double>();              // cx
  camera_intrinsics[3] = camera_node["camera_matrix"]["data"][5].as<double>();              // cy

  // plumb_bob distortions
  camera_intrinsics[4] = camera_node["distortion_coefficients"]["data"][0].as<double>();    // k1
  camera_intrinsics[5] = camera_node["distortion_coefficients"]["data"][1].as<double>();    // k2
  camera_intrinsics[6] = camera_node["distortion_coefficients"]["data"][2].as<double>();    // p1
  camera_intrinsics[7] = camera_node["distortion_coefficients"]["data"][3].as<double>();    // p2
  camera_intrinsics[8] = camera_node["distortion_coefficients"]["data"][4].as<double>();    // k3
  camera_intrinsics[9] = 0;                                                                 // k4
  camera_intrinsics[10] = 0;                                                                // k5
  camera_intrinsics[11] = 0;                                                                // k6

  return camera_intrinsics;
}


std::string getDetectionsDirectoryPath(ros::NodeHandle &n)
{
  // checking if the detections directory name was passed as a cmd line arg
  std::string detections_directory_name;
  if(!n.getParam("dir_name", detections_directory_name))
  {
    ROS_ERROR("Detections directory name not specified.\n");
    exit(-1);
  }

  // checking existance of detections directory
  std::string package_path = ros::package::getPath("rviz_simulator");
  std::string detections_directory_path = package_path+"/detections/"+detections_directory_name;
  struct stat info;
  if (stat(detections_directory_path.c_str(), &info) != 0)
  {
    ROS_ERROR("Detections directory read error.\n");
    exit(-1);
  }
  
  return detections_directory_path;
}


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "synthetic_optimization");
  ros::NodeHandle n("~");

  std::string detections_directory_path = getDetectionsDirectoryPath(n);

  std::array<double, CAMERA_INTRINSICS_SIZE> camera_intrinsics = getCameraIntrinsics(detections_directory_path+"/camera.yaml");
  std::array<double, CAMERA_INTRINSICS_SIZE> initial_intrinsics = camera_intrinsics;
  // printStdAarray(camera_intrinsics);
  
  std::vector<Target> targets = getTargets(detections_directory_path+"/targets.yaml");
  // printTargets(targets);

  std::vector<Picture> pictures = getPictures(detections_directory_path);
  // printPictures(pictures);

  ceres::Problem problem;

  ceres::CostFunction *cost_function 
          = ReProjectionResidual::Create(
              pictures[30].detections[1].corners[0].data(), 
              targets[pictures[30].detections[1].targetID].obj_points_in_target[0].data());

  problem.AddResidualBlock(
              cost_function, 
              NULL, 
              camera_intrinsics.data(), 
              pictures[30].camera_T_world.data(), 
              targets[pictures[30].detections[1].targetID].world_T_target.data());
  


  // for(int p = 0; p < pictures.size(); p++)
  // {
  //   for(int d = 0; d < pictures[p].detections.size(); d++)
  //   {
  //     for(int c = 0; c < pictures[p].detections[d].corners.size(); c++)
  //     { 
  //       ceres::CostFunction *cost_function 
  //         = ReProjectionResidual::Create(
  //             pictures[p].detections[d].corners[c].data(), 
  //             targets[pictures[p].detections[d].targetID].obj_points_in_target[c].data());

  //       problem.AddResidualBlock(
  //             cost_function, 
  //             NULL, 
  //             camera_intrinsics.data(), 
  //             pictures[p].camera_T_world.data(), 
  //             targets[pictures[p].detections[d].targetID].world_T_target.data());
  //     }
  //   }
  // }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  ROS_INFO_STREAM(summary.FullReport() << "\n");
  std::cout << "\n";

  ROS_INFO_STREAM("INITIAL\t\tFINAL");
  for(int i = 0; i < camera_intrinsics.size(); i++)
  {
    ROS_INFO_STREAM(initial_intrinsics[i] << "\t\t" << camera_intrinsics[i]);
  }

  return 0;

}
