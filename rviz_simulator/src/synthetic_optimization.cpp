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
 * *** DELETE THIS BEFORE MERGING ***
 * 
 * http://www.lloydhughes.co.za/index.php/using-eigen-quaternions-and-ceres-solver/
 * 
 * TODO/NOTES:
 * refector milestone 1 camera code to be used in milestone 3
 * 
 * 
 * templating the YAML-cpp readers to read automatically/refactoring the save methods
 * 
 * exception handling
 * 
 */

#include <ros/ros.h>
#include <ros/package.h>

#include "ceres/ceres.h"

#include <yaml-cpp/yaml.h>

// getting a directory/file listing
#include <dirent.h>
// #include <filesystem>  // only seems to work for c++17

#include "ceres/rotation.h"

#include <Eigen/Dense>


struct ReProjectionResidual
{
  // Constructor
  ReProjectionResidual(const double *observed_pixel_coordinates, const double *point_in_ROSWorld)
  {
    // projected image pixel coordinates (u,v)
    this->observed_pixel_coordinates[0] = observed_pixel_coordinates[0];
    this->observed_pixel_coordinates[1] = observed_pixel_coordinates[1];

    // 3D coordinates in ROSWorld with tag0 as origin (X, Y, Z)
    this->point_in_ROSWorld[0] = point_in_ROSWorld[0];
    this->point_in_ROSWorld[1] = point_in_ROSWorld[1];
    this->point_in_ROSWorld[2] = point_in_ROSWorld[2];
  }

  template <typename T>
  bool operator()(const T* const camera_extrinsics, const T* const intrinsics, T* residuals) 
  const
  {
    // for(int i = 0; i < 7; i++)
    //   ROS_INFO_STREAM("FROM CERES" << camera_extrinsics[i]);
    // ROS_INFO_STREAM(std::endl);

    // mapping point in ROSWorld 
    // Eigen::Matrix<T, 4, 1> point;
    // point << T(this->point_in_ROSWorld[0]) , T(this->point_in_ROSWorld[1]) , T(this->point_in_ROSWorld[2]) , T(1);


    // // mapping world_T_camera
    // Eigen::Matrix<T, 4, 4> world_T_camera_transform;
    // world_T_camera_transform = Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>>(world_T_camera);

    // ROS_INFO_STREAM("FROM CERES" << (world_T_camera[0]));
    // // for(int i = 0; i < 16; i++)
    // //   ROS_INFO_STREAM(*(world_T_camera+i));

    // // applying rotation and translation
    // Eigen::Matrix<T, 4, 1> p;
    // p = world_T_camera_transform.inverse() * point;

    // camera_extrinsics
    // [0, 1, 2, 3] = [w, i, j, k]
    
    // [4, 5, 6] = [x, y, z]
    
    
    const T quarternion[4] = 
    {
      T(camera_extrinsics[0]),
      T(camera_extrinsics[1]),
      T(camera_extrinsics[2]),
      T(camera_extrinsics[3])
    };
    T axis_angle[3];
    ceres::QuaternionToAngleAxis(quarternion, axis_angle);

    const T point[3] = 
    {
      T(this->point_in_ROSWorld[0]),
      T(this->point_in_ROSWorld[1]),
      T(this->point_in_ROSWorld[2])
    };
    T p[3];
    
    // ROS_INFO_STREAM("CERES: x3_prime: " << point[0] << "y3_prime: " << point[1] << "z3_prime: " << point[2]);

    // ceres::QuaternionRotatePoint(quarternion, point, p);
    ceres::AngleAxisRotatePoint(axis_angle, point, p);
    // ROS_INFO_STREAM("CERES: p0: " << p[0] << "p1: " << p[1] << "p2: " << p[2]);

    p[0] += camera_extrinsics[4];
    p[1] += camera_extrinsics[5];
    p[2] += camera_extrinsics[6];

    // scaling
    T x_prime = p[0] / p[2];
    T x_prime_squared = x_prime * x_prime;
    T y_prime = p[1] / p[2];
    T y_prime_squared = y_prime * y_prime;

    // ROS_INFO_STREAM("CERES: x_prime: " << x_prime << "y_prime: " << y_prime);

    // rectifying the camera distortion
    T fx = intrinsics[0];
    T fy = intrinsics[1];
    T cx = intrinsics[2];
    T cy = intrinsics[3];
    T k1 = intrinsics[4];
    T k2 = intrinsics[5];
    T k3 = intrinsics[6];
    T k4 = intrinsics[7];
    T k5 = intrinsics[8];
    T k6 = intrinsics[9];
    T p1 = intrinsics[10];
    T p2 = intrinsics[11];  

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

    // negates the distortion
    u = fx*x_prime + cx;
    v = fy*y_prime + cy;

    // ROS_INFO_STREAM("FROM CERES: u: " << u << " v: " << v);

    // residuals
    residuals[0] = u - T(this->observed_pixel_coordinates[0]);
    residuals[1] = v - T(this->observed_pixel_coordinates[1]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction *Create(const double *observed_pixel_coordinates, const double *point_in_ROSWorld)
  {
    return (new ceres::AutoDiffCostFunction<ReProjectionResidual, 2, 7, 12>(
            new ReProjectionResidual(observed_pixel_coordinates, point_in_ROSWorld)));
  }

private:
  double observed_pixel_coordinates[2];
  double point_in_ROSWorld[3];
};


// print vector
template <typename T>
void print_vector(std::vector<T> data)
{
  for (auto i = data.begin(); i != data.end(); i++)
  {
    ROS_INFO_STREAM(*i);
  }
  ROS_INFO_STREAM(std::endl);

}


// gets the list of directories with the substring "detections"
std::vector<std::string> get_detections(std::string package_path, unsigned char entity_type)
{
  std::vector<std::string> detection_entities;
  
  DIR *dir = opendir(package_path.c_str());
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


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "synthetic_optimization");
  ros::NodeHandle n;

  ceres::Problem problem;

  std::string package_path = ros::package::getPath("rviz_simulator");

  // getting camera intrinsics
  YAML::Node config_node = YAML::LoadFile(package_path+"/config/initialize_simulator.yaml");
  std::string camera_intrinsics_file = config_node["camera_intrinsics_file"].as<std::string>();
  YAML::Node camera_node = YAML::LoadFile(package_path+"/config/"+camera_intrinsics_file);

  const unsigned int NUM_INTRINSICS = 12;
  std::array<double, NUM_INTRINSICS> intrinsics = 
  {
    camera_node["fx"].as<double>(),
    camera_node["fy"].as<double>(),
    camera_node["cx"].as<double>(),
    camera_node["cy"].as<double>(),
    camera_node["k1"].as<double>(),
    camera_node["k2"].as<double>(),
    camera_node["k3"].as<double>(),
    camera_node["k4"].as<double>(),
    camera_node["k5"].as<double>(),
    camera_node["k6"].as<double>(),
    camera_node["p1"].as<double>(),
    camera_node["p2"].as<double>(),
  };
  std::array<double, NUM_INTRINSICS> initial_intrinsics = intrinsics;   // to compare with later

  // get fiducial target size
  const double TARGET_SCALE = config_node["target_scale"].as<double>();

  // getting extrinsics, observed_pixel_points, points_in_ROSWorld
  std::vector<std::array<double, 7>> camera_extrinsics;  // vector of vectors (16 elements)
  camera_extrinsics.reserve(1000); // ensures the vector memory addresses are not reallocated durind execution


  // std::string file_path = package_path+"/detections_1562784215/detections_0.yaml";

  // getting vector of detection directories
  std::vector<std::string> detection_directories = get_detections(package_path, DT_DIR);
  // print_vector(detection_directories);

  // iterating through the detection directories
  for(auto detection_directory = detection_directories.begin(); detection_directory != detection_directories.end(); ++detection_directory)
  {
    // iterating through the detection files in given detection directory
    std::vector<std::string> detection_file_names = get_detections(package_path+"/"+*detection_directory, DT_REG);
    for(auto detection_file_name = detection_file_names.begin(); 
        detection_file_name != detection_file_names.end();
        ++detection_file_name)
    {
      std::string detection_file_path = package_path+"/"+*detection_directory+"/"+*detection_file_name;
      ROS_INFO_STREAM(detection_file_path);

      // load yaml file
      YAML::Node detection_file_node = YAML::LoadFile(detection_file_path);

      // reading camera extrinsics
      std::vector<double> camera_pose = detection_file_node["camera"]["pose"].as<std::vector<double>>();
      // std::array<double, 16> camera_pose_array;
      // std::copy_n(camera_pose.begin(), 16, camera_pose_array.begin());
      // world_T_camera.push_back(camera_pose_array);

      // double *array = &camera_pose_array[0];
      // ROS_INFO_STREAM("BACK " << array[0]);

      // transform conversion
      Eigen::Matrix<double, 4, 4> world_T_camera_matrix;
      world_T_camera_matrix = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(camera_pose.data());
      Eigen::Affine3d world_T_camera_affine3d;
      world_T_camera_affine3d = world_T_camera_matrix;

      // Eigen::Matrix3d world_T_camera_rotation_matrix;
      // world_T_camera_rotation_matrix = world_T_camera_affine3d.rotation();
      // Eigen::Quaterniond world_T_camera_quarternion;
      // world_T_camera_quarternion = world_T_camera_rotation_matrix;
      // Eigen::Quaterniond camera_T_world_quarternion;
      // camera_T_world_quarternion = world_T_camera_quarternion.inverse();
      // camera_T_world_quarternion.normalize();

      // Eigen::Vector3d world_T_camera_translation;
      // world_T_camera_translation = world_T_camera_affine3d.translation();
      // Eigen::Vector3d camera_T_world_translation;
      // camera_T_world_translation = -world_T_camera_translation;

      // std::array<double, 7> camera_pose_array;
      // camera_pose_array[0] = camera_T_world_quarternion.w();
      // camera_pose_array[1] = camera_T_world_quarternion.x();
      // camera_pose_array[2] = camera_T_world_quarternion.y();
      // camera_pose_array[3] = camera_T_world_quarternion.z();
      // camera_pose_array[4] = camera_T_world_translation.x();
      // camera_pose_array[5] = camera_T_world_translation.y();
      // camera_pose_array[6] = camera_T_world_translation.z();

      // camera_extrinsics.push_back(camera_pose_array);
      // double *array = camera_extrinsics.back().data();
      // for(int i = 0; i < 7; i++)
      //   ROS_INFO_STREAM("EXTRINSICS: " << array[i]);

      // for(int i = 0; i < 7; i++)
      //   ROS_INFO_STREAM("FROM ROS" << camera_pose_array[i]);   

      // print_vector(camera_pose);
      // for(int i = 0; i < camera_pose_array.size(); i++)
      // ROS_INFO_STREAM(&camera_pose_array[0]);

      // iterating through targets in the detection file
      for(YAML::const_iterator target = detection_file_node["detections"].begin(); 
          target != detection_file_node["detections"].end(); 
          ++target)
      {
        const YAML::Node& target_node = *target;
        std::string target_id = target_node["TargetID"].as<std::string>();
        ROS_INFO_STREAM("target id: " << target_id);

        // get target pose in ROSWorld (world_T_target)
        std::vector<double> target_pose = target_node["pose"].as<std::vector<double>>();
        Eigen::Matrix4d world_T_target_matrix;
        world_T_target_matrix = Eigen::Map<Eigen::Matrix<double,4, 4, Eigen::RowMajor> >(target_pose.data());
        Eigen::Affine3d world_T_target_affine3d;
        world_T_target_affine3d = world_T_target_matrix;

        Eigen::Affine3d camera_T_target_affine3d;
        camera_T_target_affine3d = world_T_camera_affine3d.inverse()*world_T_target_affine3d;

        Eigen::Matrix3d camera_T_target_rotation_matrix;
        camera_T_target_rotation_matrix = camera_T_target_affine3d.rotation();
        Eigen::Quaterniond camera_T_target_quarternion;
        camera_T_target_quarternion = camera_T_target_rotation_matrix;
        // Eigen::Quaterniond camera_T_world_quarternion;
        // camera_T_world_quarternion = camera_T_target_quarternion.inverse();
        // camera_T_world_quarternion.normalize();

        Eigen::Vector3d camera_T_target_translation;
        camera_T_target_translation = camera_T_target_affine3d.translation();
        Eigen::Vector3d camera_T_world_translation;
        camera_T_world_translation = camera_T_target_translation;


        std::array<double, 7> camera_pose_array;
        camera_pose_array[0] = camera_T_target_quarternion.w();
        camera_pose_array[1] = camera_T_target_quarternion.x();
        camera_pose_array[2] = camera_T_target_quarternion.y();
        camera_pose_array[3] = camera_T_target_quarternion.z();
        camera_pose_array[4] = camera_T_target_translation.x();
        camera_pose_array[5] = camera_T_target_translation.y();
        camera_pose_array[6] = camera_T_target_translation.z();

        camera_extrinsics.push_back(camera_pose_array);

        for(int i = 0; i < 7; i++)
          ROS_INFO_STREAM("FROM ROS" << camera_pose_array[i]);   
        
        // set vector of corner transforms
        // *** CODE FROM CAMERA.CPP  ***
        Eigen::Vector4d c1, c2, c3, c4;
        c1 << 0, 0, 0, 1;
        c2 << TARGET_SCALE, 0, 0, 1;
        c3 << TARGET_SCALE, 0, TARGET_SCALE, 1;
        c4 << 0, 0, TARGET_SCALE, 1;
        std::vector<Eigen::Vector4d> corners4d;
        corners4d.push_back(c1);
        corners4d.push_back(c2);
        corners4d.push_back(c3);
        corners4d.push_back(c4);
        // *** CODE FROM CAMERA.CPP  ***


        // iterating through the 4 corners of a target
        int i = 0;
        for(YAML::const_iterator corner = target_node["corners"].begin(); corner != target_node["corners"].end(); ++corner)
        {
          int corner_id = corner->first.as<int>();
          const YAML::Node& coordinates_node = corner->second;
          double u = coordinates_node[0].as<double>();
          double v = coordinates_node[1].as<double>();
          double observed_pixel_coordinates[2] = {u, v};
          ROS_INFO_STREAM("u: " << u << ", v: " << v);

          double point_in_target_frame[3] = 
          {
            corners4d[i][0],
            corners4d[i][1],
            corners4d[i][2],
          };

          ROS_INFO_STREAM("ROS POINT: x: " << point_in_target_frame[0] << " y: " << point_in_target_frame[1] << " z: " << point_in_target_frame[2]);

          // add residual block
          ceres::CostFunction* cost_function = ReProjectionResidual::Create(observed_pixel_coordinates, point_in_target_frame);
          problem.AddResidualBlock(cost_function, NULL, camera_extrinsics.back().data(), intrinsics.data());

          i++;
        }
        // ceres::LocalParameterization *quaternion_parameterization = new ceres::QuaternionParameterization;
        // problem.SetParameterization()
      } // end iterating through targets in the detection file
    } // end iterating through the detection files in given detection directory
  } // end iterating through the detection directories





  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  ROS_INFO_STREAM(summary.FullReport() << "\n");
  std::cout << "\n";

  ROS_INFO_STREAM("INITIAL\t\tFINAL");
  for(int i = 0; i < intrinsics.size(); i++)
  {
    ROS_INFO_STREAM(initial_intrinsics[i] << "\t\t" << intrinsics[i]);
  }

  return 0;

}