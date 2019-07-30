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

#include "rviz_simulator/camera.h"

namespace rviz_simulator
{
void CameraProperties::populateCameraMatrix()
{
  camera_matrix.resize(3, 4);
  camera_matrix << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0;
  ROS_INFO("Camera matix populated.");
}

Eigen::MatrixXd CameraProperties::getCameraMatrix()
{
  return this->camera_matrix;
}

Camera::Camera(const std::string marker_frame_id, const std::string marker_name,
               const geometry_msgs::Point marker_position_in_ROSWorld,
               const geometry_msgs::Quaternion marker_orientation_in_ROSWorld,
               const std_msgs::ColorRGBA marker_color_RGBA, double marker_scale, double target_scale,
               boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server,
               unsigned int interaction_mode, CameraProperties camera_properties)
  : Target(marker_frame_id, marker_name, marker_position_in_ROSWorld, marker_orientation_in_ROSWorld, marker_color_RGBA,
           marker_scale, g_interactive_marker_server, interaction_mode)
{
  this->camera_properties_ = camera_properties;
  this->camera_properties_.populateCameraMatrix();

  // TODO
  // move target length properties to the target class in order to make this configurable for each target.
  this->target_x_length_ = target_scale;
  this->target_y_length_ = target_scale;
  this->initObjPointsInTarget();

  // adding a shutter release button to the camera
  this->interactive_marker_.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  ROS_INFO("Camera shutter button added.");

  this->makeOutputDirectories();

  this->num_pictures_ = 0;

  this->addCameraToServer();

  this->dumpCameraPropertiesToYAMLFile("camera.yaml");

  this->first_picture_taken_ = false;
}

Camera::~Camera()
{
}

void Camera::initObjPointsInTarget()
{
  Eigen::Vector4d c0, c1, c2, c3;
  c0 << -this->target_x_length_ / 2, -this->target_y_length_ / 2, 0, 1;  // old c0
  c3 << -this->target_x_length_ / 2, this->target_y_length_ / 2, 0, 1;   // old c1
  c2 << this->target_x_length_ / 2, this->target_y_length_ / 2, 0, 1;    // old c2
  c1 << this->target_x_length_ / 2, -this->target_y_length_ / 2, 0, 1;   // old c3

  this->obj_points_in_target_.push_back(c0);
  this->obj_points_in_target_.push_back(c1);
  this->obj_points_in_target_.push_back(c2);
  this->obj_points_in_target_.push_back(c3);

  ROS_INFO("Object points in target calculated.");
}

void Camera::makeOutputDirectories()
{
  int detections_folder_number = int(ros::Time::now().toSec());
  std::string output_folder_name = "detections_" + std::to_string(detections_folder_number);

  std::string package_path = ros::package::getPath("rviz_simulator");
  std::string detections_root_folder = package_path + "/detections";

  // if detections root directory does not exist then create it
  struct stat info;
  if (stat(detections_root_folder.c_str(), &info) != 0)
  {
    ROS_INFO_STREAM("Making directory: " << detections_root_folder);
    if (mkdir(detections_root_folder.c_str(), 0777) == -1)
      ROS_ERROR_STREAM("Error making directory :  " << strerror(errno) << std::endl);
  }

  this->output_folder_path_ = detections_root_folder + "/" + output_folder_name;

  if (mkdir(this->output_folder_path_.c_str(), 0777) == -1)
    ROS_ERROR_STREAM("Error :  " << strerror(errno) << std::endl);

  else
    ROS_INFO_STREAM("Output directory created sucessfully: " << this->output_folder_path_.c_str());
}

void Camera::addCameraToServer()
{
  this->g_interactive_marker_server_->insert(this->interactive_marker_);
  this->g_interactive_marker_server_->setCallback(this->interactive_marker_.name,
                                                  boost::bind(&rviz_simulator::Camera::cameraFeedback, this, _1));
  ROS_INFO_STREAM("Camera added to interactive marker server.");
}

void Camera::cameraFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::ostringstream s;
  s << "Feedback from camera '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
                   << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM(s.str() << ": button click, camera taking picture" << mouse_point_ss.str() << ".");
      this->takePicture();
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM(s.str() << ": pose changed"
                              << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y
                              << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w
                              << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", "
                              << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: "
                              << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
      visualization_msgs::InteractiveMarker temp;
      break;
  }

  this->g_interactive_marker_server_->applyChanges();
}

Eigen::Affine3d Camera::calculateReference_T_Target(visualization_msgs::InteractiveMarker& reference,
                                                    visualization_msgs::InteractiveMarker& target)
{
  Eigen::Affine3d origin_T_reference, origin_T_target;
  tf::poseMsgToEigen(reference.pose, origin_T_reference);
  tf::poseMsgToEigen(target.pose, origin_T_target);
  return origin_T_reference.inverse() * origin_T_target;
}

void Camera::takePicture()
{
  if (!this->first_picture_taken_)
  {
    this->calculateWorld_T_Targets("targets.yaml");
    this->first_picture_taken_ = true;
  }

  // YAML Emitter for the detections_X.yaml file
  YAML::Emitter detections_out;
  detections_out << YAML::BeginMap;  // level 0

  // calculating world_T_camera and dumping to YAML::Emitter
  visualization_msgs::InteractiveMarker camera_marker;
  this->g_interactive_marker_server_->get(this->marker_name_, camera_marker);

  visualization_msgs::InteractiveMarker world_marker;
  this->g_interactive_marker_server_->get("tag0", world_marker);

  Eigen::Affine3d world_T_camera = this->calculateReference_T_Target(world_marker, camera_marker);
  this->Affine3dToYaml(world_T_camera, "world_T_camera", detections_out);

  detections_out << YAML::Key << "detections";
  detections_out << YAML::Value << YAML::BeginSeq;  // detections sequence

  // processing all camera_T_target transforms
  for (int i = 0; i < this->world_T_targets_.size(); i++)
  {
    // getting camera_T_target
    Eigen::Affine3d camera_T_target = world_T_camera.inverse() * this->world_T_targets_[i];

    // check if target is in front of camera and camera is in front of target and target is in range
    // if yes then process corners
    ROS_INFO_STREAM("Processing targetID: " << i);
    if (this->isInFrontOfCamera(camera_T_target) && this->isInFrontOfTarget(camera_T_target.inverse()))
    {
      std::vector<Eigen::Vector2d> corners = processCorners(camera_T_target);

      // output to yaml file
      if (!corners.empty())
      {
        detections_out << YAML::BeginMap;  // target
        detections_out << YAML::Key << "targetID" << YAML::Value << std::to_string(i);
        std::vector<double> size = { this->target_x_length_, this->target_y_length_ };
        detections_out << YAML::Key << "size" << YAML::Value << YAML::Flow << size;
        detections_out << YAML::Key << "corners";
        detections_out << YAML::Value << YAML::BeginMap;  // corners
        for (int i = 0; i < corners.size(); i++)
        {
          std::vector<double> corner = { corners[i].x(), corners[i].y() };
          detections_out << YAML::Key << i;
          detections_out << YAML::Value << YAML::Flow << corner;
        }
        detections_out << YAML::EndMap;  // corners
        detections_out << YAML::EndMap;  // target
      }
    }
  }                                // end processing all camera_T_target transforms
  detections_out << YAML::EndSeq;  // detections sequence
  detections_out << YAML::EndMap;  // level 0

  // outputting to .yaml file
  std::string output_file_name = "detections_" + std::to_string(this->num_pictures_) + ".yaml";
  std::string output_file_path = this->output_folder_path_ + "/" + output_file_name;
  std::ofstream fout(output_file_path);
  if (!fout)
    ROS_ERROR_STREAM("Output file error.\n" << output_file_path << "\n");
  else
  {
    fout << detections_out.c_str();
    ROS_INFO_STREAM("Corner detections dumped to " << output_file_path << "\n");
    fout.close();
    this->num_pictures_++;
  }
}

std::vector<Eigen::Vector2d> Camera::processCorners(Eigen::Affine3d camera_T_target)
{
  std::vector<Eigen::Vector2d> corners2d;

  // get corners
  for (int i = 0; i < this->obj_points_in_target_.size(); i++)
  {
    Eigen::Vector2d corner = calculatePixelCoords(camera_T_target, this->obj_points_in_target_[i]);
    ROS_INFO_STREAM("Corner " << i << " u: " << corner[0] << " v: " << corner[1] << std::endl);
    if (!this->isWithinImage(corner))
      return std::vector<Eigen::Vector2d>();  // return empty vector
    corners2d.push_back(corner);
  }

  // check if corners are not too close (6 distance checks)
  if (!this->isInRange(corners2d))
    return std::vector<Eigen::Vector2d>();  // return empty vector

  ROS_INFO_STREAM("All corners valid.\n");
  return corners2d;
}

Eigen::Vector2d Camera::calculatePixelCoords(Eigen::Affine3d camera_T_target, Eigen::Vector4d point_in_target)
{
  Eigen::Vector4d point_in_camera = camera_T_target * point_in_target;

  // scaling
  double x_prime = (point_in_camera[0] / point_in_camera[2]);
  double x_prime_squared = x_prime * x_prime;
  double y_prime = (point_in_camera[1] / point_in_camera[2]);
  double y_prime_squared = y_prime * y_prime;

  // rectifying the camera distortion
  double r_raise_2 = x_prime_squared + y_prime_squared;
  double r_raise_4 = r_raise_2 * r_raise_2;
  double r_raise_6 = r_raise_4 * r_raise_2;

  double numerator =
      1 + camera_properties_.k1 * r_raise_2 + camera_properties_.k2 * r_raise_4 + camera_properties_.k3 * r_raise_6;
  double denominator =
      1 + camera_properties_.k4 * r_raise_2 + camera_properties_.k5 * r_raise_4 + camera_properties_.k6 * r_raise_6;
  double coefficient = numerator / denominator;

  double x_double_prime = x_prime * coefficient + 2 * camera_properties_.p1 * x_prime * y_prime +
                          camera_properties_.p2 * (r_raise_2 + 2 * x_prime_squared);
  double y_double_prime = y_prime * coefficient + camera_properties_.p1 * (r_raise_2 + 2 * y_prime_squared) +
                          2 * camera_properties_.p2 * x_prime * y_prime;

  double u = camera_properties_.fx * x_double_prime + camera_properties_.cx;
  double v = camera_properties_.fy * y_double_prime + camera_properties_.cy;

  Eigen::Vector2d pixelCoordinates;
  pixelCoordinates[0] = u;
  pixelCoordinates[1] = v;

  return pixelCoordinates;
}

bool Camera::isWithinImage(Eigen::Vector2d pixelCoords)
{
  if (pixelCoords.x() >= 0 && pixelCoords.x() < this->camera_properties_.image_width && pixelCoords.y() >= 0 &&
      pixelCoords.y() < this->camera_properties_.image_height)
  {
    ROS_INFO_STREAM("Corner is within image\n");
    return true;
  }
  ROS_INFO_STREAM("Corner is outside image\n");
  return false;
}

bool Camera::isInFrontOfCamera(Eigen::Affine3d camera_T_target)
{
  if (camera_T_target.translation().z() > 0)
  {
    ROS_INFO_STREAM("Target in front of camera\n");
    return true;
  }
  ROS_INFO_STREAM("Target behind camera\n");
  return false;
}

bool Camera::isInFrontOfTarget(Eigen::Affine3d target_T_camera)
{
  if (target_T_camera.translation().z() > 0)
  {
    ROS_INFO_STREAM("Camera in front of target\n");
    return true;
  }
  ROS_INFO_STREAM("Camera behind target\n");
  return false;
}

bool Camera::isInRange(std::vector<Eigen::Vector2d> corners)
{
  std::vector<Eigen::Vector2d> differences;
  differences.push_back(corners[0] - corners[1]);
  differences.push_back(corners[1] - corners[2]);
  differences.push_back(corners[2] - corners[3]);
  differences.push_back(corners[3] - corners[0]);
  differences.push_back(corners[1] - corners[3]);
  differences.push_back(corners[0] - corners[2]);

  for (int i = 0; i < differences.size(); i++)
  {
    if (differences[i].norm() < this->camera_properties_.min_corner_dist)
    {
      ROS_INFO_STREAM("Target not in range\n.");
      return false;
    }
  }
  ROS_INFO_STREAM("Target in range\n.");
  return true;
}

void Camera::dumpCameraPropertiesToYAMLFile(std::string output_file_name)
{
  YAML::Emitter out;
  out << YAML::BeginMap;  // level 0 map

  out << YAML::Key << "image_width" << YAML::Value << this->camera_properties_.image_width;
  out << YAML::Key << "image_height" << YAML::Value << this->camera_properties_.image_height;
  out << YAML::Key << "camera_name" << YAML::Value << this->camera_properties_.camera_name;

  // camera matrix
  out << YAML::Key << "camera_matrix" << YAML::Value;
  out << YAML::BeginMap;  // camera matrix map
  out << YAML::Key << "rows" << YAML::Value << 3;
  out << YAML::Key << "cols" << YAML::Value << 3;

  // RowMajor
  std::vector<double> camera_matrix_data = {
    camera_properties_.fx, 0, camera_properties_.cx, 0, camera_properties_.fy, camera_properties_.cy, 0, 0, 1
  };

  out << YAML::Key << "data" << YAML::Value << YAML::Flow << camera_matrix_data;
  out << YAML::EndMap;  // camera matrix map

  // distortions
  out << YAML::Key << "distortion_model" << YAML::Value << camera_properties_.distortion_model;
  out << YAML::Key << "distortion_coefficients" << YAML::Value;
  out << YAML::BeginMap;  // distortion coefficitents map
  out << YAML::Key << "rows" << YAML::Value << 1;
  out << YAML::Key << "cols" << YAML::Value << 5;

  std::vector<double> distortion_coefficients = { camera_properties_.k1, camera_properties_.k2, camera_properties_.p1,
                                                  camera_properties_.p2, camera_properties_.k3 };

  out << YAML::Key << "data" << YAML::Value << YAML::Flow << distortion_coefficients;
  out << YAML::EndMap;  // distortion coefficitents map

  // rectification matrix (omitted)

  // projection matrix (omitted)

  out << YAML::EndMap;  // level 0 map

  // dumping to YAML file
  std::string camera_file_path = this->output_folder_path_ + "/" + output_file_name;
  std::ofstream fout(camera_file_path);
  if (!fout)
  {
    ROS_ERROR_STREAM("Output file error.\n" << camera_file_path << "\n");
  }
  else
  {
    fout << out.c_str();
    ROS_INFO_STREAM("Camera properties dumped to " << camera_file_path);
  }
  fout.close();
}

void Camera::calculateWorld_T_Targets(std::string output_file_name)
{
  // YAML Emitter for the targets.yaml file
  YAML::Emitter targets_out;
  targets_out << YAML::BeginMap;  // level 0
  targets_out << YAML::Key << "targets";
  targets_out << YAML::Value << YAML::BeginSeq;  // targets sequence

  // considering tag0 as the world origin
  visualization_msgs::InteractiveMarker world_marker;
  this->g_interactive_marker_server_->get("tag0", world_marker);

  int num_targets = this->g_interactive_marker_server_->size() - 1;  // -1 to cater for the camera marker

  for (int i = 0; i < num_targets; i++)
  {
    // getting world_T_target
    std::string target_name = "tag" + std::to_string(i);
    ROS_INFO_STREAM("Calculated and stored targetID: " << target_name << std::endl);
    visualization_msgs::InteractiveMarker target_marker;
    this->g_interactive_marker_server_->get(target_name, target_marker);
    Eigen::Affine3d world_T_target = this->calculateReference_T_Target(world_marker, target_marker);
    this->world_T_targets_.push_back(world_T_target);

    // yaml output
    targets_out << YAML::BeginMap;
    targets_out << YAML::Key << "targetID" << YAML::Value << i;
    this->Affine3dToYaml(world_T_target, "world_T_target", targets_out);
    this->ObjPointsInTargetToYAML(targets_out);
    targets_out << YAML::EndMap;
  }
  targets_out << YAML::EndMap;  // level 0

  // outputting to .yaml file
  std::string output_file_path = this->output_folder_path_ + "/" + output_file_name;
  std::ofstream fout(output_file_path);
  if (!fout)
    ROS_ERROR_STREAM("Output file error.\n" << output_file_path << "\n");
  else
  {
    fout << targets_out.c_str();
    ROS_INFO_STREAM("Target data dumped to " << output_file_path << "\n");
    ROS_WARN("Do not move the targets in rviz from this point!\n");
    fout.close();
  }
}

std::vector<double> Camera::Affine3dRotationToRodrigues(Eigen::Affine3d affine3d)
{
  double rodriguesArray[3];
  ceres::RotationMatrixToAngleAxis(affine3d.rotation().data(), rodriguesArray);
  std::vector<double> rodriguesVector(rodriguesArray, rodriguesArray + 3);
  return rodriguesVector;
}

std::vector<double> Camera::Affine3dToTranslation(Eigen::Affine3d affine3d)
{
  std::vector<double> translation(affine3d.translation().data(), affine3d.translation().data() + 3);
  return translation;
}

void Camera::Affine3dToYaml(Eigen::Affine3d affine3d, std::string name, YAML::Emitter& out)
{
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "rotation" << YAML::Value << YAML::Flow << this->Affine3dRotationToRodrigues(affine3d);
  out << YAML::Key << "translation" << YAML::Value << YAML::Flow << this->Affine3dToTranslation(affine3d);
  out << YAML::EndMap;
}

void Camera::ObjPointsInTargetToYAML(YAML::Emitter& out)
{
  out << YAML::Key << "obj_points_in_target" << YAML::Value << YAML::BeginMap;
  for (int i = 0; i < this->obj_points_in_target_.size(); i++)
  {
    Eigen::Vector4d point = this->obj_points_in_target_[i];
    std::vector<double> p = { point.x(), point.y(), point.z() };
    out << YAML::Key << i << YAML::Value << YAML::Flow << p;
  }
  out << YAML::EndMap;
}
}
