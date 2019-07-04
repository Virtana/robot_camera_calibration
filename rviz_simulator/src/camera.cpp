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
Camera::Camera(const std::string marker_frame_id, const std::string marker_name,
               const geometry_msgs::Point marker_position_in_ROSWorld,
               const geometry_msgs::Quaternion marker_orientation_in_ROSWorld,
               const std_msgs::ColorRGBA marker_color_RGBA, double marker_scale,
               boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server,
               unsigned int interaction_mode, CameraIntrinsics camera_intrinsics, CameraDistortions camera_distortions)
  : Target(marker_frame_id, marker_name, marker_position_in_ROSWorld, marker_orientation_in_ROSWorld, marker_color_RGBA,
           marker_scale, g_interactive_marker_server, interaction_mode)
{
  // setting camera intrinsics
  this->camera_intrinsics_ = camera_intrinsics;
  this->camera_intrisics_matrix_.resize(3, 4);
  this->camera_intrisics_matrix_ << this->camera_intrinsics_.fx, 0, this->camera_intrinsics_.cx, 0, 0,
      this->camera_intrinsics_.fy, this->camera_intrinsics_.cy, 0, 0, 0, 1, 0;

  this->camera_distortions_ = camera_distortions;

  this->length_of_target_ = marker_scale;

  this->addShutterReleaseButton();

  // create ouput directory for yaml files
  int detections_folder_number_ = int(ros::Time::now().toSec());
  this->detections_file_number_ = 0;
  this->output_folder_name_ = "detections_" + std::to_string(detections_folder_number_);

  std::string package_path = ros::package::getPath("rviz_simulator");
  this->output_folder_path_ = package_path + "/" + this->output_folder_name_;

  if (mkdir(this->output_folder_path_.c_str(), 0777) == -1)
    ROS_ERROR_STREAM("Error :  " << strerror(errno) << std::endl);

  else
    ROS_INFO_STREAM("Output directory created sucessfully: " << this->output_folder_path_.c_str() << std::endl);

  // add camera interactive marker to the global interactive marker server
  this->addCameraToServer();
}

Camera::~Camera()
{
}

void Camera::addCameraToServer()
{
  this->g_interactive_marker_server_->insert(this->interactive_marker_);
  this->g_interactive_marker_server_->setCallback(this->interactive_marker_.name,
                                                  boost::bind(&rviz_simulator::Camera::cameraFeedback, this, _1));
  ROS_INFO_STREAM("Camera added\n");
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

void Camera::addShutterReleaseButton()
{
  this->interactive_marker_.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
}

Eigen::Affine3d Camera::getTransformBetweenInteractiveMarkers(visualization_msgs::InteractiveMarker& reference,
                                                              visualization_msgs::InteractiveMarker& target)
{
  Eigen::Affine3d reference_pose, target_pose;
  tf::poseMsgToEigen(reference.pose, reference_pose);
  tf::poseMsgToEigen(target.pose, target_pose);
  return reference_pose.inverse() * target_pose;
}

void transformToYAMLSequence(Eigen::Affine3d& transform, YAML::Emitter& out)
{
  for (int i = 0; i < transform.Dim + 1; i++)
  {
    for (int j = 0; j < transform.Dim + 1; j++)
    {
      out << transform(i, j);
    }
  }
}

void Camera::takePicture()
{
  YAML::Emitter out;
  out << YAML::BeginMap;  // level 0
  out << YAML::Key << "detections";
  out << YAML::Value << YAML::BeginSeq;  // level 1

  // considering tag0 as the world origin
  visualization_msgs::InteractiveMarker world_marker, camera_marker;
  this->g_interactive_marker_server_->get("tag0", world_marker);

  int num_targets = this->g_interactive_marker_server_->size() - 1;  // -1 to cater for the camera marker

  // processing all camera_T_target transforms
  for (int i = 0; i < num_targets; i++)
  {
    // getting camera extrinsics (camera_T_target)
    std::string target_name = "tag" + std::to_string(i);
    ROS_INFO_STREAM("Target name: " << target_name << std::endl);
    visualization_msgs::InteractiveMarker target_marker;
    this->g_interactive_marker_server_->get(target_name, target_marker);
    this->g_interactive_marker_server_->get(this->marker_name_, camera_marker);
    Eigen::Affine3d camera_T_target = this->getTransformBetweenInteractiveMarkers(camera_marker, target_marker);

    // check if target is in front of camera and camera is in front of target and target is in range
    // if yes then process corners
    if (this->isInFrontOfCamera(camera_T_target) && this->isInFrontOfTarget(camera_T_target.inverse()) &&
        this->isInRange(camera_T_target))
    {
      std::vector<Eigen::Vector2d> corners = processCorners(camera_T_target);

      if (!corners.empty())
      {
        out << YAML::BeginMap;
        out << YAML::Key << "TargetID" << YAML::Value << std::to_string(i);
        out << YAML::Key << "corners";
        out << YAML::Value << YAML::BeginMap;
        for (int i = 0; i < corners.size(); i++)
        {
          out << YAML::Key << i;
          out << YAML::Value << YAML::BeginSeq << int(corners[i][0]) << int(corners[i][1]) << YAML::EndSeq;
        }
        out << YAML::EndMap;

        // getting world_T_target
        // outputting world_T_target to YAML row_wise
        Eigen::Affine3d world_T_target = this->getTransformBetweenInteractiveMarkers(world_marker, target_marker);
        out << YAML::Key << "pose" << YAML::Value << YAML::BeginSeq;
        transformToYAMLSequence(world_T_target, out);
        out << YAML::EndSeq;
        out << YAML::EndMap;
      }
    }
  }                     // end processing all camera_T_target transforms
  out << YAML::EndSeq;  // level 1

  // publishing camera pose
  Eigen::Affine3d world_T_camera = this->getTransformBetweenInteractiveMarkers(world_marker, camera_marker);

  out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;  // level 0
  out << YAML::Key << "pose" << YAML::Value << YAML::BeginSeq;
  transformToYAMLSequence(world_T_camera, out);
  out << YAML::EndSeq;

  // publishing camera intrinsics
  out << YAML::Key << "intrinsics" << YAML::Value << YAML::BeginMap;  // level 1
  out << YAML::Key << "fx" << YAML::Value << this->camera_intrinsics_.fx;
  out << YAML::Key << "fy" << YAML::Value << this->camera_intrinsics_.fy;
  out << YAML::Key << "cx" << YAML::Value << this->camera_intrinsics_.cx;
  out << YAML::Key << "cy" << YAML::Value << this->camera_intrinsics_.cy;
  out << YAML::Key << "width" << YAML::Value << this->camera_intrinsics_.image_width;
  out << YAML::Key << "height" << YAML::Value << this->camera_intrinsics_.image_height;
  out << YAML::Key << "near_clip" << YAML::Value << this->camera_intrinsics_.min_distance_between_camera_and_target;
  out << YAML::Key << "far_clip" << YAML::Value << this->camera_intrinsics_.max_distance_between_camera_and_target;
  out << YAML::EndMap;  // level 1

  out << YAML::EndMap;  // level 0

  out << YAML::EndMap;  // level 0

  // outputting to .yaml file
  std::string output_file_name = "detections_" + std::to_string(this->detections_file_number_) + ".yaml";
  std::string output_file_path = this->output_folder_path_ + "/" + output_file_name;
  std::ofstream fout(output_file_path);
  if (!fout)
    ROS_ERROR_STREAM("Output file error.\n" << output_file_path << "\n");
  else
  {
    fout << out.c_str();
    ROS_INFO_STREAM("Target corner detections dumped to " << output_file_path << "\n");
    fout.close();
    this->detections_file_number_++;
  }
}

std::vector<Eigen::Vector2d> Camera::processCorners(Eigen::Affine3d camera_extrinsics)
{
  Eigen::Vector4d c1, c2, c3, c4;
  c1 << 0, 0, 0, 1;
  c2 << this->length_of_target_, 0, 0, 1;
  c3 << this->length_of_target_, 0, this->length_of_target_, 1;
  c4 << 0, 0, this->length_of_target_, 1;
  std::vector<Eigen::Vector4d> corners4d;
  corners4d.push_back(c1);
  corners4d.push_back(c2);
  corners4d.push_back(c3);
  corners4d.push_back(c4);

  std::vector<Eigen::Vector2d> corners2d;

  // get corners
  for (int i = 0; i < 4; i++)
  {
    Eigen::Vector2d corner = toPixelCoordinates(camera_extrinsics, corners4d[i]);
    ROS_INFO_STREAM("Corner " << i << " u: " << corner[0] << " v: " << corner[1] << std::endl);
    if (!this->isValidCorner(corner[0], corner[1]))
      return std::vector<Eigen::Vector2d>();  // return empty vector
    corners2d.push_back(corner);
  }

  // TODO: check if corners are not too close (6 distance checks)

  ROS_INFO_STREAM("All corners valid.\n");
  return corners2d;
}

Eigen::Vector2d Camera::toPixelCoordinates(Eigen::Affine3d camera_extrinsics, Eigen::Vector4d point)
{
  // without distortion rectification
  // Eigen::Vector3d pixel_coordinates3d = this->camera_intrisics_matrix_*camera_extrinsics.matrix()*point;

  // with distortion rectification
  Eigen::Vector4d pixel_coordinates3d = camera_extrinsics * point;

  // scaling
  double x_prime = (pixel_coordinates3d[0] / pixel_coordinates3d[2]);
  double x_prime_squared = x_prime * x_prime;
  double y_prime = (pixel_coordinates3d[1] / pixel_coordinates3d[2]);
  double y_prime_squared = y_prime * y_prime;

  // rectifying the camera distortion
  double r_raise_2 = x_prime_squared + y_prime_squared;
  double r_raise_4 = r_raise_2 * r_raise_2;
  double r_raise_6 = r_raise_4 * r_raise_2;

  double numerator =
      1 + camera_distortions_.k1 * r_raise_2 + camera_distortions_.k2 * r_raise_4 + camera_distortions_.k3 * r_raise_6;
  double denominator =
      1 + camera_distortions_.k4 * r_raise_2 + camera_distortions_.k5 * r_raise_4 + camera_distortions_.k6 * r_raise_6;
  double coefficient = numerator / denominator;

  double x_double_prime = x_prime * coefficient + 2 * camera_distortions_.p1 * x_prime * y_prime +
                          camera_distortions_.p2 * (r_raise_2 + 2 * x_prime_squared);
  double y_double_prime = y_prime * coefficient + camera_distortions_.p1 * (r_raise_2 + 2 * y_prime_squared) +
                          2 * camera_distortions_.p2 * x_prime * y_prime;

  double u = camera_intrinsics_.fx * x_double_prime + camera_intrinsics_.cx;
  double v = camera_intrinsics_.fy * y_double_prime + camera_intrinsics_.cy;

  Eigen::Vector2d pixelCoordinates2d;
  pixelCoordinates2d[0] = u;
  pixelCoordinates2d[1] = v;

  return pixelCoordinates2d;
}

bool Camera::isValidCorner(int x, int y)
{
  if (x >= 0 && x < this->camera_intrinsics_.image_width && y >= 0 && y < this->camera_intrinsics_.image_height)
  {
    ROS_INFO_STREAM("Valid corner coordinate\n");
    return true;
  }
  ROS_INFO_STREAM("Invalid corner coordinate\n");
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
  if (target_T_camera.translation().y() > 0)
  {
    ROS_INFO_STREAM("Camera in front of target\n");
    return true;
  }
  ROS_INFO_STREAM("Camera behind target\n");
  return false;
}

bool Camera::areValidCorners(std::vector<Eigen::Vector2d> corners)
{
  // TODO
}

bool Camera::isInRange(Eigen::Affine3d transform)
{
  if (transform.translation().norm() >= this->camera_intrinsics_.min_distance_between_camera_and_target &&
      transform.translation().norm() <= this->camera_intrinsics_.max_distance_between_camera_and_target)
  {
    ROS_INFO_STREAM("Target in range\n.");
    return true;
  }
  ROS_INFO_STREAM("Target not in range\n.");
  return false;
}
}