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

/*  TODO:
 *  make wall
 *  make room
 *  publish transforms to yaml
 *  neaten up includes, cmake, packages
 *  UML diagram
 *  Error checking and exception handling
 */

#include <ros/ros.h>

#include "rviz_simulator/target.h"
#include "rviz_simulator/camera.h"

/// pointer for the global interactive marker server for all the markers
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server;

/// Loads std_msgs::ColorRGBA from ROS parameter server
std_msgs::ColorRGBA loadColor(const ros::NodeHandle& n, const std::string& color_name)
{
  std::vector<double> color_vector;
  n.getParam(color_name, color_vector);
  std_msgs::ColorRGBA colorRGBA;
  colorRGBA.r = color_vector[0];
  colorRGBA.g = color_vector[1];
  colorRGBA.b = color_vector[2];
  colorRGBA.a = color_vector[3];
  return colorRGBA;
}

/// Loads geometry_msgs::Point from ROS parameter server
geometry_msgs::Point loadPoint(const ros::NodeHandle& n, const std::string& point_name)
{
  std::vector<double> point_vector;
  n.getParam(point_name, point_vector);
  geometry_msgs::Point point;
  point.x = point_vector[0];
  point.y = point_vector[1];
  point.z = point_vector[2];
  return point;
}

/// Loads geometry_msgs::Quaternion from ROS parameter server
geometry_msgs::Quaternion loadOrientation(const ros::NodeHandle& n, const std::string& quarternion_name)
{
  std::vector<double> quaternion_vector;
  n.getParam(quarternion_name, quaternion_vector);
  geometry_msgs::Quaternion q;
  q.x = quaternion_vector[0];
  q.y = quaternion_vector[1];
  q.z = quaternion_vector[2];
  q.w = quaternion_vector[3];
  return q;
}

/// Loads rviz_simulator::CameraIntrinsics from ROS parameter server
rviz_simulator::CameraIntrinsics loadCameraIntrinsics(const ros::NodeHandle& n)
{
  rviz_simulator::CameraIntrinsics intrinsics;
  n.getParam("width", intrinsics.image_width);
  n.getParam("height", intrinsics.image_height);
  n.getParam("fx", intrinsics.fx);
  n.getParam("fy", intrinsics.fy);
  n.getParam("cx", intrinsics.cx);
  n.getParam("cy", intrinsics.cy);
  n.getParam("near_clip", intrinsics.min_distance_between_camera_and_target);
  n.getParam("far_clip", intrinsics.max_distance_between_camera_and_target);
  return intrinsics;
}

/// Loads rviz_simulator::CameraDistortions from ROS parameter server
rviz_simulator::CameraDistortions loadCameraDistortions(const ros::NodeHandle& n)
{
  rviz_simulator::CameraDistortions distortions;
  n.getParam("k1", distortions.k1);
  n.getParam("k2", distortions.k2);
  n.getParam("k3", distortions.k3);
  n.getParam("k4", distortions.k4);
  n.getParam("k5", distortions.k5);
  n.getParam("k6", distortions.k6);
  n.getParam("p1", distortions.p1);
  n.getParam("p2", distortions.p2);
  return distortions;
}

/// Makes a row of targets
void makeLineOfTargets(std::string world_frame_id, int num_targets, int distance_between_targets,
                       int first_target_number, geometry_msgs::Point first_target_position,
                       geometry_msgs::Quaternion target_orientation, std_msgs::ColorRGBA world_origin_color,
                       std_msgs::ColorRGBA regular_target_color)
{
  std_msgs::ColorRGBA color = regular_target_color;
  if (first_target_number == 0)
    color = world_origin_color;

  geometry_msgs::Point position = first_target_position;
  for (int i = first_target_number; i < num_targets; i++)
  {
    rviz_simulator::Target* target =
        new rviz_simulator::Target(world_frame_id, "tag" + std::to_string(i), position, target_orientation, color, 0.1,
                                   g_interactive_marker_server, visualization_msgs::InteractiveMarkerControl::MOVE_3D);
    target->addTargetToServer();
    position.x += distance_between_targets;

    color = regular_target_color;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulate");
  ros::NodeHandle n;

  g_interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("simulate", "", false));

  ros::Duration(0.1).sleep();

  // loading parameters
  std::string world_frame_id;
  n.getParam("/world_frame_id", world_frame_id);

  /// setting target marker color and scale
  std_msgs::ColorRGBA grey = loadColor(n, "grey");
  std_msgs::ColorRGBA blue = loadColor(n, "blue");
  double target_scale;
  n.getParam("target_scale", target_scale);

  /// setting camera marker color and scale
  std_msgs::ColorRGBA orange = loadColor(n, "orange");
  double camera_scale;
  n.getParam("camera_scale", camera_scale);

  /// setting marker position in ROSWorld
  geometry_msgs::Point starting_target_position = loadPoint(n, "starting_target_position");
  geometry_msgs::Quaternion starting_target_orientation = loadOrientation(n, "starting_target_orientation");

  /// making a line of targets and adding to server
  int num_targets;
  n.getParam("num_targets_in_line", num_targets);
  double distance_between_targets;
  n.getParam("distance_between_targets", distance_between_targets);
  int first_target_number = 0;
  makeLineOfTargets(world_frame_id, num_targets, distance_between_targets, first_target_number,
                    starting_target_position, starting_target_orientation, blue, grey);

  /// adding camera
  rviz_simulator::CameraIntrinsics camera_intrinsics = loadCameraIntrinsics(n);
  rviz_simulator::CameraDistortions camera_distortions = loadCameraDistortions(n);
  geometry_msgs::Point starting_camera_postion = loadPoint(n, "starting_camera_positon");
  geometry_msgs::Quaternion starting_camera_orientation = loadOrientation(n, "starting_camera_orientation");

  // accurate starting camera orientation
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << -1, 0, 0, 0, 0, -1, 0, -1, 0;
  Eigen::Quaterniond q(rotation_matrix);
  ROS_INFO_STREAM("Quaternion: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n");
  tf::quaternionEigenToMsg(q, starting_camera_orientation);

  rviz_simulator::Camera camera(world_frame_id, "camera", starting_camera_postion, starting_camera_orientation, orange,
                                0.2, g_interactive_marker_server, visualization_msgs::InteractiveMarkerControl::BUTTON,
                                camera_intrinsics, camera_distortions);

  g_interactive_marker_server->applyChanges();
  ros::spin();
  g_interactive_marker_server.reset();
}