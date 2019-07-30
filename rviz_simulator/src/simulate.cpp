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
 *  UML diagram
 *  Error checking and exception handling
 *
 * ===========================================
 *
 * Refactor using a custom yaml_io class
 *
 *  Make it impossible to move markers after first camera click
 *      Requires a removal and reinitialization of interactive markers upon first camera click
 */

#include <ros/ros.h>

#include "rviz_simulator/target.h"
#include "rviz_simulator/camera.h"

/// pointer for the global interactive marker server for all the markers
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server;

/// Loads geometry_msgs::Point from ROS parameter server
geometry_msgs::Point loadPoint(const ros::NodeHandle& n, const std::string& point_name)
{
  std::vector<double> point_vector;
  geometry_msgs::Point point;
  if (n.getParam(point_name, point_vector))
  {
    point.x = point_vector[0];
    point.y = point_vector[1];
    point.z = point_vector[2];
  }
  else  // point param not specified
  {
    ROS_WARN_STREAM(point_name << " param was not specified. Using default values.");
    if (point_name == "starting_target_position")
    {
      point.x = 0.0;
      point.y = 0.0;
      point.z = 0.0;
    }
    else if (point_name == "starting_camera_position")
    {
      point.x = 0.0;
      point.y = 3.0;
      point.z = 0.0;
    }
    else  // invalid point name
    {
      ROS_ERROR_STREAM(point_name << " is an invalid param name.");
      ros::shutdown();
    }
  }
  return point;
}

/// Loads an orientation from ROS parameter server
/// The orientation is stored as a 9 element YAML sequence in col-major order
/// And is converted to a geometry_msgs::Quaternion
geometry_msgs::Quaternion loadOrientation(const ros::NodeHandle& n, const std::string& orientation_name)
{
  std::vector<double> rotation_matrix_col_major;
  if (!n.getParam(orientation_name, rotation_matrix_col_major))
  {
    ROS_WARN_STREAM(orientation_name << " param was not specified. Using default values.");
    if (orientation_name == "starting_target_orientation")
    {
      rotation_matrix_col_major = { -1, 0, 0, 0, 0, 1, 0, 1, 0 };
    }
    else if (orientation_name == "starting_camera_orientation")
    {
      rotation_matrix_col_major = { -1, 0, 0, 0, 0, -1, 0, -1, 0 };
    }
    else
    {
      ROS_ERROR_STREAM(orientation_name << " is an invalid param name.");
      ros::shutdown();
    }
  }

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::Map<Eigen::Matrix3d>(rotation_matrix_col_major.data());
  Eigen::Quaterniond q;
  q = rotation_matrix;
  geometry_msgs::Quaternion starting_orientation;
  tf::quaternionEigenToMsg(q, starting_orientation);
  return starting_orientation;
}

/// Loads rviz_simulator::CameraProperties from ROS parameter server
rviz_simulator::CameraProperties loadCameraProperties(const ros::NodeHandle& n)
{
  rviz_simulator::CameraProperties camera_properties;
  if (!n.getParam("image_width", camera_properties.image_width))
  {
    ROS_ERROR("image_width camera property not specified.");
    ros::shutdown();
  }

  if (!n.getParam("image_height", camera_properties.image_height))
  {
    ROS_ERROR("image_height camera property not specified.");
    ros::shutdown();
  }

  if (!n.getParam("camera_name", camera_properties.camera_name))
  {
    ROS_ERROR("camera_name camera property not specified.");
    ros::shutdown();
  }

  std::vector<double> camera_matrix;
  if (!n.getParam("camera_matrix/data", camera_matrix))
  {
    ROS_ERROR("camera_matrix camera property not specified.");
    ros::shutdown();
  }

  if (!n.getParam("distortion_model", camera_properties.distortion_model))
  {
    ROS_ERROR("distortion_model camera property not specified.");
    ros::shutdown();
  }

  std::vector<double> distortion_coefficients;
  if (!n.getParam("distortion_coefficients/data", distortion_coefficients))
  {
    ROS_ERROR("distortion_coefficients camera property not specified.");
    ros::shutdown();
  }

  camera_properties.fx = camera_matrix[0];
  camera_properties.fy = camera_matrix[4];
  camera_properties.cx = camera_matrix[2];
  camera_properties.cy = camera_matrix[5];

  // TODO
  // Cater for other camera models
  if (camera_properties.distortion_model == "plumb_bob")
  {
    camera_properties.k1 = distortion_coefficients[0];
    camera_properties.k2 = distortion_coefficients[1];
    camera_properties.k3 = distortion_coefficients[4];
    camera_properties.k4 = 0;
    camera_properties.k5 = 0;
    camera_properties.k6 = 0;
    camera_properties.p1 = distortion_coefficients[2];
    camera_properties.p2 = distortion_coefficients[3];
  }
  else
  {
    ROS_ERROR("Unknown camera distortion model specified!\n");
    ros::shutdown();
  }

  return camera_properties;
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

  ros::Duration(1.0).sleep();

  // loading parameters
  // getting the world_frame_id
  std::string world_frame_id;
  if (!n.getParam("/world_frame_id", world_frame_id))
  {
    world_frame_id = "ROSWorld";
    ROS_WARN_STREAM("\"world_frame_id\" param was not specified. Using \"" << world_frame_id << "\" as the "
                                                                                                "world_frame_id.");
  }

  /// setting target marker color and scale
  double target_scale = 0.1;
  std_msgs::ColorRGBA target_color;  // grey
  target_color.r = 0.5;
  target_color.g = 0.5;
  target_color.b = 0.5;
  target_color.a = 1.0;

  std_msgs::ColorRGBA world_origin_color;  // blue
  world_origin_color.r = 0.0;
  world_origin_color.g = 0.67;
  world_origin_color.b = 0.9;
  world_origin_color.a = 1.0;

  /// setting camera marker color and scale
  double camera_scale = 0.2;
  std_msgs::ColorRGBA camera_color;  // orange
  camera_color.r = 1.0;
  camera_color.g = 0.39;
  camera_color.b = 0.12;
  camera_color.a = 1.0;

  /// setting first target position and orientaiton in ROSWorld
  geometry_msgs::Point starting_target_position = loadPoint(n, "starting_target_position");
  geometry_msgs::Quaternion starting_target_orientation = loadOrientation(n, "starting_target_orientation");

  /// making a line of targets and adding to server
  int num_targets;
  if (!n.getParam("num_targets", num_targets))
  {
    num_targets = 5;
    ROS_WARN_STREAM("num_targets param not specified. Using default value: " << num_targets);
  }

  double distance_between_targets;
  if (!n.getParam("distance_between_targets", distance_between_targets))
  {
    distance_between_targets = 1;
    ROS_WARN_STREAM("distance_between_targets param not specified. Using default value: " << distance_between_targets);
  }

  int first_target_number = 0;

  makeLineOfTargets(world_frame_id, num_targets, distance_between_targets, first_target_number,
                    starting_target_position, starting_target_orientation, world_origin_color, target_color);

  /// loading camera properties
  rviz_simulator::CameraProperties camera_properties = loadCameraProperties(n);

  /// setting camera position and orientaiton in ROSWorld
  geometry_msgs::Point starting_camera_postion = loadPoint(n, "starting_camera_position");
  geometry_msgs::Quaternion starting_camera_orientation = loadOrientation(n, "starting_camera_orientation");

  /// adding camera to interactive marker server
  rviz_simulator::Camera camera(world_frame_id, "camera", starting_camera_postion, starting_camera_orientation,
                                camera_color, camera_scale, target_scale, g_interactive_marker_server,
                                visualization_msgs::InteractiveMarkerControl::BUTTON, camera_properties);

  g_interactive_marker_server->applyChanges();
  ros::spin();
  g_interactive_marker_server.reset();
}
