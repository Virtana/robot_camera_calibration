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

#include "rviz_simulator/target.h"

namespace rviz_simulator
{
Target::Target(const std::string marker_frame_id, const std::string marker_name,
               const geometry_msgs::Point marker_position_in_ROSWorld, const geometry_msgs::Quaternion marker_orientation_in_ROSWorld, const std_msgs::ColorRGBA marker_color_RGBA,
               double marker_scale,
               boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server,
               unsigned int interaction_mode )
{
  this->marker_frame_id_ = marker_frame_id;
  this->marker_name_ = marker_name;
  this->marker_position_in_ROSWorld_ = marker_position_in_ROSWorld;
  this->marker_orientation_in_ROSWorld_ = marker_orientation_in_ROSWorld;
  this->marker_color_RGBA_ = marker_color_RGBA;
  this->marker_scale_ = marker_scale;
  this->g_interactive_marker_server_ = g_interactive_marker_server;
  this->interaction_mode_ = interaction_mode;

  this->make6DofMarker(marker_position_in_ROSWorld);
}

Target::~Target()
{
}

visualization_msgs::Marker Target::makeMarkerBox(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.scale.x = msg.scale * this->marker_scale_;
  marker.scale.y = msg.scale * this->marker_scale_;
  marker.scale.z = msg.scale * this->marker_scale_;

  marker.color = this->marker_color_RGBA_;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& Target::makeMarkerBoxControl(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeMarkerBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void Target::make6DofMarker(const geometry_msgs::Point& position)
{
  this->interactive_marker_.header.frame_id = this->marker_frame_id_;
  this->interactive_marker_.pose.position = this->marker_position_in_ROSWorld_;
  this->interactive_marker_.pose.orientation = this->marker_orientation_in_ROSWorld_;

  this->interactive_marker_.scale = 1;

  this->interactive_marker_.name = this->marker_name_;
  this->interactive_marker_.description = this->marker_name_;

  /// insert box with controls
  this->makeMarkerBoxControl(interactive_marker_);
  this->interactive_marker_.controls[0].interaction_mode = this->interaction_mode_;
  visualization_msgs::InteractiveMarkerControl control;

  /// set controls
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->interactive_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->interactive_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->interactive_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->interactive_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->interactive_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->interactive_marker_.controls.push_back(control);
}

void Target::targetFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  /// logging marker information
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
                   << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  /// determining feedback type
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM(s.str() << ": pose changed"
                              << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y
                              << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w
                              << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", "
                              << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: "
                              << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
      break;
  }

  this->g_interactive_marker_server_->applyChanges();
}

void Target::addTargetToServer()
{
  this->g_interactive_marker_server_->insert(this->interactive_marker_);
  this->g_interactive_marker_server_->setCallback(this->interactive_marker_.name,
                                                  boost::bind(&rviz_simulator::Target::targetFeedback, this, _1));
  ROS_INFO_STREAM("Target " << this->marker_name_ << " added.\n");
}
}