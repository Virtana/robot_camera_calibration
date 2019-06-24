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
Target::Target( const std::string frame_id, 
                const std::string name, 
                const tf::Vector3 position, 
                const float r, const float g, const float b, const float a, const float scale,
                boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_server)
{
  this->frame_id_ = frame_id;
  this->name_ = name;
  this->position_ = position;
  this->r_ = r;
  this->g_ = g;
  this->b_ = b;
  this->a_ = a;
  this->scale_ = scale;
  this->server_ = g_server;

  /// make target
  this->make6DofMarker(position);
}

Target::~Target()
{
}

visualization_msgs::Marker Target::makeBox(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.scale.x = msg.scale * this->scale_;
  marker.scale.y = msg.scale * this->scale_;
  marker.scale.z = msg.scale * this->scale_;

  marker.color.r = this->r_;
  marker.color.g = this->g_;
  marker.color.b = this->b_;
  marker.color.a = this->a_;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& Target::makeBoxControl(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void Target::make6DofMarker(const tf::Vector3& positition)
{
  this->int_marker_.header.frame_id = this->frame_id_;
  tf::pointTFToMsg(position_, this->int_marker_.pose.position);

  this->int_marker_.scale = 1;

  this->int_marker_.name = this->name_;
  this->int_marker_.description = "6DOF_MOVE_ROTATE_3D";

  /// insert box with controls
  this->makeBoxControl(int_marker_);
  this->int_marker_.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  visualization_msgs::InteractiveMarkerControl control;
  /// marker axis not fixed

  /// set controls
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->int_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->int_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  this->int_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  this->int_marker_.controls.push_back(control);
}

void Target::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
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
      ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    /// case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    ///   ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() <<
    ///   "." );
    ///   break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM(s.str() << ": pose changed"
                              << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y
                              << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w
                              << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", "
                              << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: "
                              << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
      break;

      /// case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ///   ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      ///   break;

      /// case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ///   ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      ///   break;
  }

  this->server_->applyChanges();
}

void Target::addToServer()
{
  this->server_->insert(this->int_marker_);
  this->server_->setCallback(this->int_marker_.name, boost::bind(&rviz_simulator::Target::processFeedback, this, _1));
}
}