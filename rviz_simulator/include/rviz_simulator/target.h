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

#ifndef RVIZ_SIMULATOR_TARGET_H
#define RVIZ_SIMULATOR_TARGET_H

#include <interactive_markers/interactive_marker_server.h>

namespace rviz_simulator
{
/// Target is the superclass for the Camera class defined in camera.h
/// Target uses interactive markers to model a fiducial target in rviz
class Target
{
public:
  /// @param marker_frame_id                  RViz fixed frame: chosen name: "ROSWorld"
  /// @param marker_name                      Target name (ex: tag0, tag1, tag2)
  /// @param marker_position_in_ROSWorld      Initial position of tag in ROSWorld
  /// @param marker_orientation_in_ROSWorld   Initial orientation of tag in ROSWorld
  /// @param marker_color_RGBA                Marker colour (red, green blue) and opacity setting
  /// @param marker_scale                     The length of a side of the interactive marker
  /// @param g_interactive_marker_server      Shared pointer for interactive marker server
  /// @param interaction_mode                 Specifies how the marker will react to events (3D MOVEMENT or BUTTON
  /// clicks)
  Target(const std::string marker_frame_id, const std::string marker_name,
         const geometry_msgs::Point marker_position_in_ROSWorld,
         const geometry_msgs::Quaternion marker_orientation_in_ROSWorld, const std_msgs::ColorRGBA marker_color_RGBA,
         std::vector<double> target_size,
         boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server,
         unsigned int interaction_mode);

  /// Destructor
  ~Target();

  /// Adds target to shared interactive marker server.
  void addTargetToServer();

protected:
  /// Server pointer is stored as a member variable.
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server_;

  /// The interactive marker.
  visualization_msgs::InteractiveMarker interactive_marker_;

  /// The name of the interactive marker
  std::string marker_name_;

private:
  /// Marker properties.
  std::string marker_frame_id_;
  geometry_msgs::Point marker_position_in_ROSWorld_;
  geometry_msgs::Quaternion marker_orientation_in_ROSWorld_;
  std_msgs::ColorRGBA marker_color_RGBA_;
  std::vector<double> target_size_;
  unsigned int interaction_mode_;

  /// Makes the cube representative of the marker and sets its characteristics.
  /// @param msg        Reference to the interactive marker message.
  /// @return           The marker (cube) generated
  visualization_msgs::Marker makeMarkerBox(visualization_msgs::InteractiveMarker& msg);

  /// Sets the type of marker interaction (move, rotate, button) for the marker.
  /// @param msg        Reference to the interactive marker message.
  /// @return           A reference to the interactive marker control.
  visualization_msgs::InteractiveMarkerControl& makeMarkerBoxControl(visualization_msgs::InteractiveMarker& msg);

  /// Makes six degree of freedom (6DoF) marker.
  /// Calls the makeBox and makeBoxControl functions above.
  /// @param    Initial position of marker in ROSWorld.
  void make6DofMarker(const geometry_msgs::Point& position);

  /// Function to call on the arrival of a feedback message (3D movement of target).
  /// @param    Reference for marker message.
  void targetFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};
}
#endif
