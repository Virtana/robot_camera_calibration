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

#include <ros/ros.h>

#include "rviz_simulator/target.h"

/// pointer for the global interactive marker server for all the markers
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulate");
  ros::NodeHandle n;

  g_interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("simulate", "", false));

  ros::Duration(0.1).sleep();

  /// setting marker color and scale
  std_msgs::ColorRGBA color_RGBA;
  color_RGBA.r = 0.5;
  color_RGBA.g = 0.5;
  color_RGBA.b = 0.5;
  color_RGBA.a = 1.0;
  double marker_scale = 0.1;

  /// setting marker position in ROSWorld
  geometry_msgs::Point marker_position;
  marker_position.x = 0;
  marker_position.y = 0;
  marker_position.z = 0;

  /// making target and adding to server
  rviz_simulator::Target target("ROSWorld", "target_1", marker_position, color_RGBA, marker_scale,
                                g_interactive_marker_server);
  target.addInteractiveMarkerToServer();

  g_interactive_marker_server->applyChanges();
  ros::spin();
  g_interactive_marker_server.reset();
}