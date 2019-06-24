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

#ifndef TARGET
#define TARGET

#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>

namespace rviz_simulator
{
/// Uses interactive markers to model a fiducial target in rviz
class Target
{
public:
  /// @param frame_id       RViz fixed frame; chosen name: "ROSWorld"
  /// @param name           Target name (ex: tag1, tag2)
  /// @param position       Initial position of tag in ROSWorld (Vector3)
  /// @param r              Red color for marker
  /// @param g              Green color for marker
  /// @param b              Blue color for marker
  /// @param a              Opacity for marker
  /// @param scale          Size scaling color for marker
  /// @param g_server       Shared pointer for interactive marker server
  Target(const std::string frame_id, 
        const std::string name, 
        const tf::Vector3 position, 
        const float r, const float g, const float b, const float a, const float scale,
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_server);

  /// Destructor
  ~Target();

  /// Adds target to shared interactive marker server.
  void addToServer();

protected:
  /// Server pointer is stored as a member variable.
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

private:
  /// Marker properties.
  std::string frame_id_;
  std::string name_;
  tf::Vector3 position_;
  float r_, g_, b_, a_, scale_;

  /// The interactive marker.
  visualization_msgs::InteractiveMarker int_marker_;

  /// Makes the cube representative of the marker and sets its characteristics.
  /// @param msg        Reference to the interactive marker message.
  /// @return           The marker (cube) generated
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker& msg);

  /// Sets the type of marker interaction (move, rotate, button) for the marker.
  /// @param msg        Reference to the interactive marker message.
  /// @return           A reference to the interactive marker control.
  visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker& msg);

  /// Makes 6DOF marker.
  /// Calls the makeBox and makeBoxControl functions above.
  /// @param    Initial position of marker in ROSWorld.
  void make6DofMarker(const tf::Vector3& positition);

  /// Function to call on the arrival of a feedback message.
  /// @param    Reference for marker message.
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};
}

#endif