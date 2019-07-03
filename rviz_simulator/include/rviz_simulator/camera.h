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

#ifndef RVIZ_SIMULATOR_CAMERA_H
#define RVIZ_SIMULATOR_CAMERA_H

// #include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
// #include "tf/transform_datatypes.h"
#include <eigen_conversions/eigen_msg.h>
#include "yaml-cpp/yaml.h"
#include <fstream>

// for making directories
#include <sys/stat.h> 
#include <sys/types.h>

// for getting package path
#include <ros/package.h>

#include "rviz_simulator/target.h"

// #include <tf/tf_eigen.h>

namespace rviz_simulator
{

/// structs used by the Camera class
struct CameraIntrinsics
{
  /// focal length in pixels
  double fx;
  double fy;

  /// optical center in pixels
  double cx;
  double cy;

  /// camera image extremes
  int image_height;
  int image_width;
  double min_distance_between_target_corners;
  double max_distance_between_camera_and_target;
  double min_distance_between_camera_and_target;
};

/// Camera distortions. Nomenclature follows from the OpenCV docs
/// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html 
struct CameraDistortions
{
  double k1, k2, k3, k4, k5, k6;  /// radial distortions
  double p1, p2;                  /// tangential distortions
};

/// Camera is the subclaass of the Target class defined in target.h
/// The model is based on the OpenCV camera calibration doc
///   https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html 
class Camera: public Target
{
public:

  /// @param marker_frame_id                  RViz fixed frame: chosen name: "ROSWorld"
  /// @param marker_name                      Target name (ex: tag0, tag1, tag2)
  /// @param marker_position_in_ROSWorld      Initial position of tag in ROSWorld
  /// @param marker_orientation_in_ROSWorld   Initial orientation of tag in ROSWorld
  /// @param marker_color_RGBA                Marker colour (red, green blue) and opacity setting
  /// @param marker_scale                     The length of a side of the interactive marker
  /// @param g_interactive_marker_server      Shared pointer for interactive marker server
  /// @oaram interaction_mode                 Specifies how the marker will react to events (3D MOVEMENT or BUTTON clicks)
  /// @param camera_intrinsics              Intrinsic properties for the simulated camera
  /// @param camera_distortions             Camera distortions
  Camera( const std::string marker_frame_id, const std::string marker_name,
          const geometry_msgs::Point marker_position_in_ROSWorld, const geometry_msgs::Quaternion marker_orientation_in_ROSWorld, const std_msgs::ColorRGBA marker_color_RGBA,
          double marker_scale,
          boost::shared_ptr<interactive_markers::InteractiveMarkerServer> g_interactive_marker_server,
          unsigned int interaction_mode,
          CameraIntrinsics camera_intrinsics,
          CameraDistortions camera_distortions );

    /// Destructor
    ~Camera();

    /// Adds camera to shared interactive marker server.
    void addCameraToServer();

private:

    /// Camera properties
    CameraIntrinsics camera_intrinsics_;
    CameraDistortions camera_distortions_;
    Eigen::MatrixXd camera_intrisics_matrix_;
    double length_of_target_;                   /// The real-world length of a target
    int detections_file_number_;                /// The number of pictures taken during the running of the node
    std::string output_folder_name_;            /// name of folder in which the detection yaml files are dumped, suffixed by a ROS timestamp
    std::string output_folder_path_;

    /// Function to call on the arrival of a feedback message (3D movement of camera or left mouse click).
    /// @param    Reference for marker message.
    void cameraFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

    /// Adds interactive marker control button to simulate the camera's shutter release.
    /// This enables an event (function) to be triggered upon the user's left mouse click.
    /// This event is described in the "takePicture" function
    void addShutterReleaseButton();

    /// calculates the transform (translation and rotation) between two interactive markers
    /// @param reference    The reference frame
    /// @param target       The target frame
    /// @return             A 4x4 Affine3d matrix representing the transfrom from the reference to the target frame
    Eigen::Affine3d getTransformBetweenInteractiveMarkers(visualization_msgs::InteractiveMarker &reference, visualization_msgs::InteractiveMarker &target);

    /// Simulated the taking of a picture with the camera.
    /// On left mouse click, the camera captures all targets in its field of view
    /// and projects their corners onto a 2d image which is output as a .yaml file.
    void takePicture();

    /// Calculates the pixel coordinates of the four corners of a target.
    /// @param camera_extrinsics    The transform camera_T_target.
    /// @return                     Vector of corner (x, y) pixel coordinates
    std::vector<Eigen::Vector2d> processCorners( Eigen::Affine3d camera_extrinsics );
    
    /// Converts the single 3D point of a target's corner in the world to a 2d pixel coordinate in an images
    /// Performs rectification on distorted camera images.
    /// Follows the model used in the OpenCV camera calibration doc
    /// @param camera_extrinsics    The transform camera_T_target.
    /// @param point                A 3D point of a targets corner in the world frame represented as (X, Y, Z, 1).
    /// @return                     A 2D vector (u,v) representing the pixel coordinate.
    Eigen::Vector2d toPixelCoordinates( Eigen::Affine3d camera_extrinsics, Eigen::Vector4d point );

    /// Checks if a single pixel coordinate is in range.
    /// @param x      The x coordinate of a pixel in range [0 - image_width-1]
    /// @param y      The y coordinate of a pixel in range [0 - image_height-1]
    /// @return       Returns true if both coordinates are in range, false otherwise
    bool isValidCorner( int x, int y );

    /// Checks if a target is in front of the camera 
    /// @param camera_T_target    Transform from camera's reference frame to the target's reference frame
    /// @return                   True if the target is in front of the camera, false otherwise
    bool isInFrontOfCamera( Eigen::Affine3d camera_T_target );

    /// Checks if a camera is in front of the target 
    /// @param target_T_camera    Transform from targets's reference frame to the camera's reference frame
    /// @return                   True if the camera is in front of the target, false otherwise
    bool isInFrontOfTarget( Eigen::Affine3d target_T_camera );

    /// Checks if the four corners of a target are not too close to each other when projected into 2d
    /// @param corners    A vector of the 4 corners of a target
    /// return            True if the four corners of a target are not too close when projected into 2d, false otherwise
    bool areValidCorners( std::vector<Eigen::Vector2d> corners );

    /// Checks if a target is too far from the camera (distance between interactive markers)
    bool isInRange(Eigen::Affine3d transform);

    /// Checks if the angle between the camera's z axis and the target's z axis is valid 
    bool isValidAngle();
};
}

#endif