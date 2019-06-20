/*
 * interactivemarker processfeedback callback as a class method
 * http://answers.ros.org/question/87221/interactivemarker-processfeedback-callback-as-a-class-method/
 * 
 * debugging segmentation faults
 * 
 * 
 */


#ifndef TARGET
#define TARGET

#include <interactive_markers/interactive_marker_server.h>
// #include <boost/shared_ptr.hpp>
// #include <ros/ros.h>
#include <tf/tf.h>


namespace rviz_simulator
{

class Target
{
public:

    /// constructor for Target
    Target( const std::string frame_id, 
            const std::string name, 
            const tf::Vector3 position, 
            const float r, const float g, const float b, const float a, const float scale,
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server );

    /// destructor
    ~Target();

    /// adds target to shared server
    void addToServer();


// protected:

    /// server pointer
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


// private:

    std::string frame_id;
    std::string name;
    tf::Vector3 position;
    float r, g, b, a, scale;

    // float 

    /// the interactive marker
    visualization_msgs::InteractiveMarker int_marker;

    /// makes the cube representative of the marker and sets its characteristics
    /// @param msg      The interactive marker message passed in
    /// etc
    visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg );

    /// makes the control for the marker
    visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg );

    /// make 6DOF marker
    void make6DofMarker( const tf::Vector3& positition );

    /// function to be called upon callBack
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

};

}

#endif