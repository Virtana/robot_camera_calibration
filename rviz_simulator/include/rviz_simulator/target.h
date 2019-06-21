/* 
 * DEV NOTES
 * 
 * writing interactivemarker processfeedback callback as a class method
 * need to use boost::bind
 * http://answers.ros.org/question/87221/interactivemarker-processfeedback-callback-as-a-class-method/
 * 
 * debugging segmentation faults, running a rosnode in GDB
 * https://bluesat.com.au/a-dummys-guide-to-debugging-ros-systems/
 * catkin_make -DCMAKE_BUILD_TYPE=Debug
 * gdb devel/lib/[ros_package_name]/[node_name] 
 * 
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

    /// @param frame_id     rviz fixed frame; chosen name: "ROSworld"
    /// @param name         target name (ex: tag1, tag2)
    /// @param position      initial position of tag in ROSworld
    /// @param etc...
    Target( const std::string frame_id, 
            const std::string name, 
            const tf::Vector3 position, 
            const float r, const float g, const float b, const float a, const float scale,
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server );

    /// destructor
    ~Target();

    /// adds target to shared server
    void addToServer();


protected:

    /// server pointer
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


private:

    std::string frame_id;
    std::string name;
    tf::Vector3 position;
    float r, g, b, a, scale;

    

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