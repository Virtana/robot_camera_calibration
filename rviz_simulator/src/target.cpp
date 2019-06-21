#include "rviz_simulator/target.h"

namespace rviz_simulator
{

    Target::Target( const std::string frame_id, 
                    const std::string name, 
                    const tf::Vector3 position, 
                    const float r, const float g, const float b, const float a, const float scale,
                    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server )
    {
        this->frame_id = frame_id;
        this->name = name;
        this->position;
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
        this->scale = scale;
        this->server = server;       
        
        // make target
        this->make6DofMarker( position );        
    }


    Target::~Target()
    {

    }


    visualization_msgs::Marker Target::makeBox( visualization_msgs::InteractiveMarker &msg )
    {
        visualization_msgs::Marker marker;
        
        marker.type = visualization_msgs::Marker::CUBE;

        marker.scale.x = msg.scale * this->scale;
        marker.scale.y = msg.scale * this->scale;
        marker.scale.z = msg.scale * this->scale;

        marker.color.r = this->r;
        marker.color.g = this->g;
        marker.color.b = this->b;
        marker.color.a = this->a;

        return marker;
    }


    visualization_msgs::InteractiveMarkerControl& Target::makeBoxControl( visualization_msgs::InteractiveMarker &msg )
    {
        visualization_msgs::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back( makeBox(msg) );
        msg.controls.push_back( control );

        return msg.controls.back();
    }

    
    void Target::make6DofMarker( const tf::Vector3& positition )
    {       
        this->int_marker.header.frame_id = this->frame_id;
        tf::pointTFToMsg( position, this->int_marker.pose.position );

        this->int_marker.scale = 1;

        this->int_marker.name = this->name;
        this->int_marker.description = "6DOF_MOVE_ROTATE_3D";

        // insert box with controls
        this->makeBoxControl( int_marker );
        this->int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
        visualization_msgs::InteractiveMarkerControl control;
        // marker axis not fixed

        

        // set controls
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        this->int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        this->int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        this->int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        this->int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        this->int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        this->int_marker.controls.push_back(control);
    }


    void Target::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "' "
            << " / control '" << feedback->control_name << "'";

        std::ostringstream mouse_point_ss;
        if( feedback->mouse_point_valid )
        {
            mouse_point_ss << " at " << feedback->mouse_point.x
                        << ", " << feedback->mouse_point.y
                        << ", " << feedback->mouse_point.z
                        << " in frame " << feedback->header.frame_id;
        }

        switch ( feedback->event_type )
        {
            case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

            // case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            //   ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            //   break;

            case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( s.str() << ": pose changed"
                << "\nposition = "
                << feedback->pose.position.x
                << ", " << feedback->pose.position.y
                << ", " << feedback->pose.position.z
                << "\norientation = "
                << feedback->pose.orientation.w
                << ", " << feedback->pose.orientation.x
                << ", " << feedback->pose.orientation.y
                << ", " << feedback->pose.orientation.z
                << "\nframe: " << feedback->header.frame_id
                << " time: " << feedback->header.stamp.sec << "sec, "
                << feedback->header.stamp.nsec << " nsec" );
            break;

            // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            //   ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            //   break;

            // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            //   ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            //   break;
        }

        this->server->applyChanges();
    }


    void Target::addToServer()
    {
        this->server->insert( this->int_marker );
        this->server->setCallback( this->int_marker.name, boost::bind(&rviz_simulator::Target::processFeedback, this, _1) );
    }

}