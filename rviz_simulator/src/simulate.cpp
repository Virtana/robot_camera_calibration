#include <ros/ros.h>

#include "rviz_simulator/target.h"
#include "rviz_simulator/camera.h"
// #include <boost/shared_ptr.hpp>


/// server for all the markers
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


int main(int argc, char** argv)
{
  ros::init( argc, argv, "simulate" );
  ros::NodeHandle n;  
  
  server.reset( new interactive_markers::InteractiveMarkerServer("simulate", "", false) );
  
  ros::Duration(0.1).sleep();

  rviz_simulator::Target target("ROSworld", "target_1", tf::Vector3(-3, 3, 0), 0.5, 0.5, 0.5, 1.0, 1.0, server);  
  target.addToServer();

  // rviz_simulator::Target target2("ROSworld", "target_2", tf::Vector3(0, 3, 0), 0.5, 0.5, 0.5, 1.0, 1.0, server);  
  // target2.addToServer();

  server->applyChanges();

  ros::spin();

  server.reset();
}