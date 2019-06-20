#include <ros/ros.h>

// #include "rviz_simulator/camera.h"
#include "rviz_simulator/target.h"
// #include <boost/shared_ptr.hpp>


/// server for all the markers
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;


int main(int argc, char** argv)
{
  std::cout << "Good-bye";

  ros::init( argc, argv, "simulate" );
  ros::NodeHandle n;  

  // // did not add timer
  // // actual server object make ans pointer set
  server.reset( new interactive_markers::InteractiveMarkerServer("simulate", "", false) );
  
  ros::Duration(0.1).sleep();

  // // std::string s = "ROSworld";
  // std::cout << "Hello";

  tf::Vector3 position;
  position = tf::Vector3(-3, 3, 0);
  rviz_simulator::Target target("ROSworld", "target_1", position, 0.5, 0.5, 0.5, 1.0, 1.0, server);
  // std::cout << "Hello";
  // target.addToServer();
  // // server->insert(target.int_marker);
  // // server->setCallback(target.int_marker.name, &target.processFeedback);


  server->applyChanges();

  ros::spin();

  server.reset();

}