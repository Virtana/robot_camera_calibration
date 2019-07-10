#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Dense"
#include "fstream"
#include "string"
#include "iostream"

int count(0);
std::string capture("1");//prevents first iteration YAML dump
std::ofstream fout;

//returns YAML formatted pixel coordinate string given desired corner, cam_T_tag transform and tag size
std::string pixCalc(Eigen::MatrixXd cam_T_tag, double tag_size, int x, int y) 
{
  //intrinsic specification
  Eigen::MatrixXd intrinsic(3, 3);  
  intrinsic << 630.3467672874693, 0, 314.10832937485145, 0, 624.9044125522948, 241.59156511756711, 0, 0, 1;
  
  // 4x1 (X,Y,Z,1) for each tag corner  
  Eigen::MatrixXd corner(4, 1);
  corner << (x*(tag_size / 2)), (y*(tag_size / 2)), 0, 1;

  // pixel coordinate calculation
  Eigen::MatrixXd pix(3, 1); 
  pix = (intrinsic * cam_T_tag) * corner; 

  // coordinate scaling
  pix << (pix(0, 0) / pix(2, 0)), (pix(1, 0) / pix(2, 0)), 1; 

  //yaml string output formatting
  std::string coord("[ " + std::to_string(int(pix(0, 0))) + ", " + std::to_string(int(pix(1, 0))) + " ]"); 

  return coord;
}

void aprilDetection(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{ 
  if(capture=="") //specifies trigger key to dump YAML
  {
    if (!msg->detections.empty())
    {
      //file name generation
      std::string filename("detections_"+std::to_string(count)+".yml");  
      fout.open(filename.c_str());
      fout << "detections:";

      for (int i = (msg->detections.size() - 1); i > -1; i--)
      {
        // reading data from topic
        float cam_T_tag_xpos = msg->detections[i].pose.pose.pose.position.x;
        float cam_T_tag_ypos = msg->detections[i].pose.pose.pose.position.y;
        float cam_T_tag_zpos = msg->detections[i].pose.pose.pose.position.z;
        float cam_T_tag_xori = msg->detections[i].pose.pose.pose.orientation.x;
        float cam_T_tag_yori = msg->detections[i].pose.pose.pose.orientation.y;
        float cam_T_tag_zori = msg->detections[i].pose.pose.pose.orientation.z;
        float cam_T_tag_wori = msg->detections[i].pose.pose.pose.orientation.w;
        double tag_size = msg->detections[i].size[0];
        int tag_id = msg->detections[i].id[0];

        tf::Quaternion q(cam_T_tag_xori, cam_T_tag_yori, cam_T_tag_zori, cam_T_tag_wori);
        tf::Matrix3x3 m(q);  // quaternion to rotational matrix

        Eigen::MatrixXd cam_T_tag(3, 4);  // 3x4 transformation matrix
        cam_T_tag << m[0][0], m[0][1], m[0][2], cam_T_tag_xpos, m[1][0], m[1][1], m[1][2], cam_T_tag_ypos, m[2][0], m[2][1], m[2][2], cam_T_tag_zpos;

        // yaml population
        fout << "\n - TargetID: " << std::to_string(tag_id); 
        fout << "\n   size: " << std::to_string(tag_size);
        fout << "\n   corners:";
        fout << "\n    0: " << pixCalc(cam_T_tag, tag_size, -1, 1);
        fout << "\n    1: " << pixCalc(cam_T_tag, tag_size, 1, 1);
        fout << "\n    2: " << pixCalc(cam_T_tag, tag_size, 1, -1);
        fout << "\n    3: " << pixCalc(cam_T_tag, tag_size, -1, -1);
      }
      count++;
    }
    fout.close();
    capture = "1"; //ensures input key does not force infinite loop
  }
  std::getline(std::cin,capture); //trigger input 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "corner_detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tag_detections", 1, aprilDetection);
  ros::spin();
  return 0;
}
