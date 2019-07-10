#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Dense"
#include "fstream"
#include "string"
#include "iostream"

int count(0);
std::string capture("1");//prevenets first iteration YAML dump
std::ofstream fout;

void aprilDetection(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{ 
  if(capture=="") //specifies trigger key to dump YAML
  {
    if (!msg->detections.empty())
    {
      //intrinsic specification
      Eigen::MatrixXd intrinsic(3, 3);  
      intrinsic << 630.3467672874693, 0, 314.10832937485145, 0, 624.9044125522948, 241.59156511756711, 0, 0, 1;
      
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

        // 4x1 (X,Y,Z,1) for each tag corner
        Eigen::MatrixXd cornertl(4, 1);  
        cornertl << -(tag_size / 2), (tag_size / 2), 0, 1;
        Eigen::MatrixXd cornertr(4, 1);
        cornertr << (tag_size / 2), (tag_size / 2), 0, 1;
        Eigen::MatrixXd cornerbl(4, 1);
        cornerbl << -(tag_size / 2), -(tag_size / 2), 0, 1;
        Eigen::MatrixXd cornerbr(4, 1);
        cornerbr << (tag_size / 2), -(tag_size / 2), 0, 1;

        Eigen::MatrixXd pixtl(3, 1); 
        Eigen::MatrixXd pixtr(3, 1);
        Eigen::MatrixXd pixbr(3, 1);
        Eigen::MatrixXd pixbl(3, 1);

        // pixel coordinate calculation
        pixtl = (intrinsic * cam_T_tag) * cornertl;  
        pixtr = (intrinsic * cam_T_tag) * cornertr;
        pixbr = (intrinsic * cam_T_tag) * cornerbr;
        pixbl = (intrinsic * cam_T_tag) * cornerbl;

        // coordinate scaling
        pixtl << (pixtl(0, 0) / pixtl(2, 0)), (pixtl(1, 0) / pixtl(2, 0)), 1; 
        pixtr << (pixtr(0, 0) / pixtr(2, 0)), (pixtr(1, 0) / pixtr(2, 0)), 1;
        pixbr << (pixbr(0, 0) / pixbr(2, 0)), (pixbr(1, 0) / pixbr(2, 0)), 1;
        pixbl << (pixbl(0, 0) / pixbl(2, 0)), (pixbl(1, 0) / pixbl(2, 0)), 1;

        //yaml string output formatting
        std::string tl("[ " + std::to_string(int(pixtl(0, 0))) + ", " + std::to_string(int(pixtl(1, 0))) + " ]");         
        std::string tr("[ " + std::to_string(int(pixtr(0, 0))) + ", " + std::to_string(int(pixtr(1, 0))) + " ]");
        std::string br("[ " + std::to_string(int(pixbr(0, 0))) + ", " + std::to_string(int(pixbr(1, 0))) + " ]");
        std::string bl("[ " + std::to_string(int(pixbl(0, 0))) + ", " + std::to_string(int(pixbl(1, 0))) + " ]");

        // yaml population
        fout << "\n - TargetID: " << std::to_string(tag_id); 
        fout << "\n   size: " << std::to_string(tag_size);
        fout << "\n   corners:";
        fout << "\n    0: " << tl;
        fout << "\n    1: " << tr;
        fout << "\n    2: " << br;
        fout << "\n    3: " << bl;
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
