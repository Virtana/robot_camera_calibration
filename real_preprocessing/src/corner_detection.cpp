#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Dense"
#include "fstream"
#include "string"
#include "iostream"

int count(0);
std::ofstream fout;
std::vector<double> intrinsic;
std::pair<int,int> coordinate[4];
std::string capture("1");
Eigen::MatrixXd kintrinsic_mat(3, 3);

void intrinsicLoad(std::vector<double> intrinsic)
{
  ros::NodeHandle NH;
  NH.getParam("/camera_matrix/data", intrinsic);
  intrinsic_mat<<intrinsic[0],intrinsic[1],intrinsic[2],intrinsic[3],intrinsic[4],intrinsic[5],intrinsic[6],intrinsic[7],intrinsic[8];
}

//TODO: Remove copy and pastage of code in this function
std::pair<int,int>* pixCalc(Eigen::MatrixXd cam_T_tag, double tag_size) 
{
  // 4x1 (X,Y,Z,1) for each tag corner  
  Eigen::MatrixXd cornertl(4, 1);
  Eigen::MatrixXd cornertr(4, 1);
  Eigen::MatrixXd cornerbr(4, 1);
  Eigen::MatrixXd cornerbl(4, 1);
  cornertl << -(tag_size / 2), (tag_size / 2), 0, 1;
  cornertr << (tag_size / 2), (tag_size / 2), 0, 1;
  cornerbr << (tag_size / 2), -(tag_size / 2), 0, 1;
  cornerbl << -(tag_size / 2), -(tag_size / 2), 0, 1;

  // pixel coordinate calculation
  Eigen::MatrixXd pixtl(3, 1);
  Eigen::MatrixXd pixtr(3, 1);
  Eigen::MatrixXd pixbr(3, 1);
  Eigen::MatrixXd pixbl(3, 1);
  pixtl = (intrinsic_mat * cam_T_tag) * cornertl; 
  pixtr = (intrinsic_mat * cam_T_tag) * cornertr; 
  pixbr = (intrinsic_mat * cam_T_tag) * cornerbr; 
  pixbl = (intrinsic_mat * cam_T_tag) * cornerbl; 

  // coordinate scaling
  pixtl << (pixtl(0, 0) / pixtl(2, 0)), (pixtl(1, 0) / pixtl(2, 0)), 1; 
  pixtr << (pixtr(0, 0) / pixtr(2, 0)), (pixtr(1, 0) / pixtr(2, 0)), 1;
  pixbr << (pixbr(0, 0) / pixbr(2, 0)), (pixbr(1, 0) / pixbr(2, 0)), 1;
  pixbl << (pixbl(0, 0) / pixbl(2, 0)), (pixbl(1, 0) / pixbl(2, 0)), 1;

  coordinate[0]=std::make_pair(int(pixtl(0,0)),int(pixtl(1,0)));
  coordinate[1]=std::make_pair(int(pixtr(0,0)),int(pixtr(1,0)));
  coordinate[2]=std::make_pair(int(pixbr(0,0)),int(pixbr(1,0)));
  coordinate[3]=std::make_pair(int(pixbl(0,0)),int(pixbl(1,0)));
  
  return coordinate;
} 

//populates YAML file with corner pixel coordinates per tag
void yamlDump(double tag_size, int tag_id, std::pair<int,int> pix[],int det_index)
{
  //file name generation
  std::string filename("detections_"+std::to_string(count)+".yml");  
  if(det_index==0)
  {
    fout.open(filename.c_str());
    fout << "detections:";
  }

  // yaml population format
  fout << "\n - TargetID: " << std::to_string(tag_id); 
  fout << "\n   size: " << std::to_string(tag_size);
  fout << "\n   corners:";
  fout << "\n    0: [ " + std::to_string(pix[0].first) + ", " + std::to_string(pix[0].second) + " ]";
  fout << "\n    1: [ " + std::to_string(pix[1].first) + ", " + std::to_string(pix[1].second) + " ]";
  fout << "\n    2: [ " + std::to_string(pix[2].first) + ", " + std::to_string(pix[2].second) + " ]";
  fout << "\n    3: [ " + std::to_string(pix[3].first) + ", " + std::to_string(pix[3].second) + " ]";
  
}

void aprilDetection(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{ 
  if(capture=="") //specifies key to dump YAML provided there is tag detection
  {
    if (!msg->detections.empty())
    {
      for (int i = 0; i !=msg->detections.size(); i++)
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
        
        yamlDump(tag_size, tag_id, pixCalc(cam_T_tag, tag_size),i);
      }
      fout.close();
      count++;
    }
    capture = "1"; //ensures input key does not force infinite loop
  }
  std::getline(std::cin,capture); //trigger input 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "corner_detection");
  ros::NodeHandle nh;

  //load intrinsic from rosparam server
  intrinsicLoad(intrinsic);

  ros::Subscriber sub = nh.subscribe("tag_detections", 1, aprilDetection);
  ros::spin();
  return 0;
}
