#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "fstream"
#include "string"

int count(0);
std::ofstream fout;
std::pair<int,int> coordinate[4];
std::string capture("1");

//populates YAML file with corner pixel coordinates per tag
void yamlDump(double tag_size, int tag_id, int index)
{
  //file name generation
  std::string filename("detections_"+std::to_string(count)+".yaml");  
  if(index==0)
  {
    fout.open(filename.c_str());
    fout << "detections:";
  }

  // yaml population format
  fout << "\n - targetID: " << std::to_string(tag_id); 
  fout << "\n   size: [ " + std::to_string(tag_size) + ", " + std::to_string(tag_size) + " ]";
  fout << "\n   corners:";
  fout << "\n    0: [ " + std::to_string(coordinate[0].first) + ", " + std::to_string(coordinate[0].second) + " ]";
  fout << "\n    1: [ " + std::to_string(coordinate[1].first) + ", " + std::to_string(coordinate[1].second) + " ]";
  fout << "\n    2: [ " + std::to_string(coordinate[2].first) + ", " + std::to_string(coordinate[2].second) + " ]";
  fout << "\n    3: [ " + std::to_string(coordinate[3].first) + ", " + std::to_string(coordinate[3].second) + " ]";
}

void aprilDetection(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{ 
  if(capture=="") //specifies key to dump YAML provided there is tag detection
  {
    if (!msg->detections.empty())
    {
      for (int i = 0; i !=msg->detections.size(); i++)
      {
        double tag_size = msg->detections[i].size[0];
        int tag_id = msg->detections[i].id[0];
        for(int n= 0;n!=4;n++)
        {
          int pix_x = int(msg->detections[i].pixel_corners_x[n]);
          int pix_y = int(msg->detections[i].pixel_corners_y[n]);
          coordinate[n]=std::make_pair(pix_x,pix_y);
        }
        yamlDump(tag_size, tag_id,i);
      }
      fout << "\n";
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
  ros::Subscriber sub = nh.subscribe("tag_detections", 1, aprilDetection);
  ros::spin();
  return 0;
}
