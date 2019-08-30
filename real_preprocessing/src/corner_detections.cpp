#include "ros/ros.h"
#include "ros/package.h"
#include "sys/stat.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "fstream"
#include "string"

int filenum(0);
std::string capture("1");

//Returns the absolute directory path for package detections folder
std::string getBasePath()
{
	return (ros::package::getPath("real_preprocessing") +"/detections");
}

// populates YAML file with corner pixel coordinates per tag
void yamlDump(double tag_size, int tag_id, std::pair<int, int> pix_coord[], std::ofstream& fout, int index)
{
  // file name generation
  std::string filepath = getBasePath() + "/detections_" + std::to_string(filenum) + ".yaml";  
  std::string filename(filepath);

  if (index == 0)
  {
    fout.open(filename.c_str());
    fout << "detections:";
  }

  // yaml population format
  fout << "\n - targetID: " << std::to_string(tag_id);
  fout << "\n   size: [ " + std::to_string(tag_size) + ", " + std::to_string(tag_size) + " ]";
  fout << "\n   corners:";
  for (int i = 0; i != 4; i++)
  {
    fout << "\n    " + std::to_string(i) + ": [ " + std::to_string(pix_coord[i].first) + ", " +
                std::to_string(pix_coord[i].second) + " ]";
  }
}

void aprilDetection(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  if ((capture == "") && (!msg->detections.empty()))  // specifies key to dump YAML provided there is tag detection
  {
    std::ofstream fout;
    for (int i = 0; i != msg->detections.size(); i++)
    {
      double tag_size = msg->detections[i].size[0];
      int tag_id = msg->detections[i].id[0];
      std::pair<int, int> pix_coord[4];
      for (int n = 0; n != 4; n++)
      {
        int pix_x = int(msg->detections[i].pixel_corners_x[n]);
        int pix_y = int(msg->detections[i].pixel_corners_y[n]);
        pix_coord[n] = std::make_pair(pix_x, pix_y);
      }
      yamlDump(tag_size, tag_id, pix_coord, fout, i);
    }
    fout << "\n";
    fout.close();
    filenum++;
    capture = "1";  // ensures input key does not force infinite loop
  }
  std::getline(std::cin, capture);  // trigger input
}

int main(int argc, char** argv)
{
  //creates folder directory for detection files if it does not exist
  int dir = mkdir(getBasePath().c_str(),0744);
  //clears existing folder directory
  std::string clr_dir = "rm -rf " + getBasePath() + "/*";
  system(clr_dir.c_str());
  

  ros::init(argc, argv, "corner_detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tag_detections", 1, aprilDetection);
  ros::spin();
  return 0;
}
