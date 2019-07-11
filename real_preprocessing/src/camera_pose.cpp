#include "ros/ros.h"
#include "fstream"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "string"
#include "Eigen/Dense"
#include "algorithm"

using namespace cv;

int world, world_loc, known_loc;
std::vector<int> w_T_tags_id;
std::vector<int> unknown_file;
std::vector<Eigen::MatrixXd> w_T_tags_trans;
Eigen::MatrixXd obj_T_cam(4, 4);
Eigen::MatrixXd w_T_cam(4, 4);
Eigen::MatrixXd w_T_tag(4, 4);

//solvePnP calculator given file number and tag position in file
void objTcam(int loc, int filenum)
{
  //intrinsic specification
  float data_intrinsic[9] = {630.3467672874693, 0, 314.10832937485145, 0, 624.9044125522948, 241.59156511756711, 0, 0, 1};
  cv::Mat intrinsic = cv::Mat(3, 3, CV_32F, data_intrinsic);
  cv::Vec<float, 5> distCoeffs = { 0.096908, -0.1434, -0.002487, 0.001367, 0 };

  YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yml");
  int tl_pix_x = tags_pix["detections"][loc]["corners"]["0"][0].as<int>();
  int tl_pix_y = tags_pix["detections"][loc]["corners"]["0"][1].as<int>();
  int tr_pix_x = tags_pix["detections"][loc]["corners"]["1"][0].as<int>();
  int tr_pix_y = tags_pix["detections"][loc]["corners"]["1"][1].as<int>();
  int br_pix_x = tags_pix["detections"][loc]["corners"]["2"][0].as<int>();
  int br_pix_y = tags_pix["detections"][loc]["corners"]["2"][1].as<int>();
  int bl_pix_x = tags_pix["detections"][loc]["corners"]["3"][0].as<int>();
  int bl_pix_y = tags_pix["detections"][loc]["corners"]["3"][1].as<int>();
  int tag_id = tags_pix["detections"][loc]["TargetID"].as<int>();
  double tag_size = tags_pix["detections"][loc]["size"].as<double>();

  std::vector<cv::Point2d> img_pts;
  std::vector<cv::Point3d> obj_pts;
  cv::Mat rodrigues_rvec;
  cv::Mat obj_T_cam_rvec;
  cv::Mat obj_T_cam_tvec;

  img_pts.push_back(cv::Point2d(tl_pix_x, tl_pix_y));
  img_pts.push_back(cv::Point2d(tr_pix_x, tr_pix_y));
  img_pts.push_back(cv::Point2d(br_pix_x, br_pix_y));
  img_pts.push_back(cv::Point2d(bl_pix_x, bl_pix_y));

  //object corner specification
  obj_pts.push_back(cv::Point3d(-(tag_size / 2), (tag_size / 2), 0));
  obj_pts.push_back(cv::Point3d((tag_size / 2), (tag_size / 2), 0));
  obj_pts.push_back(cv::Point3d((tag_size / 2), -(tag_size / 2), 0));
  obj_pts.push_back(cv::Point3d(-(tag_size / 2), -(tag_size / 2), 0));

  cv::solvePnP(obj_pts, img_pts, intrinsic, distCoeffs, rodrigues_rvec, obj_T_cam_tvec, false, CV_ITERATIVE);
  cv::Rodrigues(rodrigues_rvec, obj_T_cam_rvec); //rotational matrix conversion

  obj_T_cam << obj_T_cam_rvec.at<double>(0, 0), obj_T_cam_rvec.at<double>(0, 1), obj_T_cam_rvec.at<double>(0, 2),
      obj_T_cam_tvec.at<double>(0, 0), obj_T_cam_rvec.at<double>(1, 0), obj_T_cam_rvec.at<double>(1, 1),
      obj_T_cam_rvec.at<double>(1, 2), obj_T_cam_tvec.at<double>(0, 1), obj_T_cam_rvec.at<double>(2, 0),
      obj_T_cam_rvec.at<double>(2, 1), obj_T_cam_rvec.at<double>(2, 2), obj_T_cam_tvec.at<double>(0, 2), 0, 0, 0, 1;
}

//Calculates w_T_tag for all unknown tags in a file with the world tag present
void calcWorld(int filenum)
{
  YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yml");
  for (int i = 0; i < tags_pix["detections"].size(); ++i)
  {
    if (i != world_loc)
    {
      objTcam(i, filenum);
      w_T_tag = w_T_cam * obj_T_cam.inverse();
      w_T_tags_trans.push_back(w_T_tag);
      w_T_tags_id.push_back(tags_pix["detections"][i]["TargetID"].as<int>());
    }
  }
}

//Calculates w_T_tag for all unknown tags in a file with one or more known tags
void calcKnown(int filenum)
{
  YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yml");
  std::vector<int>::iterator it = std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][known_loc]["TargetID"].as<int>());
  int index = std::distance(w_T_tags_id.begin(), it);
  objTcam(known_loc, filenum);
  w_T_cam = w_T_tags_trans[index] * obj_T_cam;
  for (int i = 0; i < tags_pix["detections"].size(); ++i)
  {
    if ((i != known_loc) &&
        ((std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["TargetID"].as<int>()) !=
          w_T_tags_id.end()) == false))
    {
      objTcam(i, filenum);
      w_T_tag = w_T_cam * obj_T_cam.inverse();
      w_T_tags_trans.push_back(w_T_tag);
      w_T_tags_id.push_back(tags_pix["detections"][i]["TargetID"].as<int>());
    }
  }
}

//Returns status of file : world tag present, known tag present, all unknown tags, no file
int fileReader(int filenum)
{
  bool known_tag(0);
  std::ifstream fin;
  fin.open("detections_" + std::to_string(filenum) + ".yml");
  if (fin.is_open())
  {
    fin.close();
    YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yml");
    if (filenum == 0)
    {
      world = tags_pix["detections"][0]["TargetID"].as<int>(); //world tag chosen in first file/first tag listing
      objTcam(0, 0);
      world_loc = 0;
      return 0;
    }
    for (int i = 0; i < tags_pix["detections"].size(); ++i)
    {
      if (std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["TargetID"].as<int>()) !=w_T_tags_id.end())
      {
        known_tag = 1;
        known_loc = i;
      }
      if (tags_pix["detections"][i]["TargetID"].as<int>() == world)
      {
        objTcam(i, filenum);
        world_loc = i;
        return 0;
      }
    }
    if (known_tag == 1)
    {
      return 1;
    }
    else
      return 2;
  }
  else
    return 3;
}

int main(int argc, char** argv)
{
  int file(0);
  do
  {
    if (fileReader(file) != 3)
    {
      if (fileReader(file) == 0) //world tag present
      {
        w_T_cam = obj_T_cam;
        ROS_INFO("\n");
        ROS_INFO("world_T_cam for file %i:", file);
        ROS_INFO_STREAM(w_T_cam);
        calcWorld(file);
      }
      if (fileReader(file) == 1) // known tag present
      {
        calcKnown(file);
        ROS_INFO("\n");
        ROS_INFO("world_T_cam for file %i:", file);
        ROS_INFO_STREAM(w_T_cam);
        if (unknown_file.size() != 0) //polling for unprocessed files to be reread 
        {
          for (int i = (unknown_file.size() - 1); i >= 0; i--)
          {
            if (fileReader(unknown_file[i]) == 1)
            {
              calcKnown(unknown_file[i]);
              ROS_INFO("\n");
              ROS_INFO("world_T_cam for file %i:", unknown_file[i]);
              ROS_INFO_STREAM(w_T_cam);
              unknown_file.erase(unknown_file.begin() + i);
            }
          }
        }
      }
      if (fileReader(file) == 2) //all unknown tags
      {
        unknown_file.push_back(file); //records file with all unknown tags
        ROS_INFO("File detections_%i.yml has no known tags!", file);
      }
      file++;
    }
    if (fileReader(file) == 3) //no file - end of file count
    {
      ROS_INFO("Files processed: %i", file);
    }
  } while (fileReader(file) != 3);

  return 0;
}