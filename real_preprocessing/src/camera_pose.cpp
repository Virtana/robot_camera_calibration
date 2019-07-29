#include "ros/ros.h"
#include "fstream"
#include "iostream"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "string"
#include "Eigen/Dense"

using namespace cv;

class PoseSystem
{
  public:
  int file, world, world_loc, known_loc;
  std::vector<int> w_T_tags_id;
  std::vector<int> unknown_file;
  std::vector<double> w_T_tags_size;
  std::vector<Eigen::MatrixXd> w_T_tags_trans;

  PoseSystem()
  {
    file=0;
  }

  ~PoseSystem(){}

  //loads camera intrinsics from rosparam server
  void intrinsicLoad(cv::Mat& cam_matrix, cv::Vec<float, 5>& distCoeffs)
  {
    ros::NodeHandle NH;
    std::vector<double> intrinsic;
    NH.getParam("/camera_matrix/data", intrinsic);
    memcpy(cam_matrix.data,intrinsic.data(),intrinsic.size()*sizeof(double));
    NH.getParam("/distortion_coefficients/data", intrinsic);
    distCoeffs={intrinsic[0],intrinsic[1],intrinsic[2],intrinsic[3],intrinsic[4]};
  }

  //populates existing detection YAML fules with world T cam transforms
  void worldAppend(int filenum, Eigen::MatrixXd wTcam)
  {
    std::string filename("detections_"+std::to_string(filenum)+".yaml");  
    std::ofstream fout;

    double rotation_vec[9]= {wTcam(0, 0),wTcam(0, 1),wTcam(0, 2),
      wTcam(1, 0),wTcam(1, 1),wTcam(1, 2),
      wTcam(2, 0),wTcam(2, 1),wTcam(2, 2)};
    cv::Mat rotation_mat=cv::Mat(3,3,CV_64F,rotation_vec);
    cv::Mat rodrigues;
    cv::Rodrigues(rotation_mat,rodrigues);

    fout.open(filename.c_str(),std::ios::app);
    fout<<"world_T_camera:";
    fout<< "\n rotation: [ "+std::to_string(rodrigues.at<double>(0, 0))+" , "<< std::to_string(rodrigues.at<double>(0, 1)) +" , "+ std::to_string(rodrigues.at<double>(0, 2))+" ]";
    fout<< "\n translation: [ "+std::to_string(wTcam(0, 3))+" , "<< std::to_string(wTcam(1, 3))+" , "+ std::to_string(wTcam(2, 3))+" ]";
    fout.close();
  }

  //generates YAML file with all mappable tags and their poses relative to the world tag
  void targetDump(std::vector<int> id, std::vector<double> size, std::vector<Eigen::MatrixXd> trans)
  {
    std::ofstream fout;
    fout.open("targets.yaml");
    fout << "targets:"; 
    for(int i=0;i!=id.size();i++)
    {
      Eigen::MatrixXd tag_trans=trans[i];
      double rotation_vec[9]= {tag_trans(0, 0),tag_trans(0, 1),tag_trans(0, 2),
        tag_trans(1, 0),tag_trans(1, 1),tag_trans(1, 2),
        tag_trans(2, 0),tag_trans(2, 1),tag_trans(2, 2)};
      cv::Mat rotation_mat=cv::Mat(3,3,CV_64F,rotation_vec);
      cv::Mat rodrigues;
      cv::Rodrigues(rotation_mat,rodrigues);

      fout << "\n - targetID: " << std::to_string(id[i]);
      fout << "\n   world_T_target:";
      fout << "\n    rotation: [ "+std::to_string(rodrigues.at<double>(0, 0))+" , "<< std::to_string(rodrigues.at<double>(0, 1)) +" , "+ std::to_string(rodrigues.at<double>(0, 2))+" ]";
      fout << "\n    translation: [ "+std::to_string(tag_trans(0, 3))+" , "<< std::to_string(tag_trans(1, 3))+" , "+ std::to_string(tag_trans(2, 3))+" ]";
      fout << "\n   obj_points_in_target:";
      fout << "\n    0: [ " + std::to_string(-(size[i]/2)) + ", " + std::to_string(-(size[i]/2)) + ", "+ std::to_string(0) +" ]"; //bl
      fout << "\n    1: [ " + std::to_string(size[i]/2) + ", " + std::to_string(-(size[i]/2)) + ", "+ std::to_string(0) +" ]";  //br   
      fout << "\n    2: [ " + std::to_string(size[i]/2) + ", " + std::to_string(size[i]/2) + ", "+ std::to_string(0) +" ]";   //tr
      fout << "\n    3: [ " + std::to_string(-(size[i]/2)) + ", " + std::to_string(size[i]/2) + ", "+ std::to_string(0) +" ]";  //tl   
    }
    fout.close();
  }

  //solvePnP calculator given file number and tag position in file
  Eigen::MatrixXd objTcam(int loc, int filenum)
  {
    YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yaml");
    int bl_pix_x = tags_pix["detections"][loc]["corners"]["0"][0].as<int>();
    int bl_pix_y = tags_pix["detections"][loc]["corners"]["0"][1].as<int>();
    int br_pix_x = tags_pix["detections"][loc]["corners"]["1"][0].as<int>();
    int br_pix_y = tags_pix["detections"][loc]["corners"]["1"][1].as<int>();
    int tr_pix_x = tags_pix["detections"][loc]["corners"]["2"][0].as<int>();
    int tr_pix_y = tags_pix["detections"][loc]["corners"]["2"][1].as<int>();
    int tl_pix_x = tags_pix["detections"][loc]["corners"]["3"][0].as<int>(); 
    int tl_pix_y = tags_pix["detections"][loc]["corners"]["3"][1].as<int>();
    int tag_id = tags_pix["detections"][loc]["targetID"].as<int>();
    double tag_size = tags_pix["detections"][loc]["size"][0].as<double>();

    std::vector<cv::Point2d> img_pts;
    std::vector<cv::Point3d> obj_pts;
    cv::Mat obj_T_cam_rvec;
    cv::Mat obj_T_cam_tvec;
    cv::Mat rodrigues_rvec;
    
    img_pts.push_back(cv::Point2d(bl_pix_x, bl_pix_y));
    img_pts.push_back(cv::Point2d(br_pix_x, br_pix_y));
    img_pts.push_back(cv::Point2d(tr_pix_x, tr_pix_y));
    img_pts.push_back(cv::Point2d(tl_pix_x, tl_pix_y));

    //object corner specification
    obj_pts.push_back(cv::Point3d(-(tag_size / 2), -(tag_size / 2), 0));
    obj_pts.push_back(cv::Point3d((tag_size / 2), -(tag_size / 2), 0));
    obj_pts.push_back(cv::Point3d((tag_size / 2), (tag_size / 2), 0));
    obj_pts.push_back(cv::Point3d(-(tag_size / 2), (tag_size / 2), 0));

    cv::Mat cam_matrix(3,3,CV_64F);
    cv::Vec<float, 5> distCoeffs;
    intrinsicLoad(cam_matrix, distCoeffs);

    cv::solvePnP(obj_pts, img_pts, cam_matrix, distCoeffs, rodrigues_rvec, obj_T_cam_tvec, false, CV_ITERATIVE);
    cv::Rodrigues(rodrigues_rvec, obj_T_cam_rvec); //rotational matrix conversion
    
    Eigen::MatrixXd obj_T_cam(4, 4);
    obj_T_cam << obj_T_cam_rvec.at<double>(0, 0), obj_T_cam_rvec.at<double>(0, 1), obj_T_cam_rvec.at<double>(0, 2),
        obj_T_cam_tvec.at<double>(0, 0), obj_T_cam_rvec.at<double>(1, 0), obj_T_cam_rvec.at<double>(1, 1),
        obj_T_cam_rvec.at<double>(1, 2), obj_T_cam_tvec.at<double>(0, 1), obj_T_cam_rvec.at<double>(2, 0),
        obj_T_cam_rvec.at<double>(2, 1), obj_T_cam_rvec.at<double>(2, 2), obj_T_cam_tvec.at<double>(0, 2), 0, 0, 0, 1;
    
    return obj_T_cam.inverse().eval();
  }

  //Calculates w_T_tag for all unknown tags in a file
  void tagCalc(int filenum, int loc, bool world_pres)
  {
    YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yaml");
    Eigen::MatrixXd w_T_cam(4, 4);
    if(world_pres==0)
    {
      std::vector<int>::iterator it = std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][loc]["targetID"].as<int>());
      int index = std::distance(w_T_tags_id.begin(), it);
      w_T_cam = w_T_tags_trans[index] * objTcam(loc, filenum);
    }
    if(world_pres==1)
    {
      w_T_cam=objTcam(loc,filenum);
    }
    worldAppend(filenum,w_T_cam);
    for (int i = 0; i < tags_pix["detections"].size(); ++i)
    {
      if ((i != loc) &&
          ((std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["targetID"].as<int>()) !=
            w_T_tags_id.end()) == false))
      {
        Eigen::MatrixXd obj_T_cam(4, 4);
        obj_T_cam = objTcam(i, filenum);
        w_T_tags_trans.push_back((w_T_cam * obj_T_cam.inverse().eval()));
        w_T_tags_id.push_back(tags_pix["detections"][i]["targetID"].as<int>());
        w_T_tags_size.push_back(tags_pix["detections"][i]["size"][0].as<double>());
      }
    }
  }

  //Returns status of file : world tag present, known tag present, all unknown tags, no file
  int fileReader(int filenum)
  {
    bool known_tag(0);
    std::ifstream fin;
    fin.open("detections_" + std::to_string(filenum) + ".yaml");
    if (fin.is_open())
    {
      fin.close();
      YAML::Node tags_pix = YAML::LoadFile("detections_" + std::to_string(filenum) + ".yaml");
      if (filenum == 0)//polling first file?
      {
        world = tags_pix["detections"][0]["targetID"].as<int>(); //world tag chosen in first file/first tag listing
        world_loc = 0;
        return 0; //world tag present
      }
      for (int i = 0; i < tags_pix["detections"].size(); ++i)
      {
        if (std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["targetID"].as<int>()) !=w_T_tags_id.end())
        {
          known_tag = 1;
          known_loc = i;
        }
        if (tags_pix["detections"][i]["targetID"].as<int>() == world) 
        {
          world_loc = i;
          return 0; //world tag present
        }
      }
      if (known_tag == 1)
      {
        return 1; //known tag present
      }
      else
        return 2; //all unknown tags
    }
    else
      return 3; //file does not exist - end of YAML list
  }

  //Polls for unknown files to be reprocessed 
  void unknownPoll()
  {
    if (unknown_file.size() != 0) 
      {
        for (int i = (unknown_file.size() - 1); i >= 0; i--)
        {
          if (fileReader(unknown_file[i]) == 1)
          {
            tagCalc(unknown_file[i],known_loc,0);
            unknown_file.erase(unknown_file.begin() + i);
          }
        }
      }
  }  

  //streams all snapshot detection files for tag/pose processing 
  void fileStream()
  {
    while (fileReader(file) != 3)
    {
      if (fileReader(file) == 0) //world tag present
      {
        unknownPoll();
        tagCalc(file,world_loc,1);
        
      }
      if (fileReader(file) == 1) // known tag present
      {
        tagCalc(file,known_loc,0);
        unknownPoll();
      }
      if (fileReader(file) == 2) //all unknown tags
      {
        unknown_file.push_back(file); //stores file with all unknown tags
      }
      file++;
      if (fileReader(file) == 3) //end of YAML list 
      {
        targetDump(w_T_tags_id,w_T_tags_size,w_T_tags_trans);
      }
    }
  }

};

 int main(int argc, char** argv)
 {
  ros::init(argc, argv, "camera_pose");
  ros::NodeHandle nh;
  PoseSystem cam;
  while (ros::ok)
  {
    cam.fileStream();
  }

  return 0;
}