#include "ros/ros.h"
#include "fstream"
#include "iostream"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "string"
#include "Eigen/Dense"
#include "ros/package.h"
#define WORLD_PRES 0
#define KNOWN_TAG 1
#define UNKNOWN 2
#define NO_FILE 3
 
using namespace cv;

class PoseSystem
{
  public:
  int file, kworld_tag;
  std::vector<int> w_T_tags_id;
  std::vector<int> unknown_file;
  std::vector<double> w_T_tags_size;
  std::vector<Eigen::MatrixXd> w_T_tags_trans;
  cv::Mat kcam_matrix;
  cv::Vec<float, 5> kdistCoeffs;

  PoseSystem()
  {
    file=0;
    kcam_matrix=cv::Mat(3,3,CV_64F,Scalar(0));
    intrinsicLoad(kcam_matrix, kdistCoeffs);
  }

  std::string pathLoad(int filenum=-1)
  {
    std::string path = ros::package::getPath("real_preprocessing");
    path=path+"/detections";
    if(filenum!=-1)
    {
      path=path+"/detections_"+std::to_string(filenum)+".yaml";
    }
    return path;
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

  //populates world tag data based on first image taken
  void worldLoad()
  {
    YAML::Node tags_pix = YAML::LoadFile(pathLoad(0));
    kworld_tag = tags_pix["detections"][0]["targetID"].as<int>(); //world tag chosen in first file/first tag listing
    Eigen::MatrixXd identity(4,4);
    identity<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    w_T_tags_trans.push_back(identity);
    w_T_tags_id.push_back(kworld_tag);
    w_T_tags_size.push_back(tags_pix["detections"][0]["size"][0].as<double>());
  }

  //populates existing detection YAML fules with world_T_cam transforms
  void worldAppend(int filenum, Eigen::MatrixXd wTcam)
  {
    std::string filename(pathLoad(filenum));  
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
  void targetDump(std::vector<int>& id, std::vector<double>& size, std::vector<Eigen::MatrixXd>& trans)
  {
    std::ofstream fout;
    fout.open(pathLoad()+"/targets.yaml");
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
  Eigen::MatrixXd tagTcam(int loc, int filenum)
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
    cv::Mat cam_T_tag_rvec;
    cv::Mat cam_T_tag_tvec;
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

    cv::solvePnP(obj_pts, img_pts, kcam_matrix, kdistCoeffs, rodrigues_rvec, cam_T_tag_tvec, false, CV_ITERATIVE);
    cv::Rodrigues(rodrigues_rvec, cam_T_tag_rvec); //rotational matrix conversion
    
    Eigen::MatrixXd cam_T_tag(4, 4);
    cam_T_tag << cam_T_tag_rvec.at<double>(0, 0), cam_T_tag_rvec.at<double>(0, 1), cam_T_tag_rvec.at<double>(0, 2),
        cam_T_tag_tvec.at<double>(0, 0), cam_T_tag_rvec.at<double>(1, 0), cam_T_tag_rvec.at<double>(1, 1),
        cam_T_tag_rvec.at<double>(1, 2), cam_T_tag_tvec.at<double>(0, 1), cam_T_tag_rvec.at<double>(2, 0),
        cam_T_tag_rvec.at<double>(2, 1), cam_T_tag_rvec.at<double>(2, 2), cam_T_tag_tvec.at<double>(0, 2), 0, 0, 0, 1;
    
    return cam_T_tag.inverse().eval();
  }

  //Calculates w_T_tag for all unknown tags in a file
  void tagCalc(int filenum, int loc, int file_type)
  {
    YAML::Node tags_pix = YAML::LoadFile(pathLoad(filenum));
    Eigen::MatrixXd w_T_cam(4, 4);
    if(file_type==KNOWN_TAG)
    {
      std::vector<int>::iterator it = std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][loc]["targetID"].as<int>());
      int index = std::distance(w_T_tags_id.begin(), it);
      w_T_cam = w_T_tags_trans[index] * tagTcam(loc, filenum);
    }
    if(file_type==WORLD_PRES)
    {
      w_T_cam=tagTcam(loc,filenum);
    }
    worldAppend(filenum,w_T_cam);
    for (int i = 0; i < tags_pix["detections"].size(); ++i)
    {
      if ((i != loc) &&
          ((std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["targetID"].as<int>()) !=
            w_T_tags_id.end()) == false))
      {
        Eigen::MatrixXd tag_T_cam(4, 4);
        tag_T_cam = tagTcam(i, filenum);
        w_T_tags_trans.push_back((w_T_cam * tag_T_cam.inverse().eval()));
        w_T_tags_id.push_back(tags_pix["detections"][i]["targetID"].as<int>());
        w_T_tags_size.push_back(tags_pix["detections"][i]["size"][0].as<double>());
      }
    }
  }
  
  //Returns status of file : world tag present, known tag present, all unknown tags, no file
  //known_tag_in_file is an updated location of the first known tag/world tag present in "detections_(filenum)"
  int fileReader(int filenum, int& known_tag_in_file)
  {
    bool known_tag(false);
    std::ifstream fin;
    fin.open(pathLoad(filenum));
    if (fin.is_open())
    {
      fin.close();
      YAML::Node tags_pix = YAML::LoadFile(pathLoad(filenum));
      if (filenum == 0)//polling first file?
      {
        worldLoad();
        known_tag_in_file = 0;
        return WORLD_PRES; //world tag present
      }
      for (int i = 0; i < tags_pix["detections"].size(); ++i)
      {
        if (std::find(w_T_tags_id.begin(), w_T_tags_id.end(), tags_pix["detections"][i]["targetID"].as<int>()) !=w_T_tags_id.end())
        {
          known_tag = true;
          known_tag_in_file = i;
        }
        if (tags_pix["detections"][i]["targetID"].as<int>() == kworld_tag) 
        {
          known_tag_in_file = i;
          return WORLD_PRES; //world tag present
        }
      }
      if (known_tag == true)
      {
        return KNOWN_TAG; //known tag present
      }
      else
        return UNKNOWN; //all unknown tags
    }
    else 
      return NO_FILE; //file does not exist - end of YAML list
  }

  //Polls for unknown files to be reprocessed 
  void unknownFilepoll()
  {
    if (unknown_file.size() != 0) 
    {
      for (int i = (unknown_file.size() - 1); i >= 0; i--)
      {
        int known_tag_in_file;
        if (fileReader(unknown_file[i],known_tag_in_file) == KNOWN_TAG)
        {
          tagCalc(unknown_file[i],known_tag_in_file, KNOWN_TAG);
          unknown_file.erase(unknown_file.begin() + i);
        }
      }
    }
  }  

  //streams all snapshot detection files for tag/pose processing 
  void fileStream()
  {
    int known_tag_in_file;
    int status = fileReader(file,known_tag_in_file);
    if (status != NO_FILE)
    {
      if ((status == WORLD_PRES)||(status == KNOWN_TAG)) //world tag or known tags present
      {
        tagCalc(file,known_tag_in_file,status);
        unknownFilepoll();
      }
      if (status == UNKNOWN) //all unknown tags
      {
        unknown_file.push_back(file); //stores file with all unknown tags
      }
      file++;
      targetDump(w_T_tags_id,w_T_tags_size,w_T_tags_trans);
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
