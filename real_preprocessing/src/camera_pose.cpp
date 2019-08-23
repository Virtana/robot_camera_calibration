#include "ros/ros.h"
#include "fstream"
#include "iostream"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "string"
#include "Eigen/Dense"
#include "ros/package.h"

#define TARGET -1
#define WORLD_PRES 0
#define KNOWN_TAG 1
#define UNKNOWN 2
#define NO_FILE 3

//TODO: Create header file

using namespace cv;

class PoseSystem
{
  private:
  int det_file_num, kworld_tag;
  std::vector<int> w_T_tags_id;
  std::vector<int> unreferenced_files;
  std::vector<double> w_T_tags_size;
  std::vector<Eigen::MatrixXd> w_T_tags_trans;
  cv::Mat kcam_matrix;
  cv::Vec<double, 5> kdistCoeffs;

  //Returns the absolute directory path for detections/target yaml files
  std::string getFilepath(int filenum = TARGET)
  {
    std::string basepath = ros::package::getPath("real_preprocessing") +"/detections";
    if (filenum == TARGET)
    {
      return basepath + "/targets.yaml";
    }
    else
    {
      return basepath + "/detections_" + std::to_string(filenum) + ".yaml";
    }
  }

  //loads camera intrinsics from rosparam server
  void intrinsicLoad(cv::Mat& cam_matrix, cv::Vec<double, 5>& distCoeffs)
  {
    ros::NodeHandle NH;
    std::vector<double> intrinsic;
    if((NH.hasParam("/camera_matrix/data"))&&(NH.hasParam("/distortion_coefficients/data")))
    {
      NH.getParam("/camera_matrix/data", intrinsic);
      memcpy(cam_matrix.data,intrinsic.data(),intrinsic.size()*sizeof(double));
      NH.getParam("/distortion_coefficients/data", intrinsic);
      distCoeffs={intrinsic[0],intrinsic[1],intrinsic[2],intrinsic[3],intrinsic[4]};
    }
    else 
      ROS_ERROR("Camera intrinsics not loaded to parameter server!");
  }

  //populates world tag data based on first image taken
  void worldLoad()
  {
    YAML::Node YAML_handle = YAML::LoadFile(getFilepath(0));
    kworld_tag = YAML_handle["detections"][0]["targetID"].as<int>(); //world tag chosen as first tag in list
    Eigen::MatrixXd identity(4,4);
    identity<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    w_T_tags_trans.push_back(identity);
    w_T_tags_id.push_back(kworld_tag);
    w_T_tags_size.push_back(YAML_handle["detections"][0]["size"][0].as<double>());
  }

  //Appends world_T_cam transform to respective detection YAML file
  void worldAppend(int filenum, Eigen::MatrixXd w_T_cam)
  {
    std::string filename(getFilepath(filenum));  
    std::ofstream fout;

    double rotation_vec[9]= {w_T_cam(0, 0),w_T_cam(0, 1),w_T_cam(0, 2),
      w_T_cam(1, 0),w_T_cam(1, 1),w_T_cam(1, 2),
      w_T_cam(2, 0),w_T_cam(2, 1),w_T_cam(2, 2)};
    cv::Mat rotation_mat=cv::Mat(3,3,CV_64F,rotation_vec);
    cv::Mat rodrigues;
    cv::Rodrigues(rotation_mat,rodrigues);

    fout.open(filename.c_str(),std::ios::app);
    fout<<"world_T_camera:";
    fout<< "\n rotation: [ " + std::to_string(rodrigues.at<double>(0, 0)) + " , "<< std::to_string(rodrigues.at<double>(0, 1)) + " , " + std::to_string(rodrigues.at<double>(0, 2))+" ]";
    fout<< "\n translation: [ " + std::to_string(w_T_cam(0, 3)) + " , "<< std::to_string(w_T_cam(1, 3)) + " , " + std::to_string(w_T_cam(2, 3)) + " ]";
    fout.close();
  }

  //generates YAML file with all mappable tags and their poses relative to the world tag
  void targetDump(std::vector<int>& ref_tag_ids, std::vector<double>& ref_tag_sizes, std::vector<Eigen::MatrixXd>& ref_tag_trans)
  {
    std::ofstream fout;
    fout.open(getFilepath(TARGET));
    fout << "targets:"; 
    for(int tag_index=0; tag_index!=ref_tag_ids.size(); tag_index++)
    {
      Eigen::MatrixXd tag_trans=ref_tag_trans[tag_index];
      double rotation_vec[9]= {tag_trans(0, 0),tag_trans(0, 1),tag_trans(0, 2),
        tag_trans(1, 0),tag_trans(1, 1),tag_trans(1, 2),
        tag_trans(2, 0),tag_trans(2, 1),tag_trans(2, 2)};
      cv::Mat rotation_mat=cv::Mat(3,3,CV_64F,rotation_vec);
      cv::Mat rodrigues;
      cv::Rodrigues(rotation_mat,rodrigues);

      fout << "\n - targetID: " << std::to_string(ref_tag_ids[tag_index]);
      fout << "\n   world_T_target:";
      fout << "\n    rotation: [ " + std::to_string(rodrigues.at<double>(0, 0)) + " , "<< std::to_string(rodrigues.at<double>(0, 1)) + " , " + std::to_string(rodrigues.at<double>(0, 2)) + " ]";
      fout << "\n    translation: [ " + std::to_string(tag_trans(0, 3)) + " , "<< std::to_string(tag_trans(1, 3)) + " , " + std::to_string(tag_trans(2, 3)) + " ]";
      fout << "\n   obj_points_in_target:";
      fout << "\n    0: [ " + std::to_string(-(ref_tag_sizes[tag_index]/2)) + ", " + std::to_string(-(ref_tag_sizes[tag_index]/2)) + ", " + std::to_string(0) + " ]"; //bl
      fout << "\n    1: [ " + std::to_string(ref_tag_sizes[tag_index]/2) + ", " + std::to_string(-(ref_tag_sizes[tag_index]/2)) + ", " + std::to_string(0) + " ]";  //br   
      fout << "\n    2: [ " + std::to_string(ref_tag_sizes[tag_index]/2) + ", " + std::to_string(ref_tag_sizes[tag_index]/2) + ", " + std::to_string(0) + " ]";   //tr
      fout << "\n    3: [ " + std::to_string(-(ref_tag_sizes[tag_index]/2)) + ", " + std::to_string(ref_tag_sizes[tag_index]/2) + ", " + std::to_string(0) + " ]";  //tl   
    }
    fout.close();
  }

  //Calculates tag_T_cam for respective tag in specified file
  Eigen::MatrixXd tagTcam(int tag_loc, int filenum)
  {
    YAML::Node YAML_handle = YAML::LoadFile(getFilepath(filenum));
    int bl_pix_x = YAML_handle["detections"][tag_loc]["corners"]["0"][0].as<int>();
    int bl_pix_y = YAML_handle["detections"][tag_loc]["corners"]["0"][1].as<int>();
    int br_pix_x = YAML_handle["detections"][tag_loc]["corners"]["1"][0].as<int>();
    int br_pix_y = YAML_handle["detections"][tag_loc]["corners"]["1"][1].as<int>();
    int tr_pix_x = YAML_handle["detections"][tag_loc]["corners"]["2"][0].as<int>();
    int tr_pix_y = YAML_handle["detections"][tag_loc]["corners"]["2"][1].as<int>();
    int tl_pix_x = YAML_handle["detections"][tag_loc]["corners"]["3"][0].as<int>(); 
    int tl_pix_y = YAML_handle["detections"][tag_loc]["corners"]["3"][1].as<int>();
    int tag_id = YAML_handle["detections"][tag_loc]["targetID"].as<int>();
    double tag_size = YAML_handle["detections"][tag_loc]["size"][0].as<double>();

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

  //Calculates w_T_tag for all unreferenced tags in a file
  void tagCalc(int filenum, int known_tag_loc)
  {
    YAML::Node YAML_handle = YAML::LoadFile(getFilepath(filenum));
    Eigen::MatrixXd w_T_cam(4, 4);
    Eigen::MatrixXd tag_T_cam(4, 4);

    std::vector<int>::iterator it = std::find(w_T_tags_id.begin(), w_T_tags_id.end(), YAML_handle["detections"][known_tag_loc]["targetID"].as<int>());
    int index = std::distance(w_T_tags_id.begin(), it);
    w_T_cam = w_T_tags_trans[index] * tagTcam(known_tag_loc, filenum);
    worldAppend(filenum,w_T_cam);

    int total_tags_in_file = YAML_handle["detections"].size();
    for (int tag_index = 0; tag_index < total_tags_in_file; ++tag_index)
    {
      if ((tag_index != known_tag_loc) &&
          ((std::find(w_T_tags_id.begin(), w_T_tags_id.end(), YAML_handle["detections"][tag_index]["targetID"].as<int>()) !=
            w_T_tags_id.end()) == false))
      {
        tag_T_cam = tagTcam(tag_index, filenum);
        w_T_tags_trans.push_back((w_T_cam * tag_T_cam.inverse().eval()));
        w_T_tags_id.push_back(YAML_handle["detections"][tag_index]["targetID"].as<int>());
        w_T_tags_size.push_back(YAML_handle["detections"][tag_index]["size"][0].as<double>());
      }
    }
  }
  
  //Returns status of file : world tag present, known tag present, all unknown tags, no file
  //known_tag_in_file is an updated location of the last known tag or the world tag present in "detections_(filenum)"
  int fileReader(int filenum, int& known_tag_in_file)
  {
    int file_status(UNKNOWN); //initial status of file set as unknown
    std::ifstream fin;
    fin.open(getFilepath(filenum));
    if (fin.is_open())
    {
      fin.close();
      if (filenum == 0)
      {
        known_tag_in_file = 0;
        worldLoad();
        return WORLD_PRES; //world tag present
      }

      /*Despite the inclusion of the world tag in w_T_tags_id and its associated vectors, presence of the world tag is prioritised in file reading.
      Each detected tag is referenced relative to the world tag. The uncertainty in stored world_T_tags would accumulate if new unseen tags were mapped using 
      the tags that have been previously referenced to the world tag. To reduce this uncertainty in tag linking where possible, use of the
      world tag is prioritised such that all tags can be directly mapped to the this tag provided it was detected in the frame. Unseen tags should only
      be mapped using already referenced tags given that the world tag is not discernible */
      YAML::Node YAML_handle = YAML::LoadFile(getFilepath(filenum));
      int total_tags_in_file = YAML_handle["detections"].size(); 
      for (int tag_index = 0; tag_index < total_tags_in_file; ++tag_index)
      {
        if (YAML_handle["detections"][tag_index]["targetID"].as<int>() == kworld_tag) 
        {
          known_tag_in_file = tag_index;
          return WORLD_PRES; //world tag present
        }
        if (std::find(w_T_tags_id.begin(), w_T_tags_id.end(), YAML_handle["detections"][tag_index]["targetID"].as<int>()) !=w_T_tags_id.end())
        {
          known_tag_in_file = tag_index;
          file_status = KNOWN_TAG; //file status updated provided referenced file is encountered
        }
      }
      return file_status;
    }
    else 
      return NO_FILE; //file does not exist - end of YAML list
  }

  //Reprocesses recorded files with all unknown tags  
  void unknownFilepoll()
  {
    if (unreferenced_files.size() != 0) 
    {
      for (int tag_index = (unreferenced_files.size() - 1); tag_index >= 0; tag_index--)
      {
        int known_tag_in_file;
        if (fileReader(unreferenced_files[tag_index],known_tag_in_file) == KNOWN_TAG)
        {
          tagCalc(unreferenced_files[tag_index],known_tag_in_file);
          unreferenced_files.erase(unreferenced_files.begin() + tag_index);
        }
      }
    }
  }  

  public: 
  PoseSystem()
  {
    det_file_num=0;
    kcam_matrix=cv::Mat(3,3,CV_64F,Scalar(0));
    intrinsicLoad(kcam_matrix, kdistCoeffs);
  }

  ~PoseSystem(){}

  //streams all detection files for tag/pose processing 
  void fileStream()
  {
    int known_tag_in_file;
    int status = fileReader(det_file_num,known_tag_in_file);
    if (status != NO_FILE)
    {
      if ((status == WORLD_PRES)||(status == KNOWN_TAG)) //world tag or known tags present
      {
        tagCalc(det_file_num,known_tag_in_file);
        unknownFilepoll();
      }
      else if (status == UNKNOWN) //all unknown tags
      {
        unreferenced_files.push_back(det_file_num); //records file number of file with all unknown tags
      }
      det_file_num++;
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
