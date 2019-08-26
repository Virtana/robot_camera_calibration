#include "ros/ros.h"
#include "ros/package.h"
#include "fstream"
#include "string"
#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#define TARGET -1

//Returns the absolute directory path for detections/target yaml files
std::string getFilepath(int filenum=-1)
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

//converts and returns a quaternion given a rodrigues angle
Eigen::Quaterniond rodrigToQuat(double rod_vec[3])
{
	cv::Mat rodrigues=cv::Mat(1,3,CV_64F,rod_vec);
	cv::Mat rot_mat;
	cv::Rodrigues(rodrigues,rot_mat);
	Eigen::Matrix3d rot;
	rot << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
		   rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2), 
	       rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2);
	Eigen::Quaterniond quat(rot);
	return quat;
}

//populates a pose message given a tag index in the targets.yaml file
void LoadmarkerPose(int tag_index, geometry_msgs::Pose& pose_msg)
{
	YAML::Node YAML_handle= YAML::LoadFile(getFilepath());
	
	pose_msg.position.x = YAML_handle["targets"][tag_index]["world_T_target"]["translation"][0].as<double>();
	pose_msg.position.y = YAML_handle["targets"][tag_index]["world_T_target"]["translation"][1].as<double>();
	pose_msg.position.z = YAML_handle["targets"][tag_index]["world_T_target"]["translation"][2].as<double>();

	double rod_vec[3] = {YAML_handle["targets"][tag_index]["world_T_target"]["rotation"][0].as<double>(),
						 YAML_handle["targets"][tag_index]["world_T_target"]["rotation"][1].as<double>(),
						 YAML_handle["targets"][tag_index]["world_T_target"]["rotation"][2].as<double>()};
		 
	Eigen::Quaterniond quat = rodrigToQuat(rod_vec);
	pose_msg.orientation.x = quat.x();
	pose_msg.orientation.y = quat.y();
	pose_msg.orientation.z = quat.z();
	pose_msg.orientation.w = quat.w();
}

//populates a visualization marker message given a pose and tag index in the targets.yaml file 
void editMarker(int tag_index, visualization_msgs::Marker& marker, geometry_msgs::Pose& pose_msg)
{
	YAML::Node YAML_handle= YAML::LoadFile(getFilepath());

	marker.header.frame_id = "world"; 
	marker.header.stamp = ros::Time();
	marker.ns = "tag_" + std::to_string(YAML_handle["targets"][tag_index]["targetID"].as<int>());
	marker.id = YAML_handle["targets"][tag_index]["targetID"].as<int>();
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose_msg;
	marker.scale.x = YAML_handle["targets"][tag_index]["obj_points_in_target"]["2"][0].as<double>()*2;
	marker.scale.y = YAML_handle["targets"][tag_index]["obj_points_in_target"]["2"][1].as<double>()*2;
	marker.scale.z = 0.0005;
	marker.color.a = 1.0;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.lifetime = ros::Duration();
}

//updates the trajectory marker with all camera poses
void trajMarker(geometry_msgs::PoseArray& campose_array, visualization_msgs::Marker& traj_marker)
{
	traj_marker.header.frame_id = "world"; 
	traj_marker.header.stamp = ros::Time();
	traj_marker.ns = "camera_trajectory";
	traj_marker.id = -2;
	traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
	traj_marker.action = visualization_msgs::Marker::ADD;
	traj_marker.scale.x = 0.002;
	traj_marker.color.a = 1.0; // Don't forget to set the alpha!
	traj_marker.color.r = 1;
	traj_marker.color.g = 1;
	traj_marker.color.b = 0;
	traj_marker.colors.push_back(traj_marker.color);
	traj_marker.lifetime = ros::Duration();

	traj_marker.points.clear();
	for (int i = 0; i != campose_array.poses.size(); i++)
	{
		traj_marker.points.push_back(campose_array.poses[i].position);
	}
}

//populates camera marker with world_T_cam from given file number
void camUpdate(int& filenum, visualization_msgs::Marker& cam_marker, geometry_msgs::PoseArray& campose_array)
{
    std::ifstream fin;
    fin.open(getFilepath(filenum));
    if (fin.is_open())
    {
    	fin.close();
		YAML::Node YAML_handle = YAML::LoadFile(getFilepath(filenum));

		geometry_msgs::Pose cam_pose;
		cam_pose.position.x = YAML_handle["world_T_camera"]["translation"][0].as<double>();
		cam_pose.position.y = YAML_handle["world_T_camera"]["translation"][1].as<double>();
		cam_pose.position.z = YAML_handle["world_T_camera"]["translation"][2].as<double>();

		double rod_vec[3] = {YAML_handle["world_T_camera"]["rotation"][0].as<double>(),
							 YAML_handle["world_T_camera"]["rotation"][1].as<double>(),
							 YAML_handle["world_T_camera"]["rotation"][2].as<double>()};

		Eigen::Quaterniond quat = rodrigToQuat(rod_vec);
		cam_pose.orientation.x = quat.x();
		cam_pose.orientation.y = quat.y();
		cam_pose.orientation.z = quat.z();
		cam_pose.orientation.w = quat.w();
		campose_array.poses.push_back(cam_pose);
	
		cam_marker.header.frame_id = "world"; 
		cam_marker.header.stamp = ros::Time();
		cam_marker.ns = "camera";
		cam_marker.id = -1;
		cam_marker.type = visualization_msgs::Marker::CUBE;
		cam_marker.action = visualization_msgs::Marker::ADD;
		cam_marker.pose = cam_pose;
		cam_marker.scale.x = 0.08;
		cam_marker.scale.y = 0.08;
		cam_marker.scale.z = 0.04;
		cam_marker.color.a = 1.0; 
		cam_marker.color.r = 1.00;
		cam_marker.color.g = 0.41;
		cam_marker.color.b = 0.71;
		cam_marker.lifetime = ros::Duration();
		
		filenum++;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "opt_visualization");
	ros::NodeHandle nh;
	ros::Publisher tag_pub = nh.advertise<visualization_msgs::MarkerArray>("tag_markers", 1000);
	ros::Publisher tag_ps_pub = nh.advertise<geometry_msgs::PoseArray>("world_T_tags_ps", 1000);
	ros::Publisher cam_pub = nh.advertise<visualization_msgs::Marker>("cam_vis_marker", 1000);
	ros::Publisher cam_ps_pub = nh.advertise<geometry_msgs::PoseArray>("world_T_cam_ps", 1000);
	ros::Publisher cam_traj_pub = nh.advertise<visualization_msgs::Marker>("cam_traj", 1000);
	ros::Rate loop_rate(10);
	
	int filenum(0);
	visualization_msgs::MarkerArray tag_array;
	visualization_msgs::Marker cam_marker;
	visualization_msgs::Marker cam_traj;
	visualization_msgs::Marker traj_marker;
	geometry_msgs::PoseArray tagpose_array;
	geometry_msgs::PoseArray campose_array;

	tagpose_array.header.frame_id = "world";
	campose_array.header.frame_id = "world";

	while (ros::ok())
	{		
		YAML::Node YAML_handle= YAML::LoadFile(getFilepath());
	 	for(int tag_index = 0; tag_index != YAML_handle["targets"].size(); tag_index++) 
	 	{
			visualization_msgs::Marker tag_marker;
			geometry_msgs::Pose tag_marker_pose;

			LoadmarkerPose (tag_index, tag_marker_pose);
			editMarker(tag_index, tag_marker, tag_marker_pose);

			tag_array.markers.push_back(tag_marker);
			tagpose_array.poses.push_back(tag_marker_pose);
	 	}

		camUpdate(filenum, cam_marker, campose_array);
		trajMarker(campose_array, traj_marker);

		tag_pub.publish(tag_array); 
		tag_ps_pub.publish(tagpose_array);
		cam_pub.publish(cam_marker);
		cam_ps_pub.publish(campose_array);
		cam_traj_pub.publish(traj_marker);
		
		tagpose_array.poses.clear();
		tag_array.markers.clear();

		loop_rate.sleep();
	}
 	return 0;
}
