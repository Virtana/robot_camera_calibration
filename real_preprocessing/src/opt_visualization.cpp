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

//Returns the absolute directory path for package detections folder
std::string getBasePath()
{
	return (ros::package::getPath("real_preprocessing") +"/detections");
}

//Returns the absolute directory path for detections files
std::string getDetfilePath(int filenum)
{
	return (getBasePath() + "/detections_" + std::to_string(filenum) + ".yaml");
}

//Returns the absolute directory path for targets yaml file
std::string getTargetPath()
{
	return (getBasePath() + "/targets.yaml");
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

//generates a pose message given a tag index in the targets.yaml file
geometry_msgs::Pose LoadmarkerPose(int tag_index)
{
	geometry_msgs::Pose pose_msg;
	YAML::Node YAML_handle= YAML::LoadFile(getTargetPath());
	
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

	return pose_msg;
}

//populates a tag marker message with a world_T_tag pose of the indexed tag in the targets.yaml file 
void loadTagMarker(int tag_index, visualization_msgs::Marker& marker, geometry_msgs::Pose& pose_msg)
{
	YAML::Node YAML_handle= YAML::LoadFile(getTargetPath());
	pose_msg = LoadmarkerPose(tag_index);

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

//generates the trajectory marker with all past camera poses
visualization_msgs::Marker trajMarker(geometry_msgs::PoseArray campose_array)
{
	visualization_msgs::Marker traj_marker;
	traj_marker.header.frame_id = "world"; 
	traj_marker.header.stamp = ros::Time();
	traj_marker.ns = "camera_trajectory";
	traj_marker.id = -2;
	traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
	traj_marker.action = visualization_msgs::Marker::ADD;
	traj_marker.scale.x = 0.002;
	traj_marker.color.a = 1.0;
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
	return traj_marker;
}

geometry_msgs::Pose loadCamPose(int filenum)
{
	YAML::Node YAML_handle = YAML::LoadFile(getDetfilePath(filenum));

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

	return cam_pose;
}

//generates camera marker with world_T_cam pose from given file number 
void loadCamMarker(int filenum, visualization_msgs::Marker& cam_marker, geometry_msgs::Pose& cam_pose)
{
	cam_pose = loadCamPose(filenum);

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
	ros::Rate loop_rate(5);
	
	int filenum(0);
	visualization_msgs::MarkerArray tag_array;
	visualization_msgs::Marker cam_marker;
	visualization_msgs::Marker traj_marker;
	geometry_msgs::PoseArray tagpose_array;
	geometry_msgs::PoseArray campose_array;

	tagpose_array.header.frame_id = "world";
	campose_array.header.frame_id = "world";

	//Get the world_T_tags from the targets.yaml
	YAML::Node YAML_handle= YAML::LoadFile(getTargetPath());
	for(int tag_index = 0; tag_index != YAML_handle["targets"].size(); tag_index++) 
	{
		geometry_msgs::Pose tag_pose;
		visualization_msgs::Marker tag_marker;
		loadTagMarker(tag_index, tag_marker, tag_pose);

		tag_array.markers.push_back(tag_marker);
		tagpose_array.poses.push_back(tag_pose);
	}

	//Continuously publish the world_T_cam for each detection file and all world_T_tags
	while (ros::ok())
	{
		tag_pub.publish(tag_array); 
		tag_ps_pub.publish(tagpose_array);

		std::ifstream fin;
		fin.open(getDetfilePath(filenum));
		if (fin.is_open())
		{
			geometry_msgs::Pose cam_pose;
			visualization_msgs::Marker cam_marker;
			loadCamMarker(filenum, cam_marker, cam_pose);
			campose_array.poses.push_back(cam_pose);
			visualization_msgs::Marker traj_marker = trajMarker(campose_array); 

			cam_pub.publish(cam_marker);
			cam_ps_pub.publish(campose_array);
			cam_traj_pub.publish(traj_marker);
			
			filenum++;
			
		}
		loop_rate.sleep(); 
	}
 	return 0;
}
