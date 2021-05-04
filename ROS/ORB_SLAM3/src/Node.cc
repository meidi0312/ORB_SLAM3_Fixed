#include "Node.h"

#include <iostream>

Node::Node(ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport): image_transport_(image_transport){
	name_of_node_ = ros::this_node::getName();
	node_handle_ = node_handle;
	min_observations_per_point_ = 2;
	sensor_ = sensor;
}

Node::~Node(){

	orb_slam_->Shutdown();
	orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	delete orb_slam_;

}

void Node::Init(){
	node_handle_.param<std::string>(name_of_node_+"/voc_file", voc_file_name_param_, "file_not_set");
	node_handle_.param<std::string>(name_of_node_+"/settings_file", settings_file_name_param_, "file_not_set");
	orb_slam_ = new ORB_SLAM3::System (voc_file_name_param_, settings_file_name_param_, sensor_, true);

}

void Node::Update(){
	cv::Mat position = orb_slam_->GetCurrentPosition();

	cout << "got position ok ! " << endl;

}

