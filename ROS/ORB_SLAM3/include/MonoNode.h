#ifndef ORBSLAM3_ROS_MONONODE_H_
#define ORBSLAM3_ROS_MONONODE_H_


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "Node.h"

class MonoNode : public Node 
{
public:
	MonoNode(const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
	~MonoNode();
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
	image_transport::Subscriber image_subscriber;
};




#endif //ORBSLAM3_ROS_MONONODE_H_