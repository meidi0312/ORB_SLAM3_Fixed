/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM3_ROS_NODE_H_
#define ORBSLAM3_ROS_NODE_H_


#include <vector> //done
#include <ros/ros.h> //done
#include <ros/time.h>
#include <image_transport/image_transport.h> //done
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
// #include <orb_slam2_ros/dynamic_reconfigureConfig.h>

// #include "orb_slam2_ros/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

#include "System.h" //done

class Node
{
	public:
		Node(ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
		~Node();
		void Init();		

	protected:
		void Update();
		ORB_SLAM3::System* orb_slam_;
		ros::Time current_frame_time_;
		std::string camera_info_topic_;

	private:
		ORB_SLAM3::System::eSensor sensor_;
		image_transport::ImageTransport image_transport_;
		std::string name_of_node_;
		ros::NodeHandle node_handle_;
		int min_observations_per_point_;

		std::string settings_file_name_param_;
		std::string voc_file_name_param_;

		bool publish_pointcloud_param_;
		bool publish_pose_param_;
		bool publish_tf_param_;
		bool load_map_param_;
		std::string map_frame_id_param_;
		std::string camera_frame_id_param_;
		std::string target_frame_id_param_;

		boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
		boost::shared_ptr<tf2_ros::TransformListener> tfListener;

		image_transport::Publisher rendered_image_publisher_;
		ros::Publisher map_points_publisher_;
		ros::Publisher pose_publisher_;
		ros::Publisher status_gba_publisher_;

		void PublishPositionAsTransform(cv::Mat position);
		tf2::Transform TransformFromMat(cv::Mat position_mat);
		tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);

		// void PublishMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points);
		// void LoadOrbParameters(ORB_SLAM3::ORBParameters& parameters);


};

#endif //ORBSLAM3_ROS_NODE_H_