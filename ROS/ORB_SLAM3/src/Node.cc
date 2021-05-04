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
	// Static parameters
	node_handle_.param<std::string>(name_of_node_+"/voc_file", voc_file_name_param_, "file_not_set");
	node_handle_.param<std::string>(name_of_node_+"/settings_file", settings_file_name_param_, "file_not_set");
	node_handle_.param(name_of_node_+ "/publish_pointcloud", publish_pointcloud_param_, true);
	node_handle_.param(name_of_node_+ "/publish_pose", publish_pose_param_, true);
	node_handle_.param(name_of_node_+ "/publish_tf", publish_tf_param_, true);
	node_handle_.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
	node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
	node_handle_.param<std::string>(name_of_node_+ "/target_frame_id", target_frame_id_param_, "base_link");
	node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);
	orb_slam_ = new ORB_SLAM3::System (voc_file_name_param_, settings_file_name_param_, sensor_, false);

	// TF listener
	tfBuffer.reset(new tf2_ros::Buffer);
	tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

	// Publishers
	rendered_image_publisher_ = image_transport_.advertise (name_of_node_+"/debug_image", 1);
	if (publish_pointcloud_param_) {
    	map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1);
  	}

  	// Enable publishing camera's pose as PoseStamped message
  	if (publish_pose_param_) {
    	pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  	}

  	status_gba_publisher_ = node_handle_.advertise<std_msgs::Bool> (name_of_node_+"/gba_running", 1);
}

void Node::Update(){

	cv::Mat position = orb_slam_->GetCurrentPosition();

	if (!position.empty()){
		if (publish_tf_param_){
			PublishPositionAsTransform(position);
		}
	
		// if (publish_pose_param_){
		// 	PublishPositionAsPoseStamped(position);
		// }
	}
	PublishRenderedImage (orb_slam_->DrawCurrentFrame());

	if (publish_pointcloud_param_){
		PublishMapPoints (orb_slam_->GetAllMapPoints());
	}

}

/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
void Node::PublishMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points){
	sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
	map_points_publisher_.publish(cloud);
}

/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}
/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
void Node::PublishPositionAsTransform (cv::Mat position) {
  // Get transform from map to camera frame
  tf2::Transform tf_transform = TransformFromMat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id_param_, target_frame_id_param_);

  // Make message
  tf2::Stamped<tf2::Transform> tf_map2target_stamped;
  tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
  msg.child_frame_id = target_frame_id_param_;
  // Broadcast tf
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(msg);
}


/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
tf2::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}


/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
tf2::Transform Node::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
  // Transform tf_in from frame_in to frame_target
  tf2::Transform tf_map2orig = tf_in;
  tf2::Transform tf_orig2target;
  tf2::Transform tf_map2target;

  tf2::Stamped<tf2::Transform> transformStamped_temp;
  try {
    // Get the transform from camera to target
    geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
    // Convert to tf2
    tf2::fromMsg(tf_msg, transformStamped_temp);
    tf_orig2target.setBasis(transformStamped_temp.getBasis());
    tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    //ros::Duration(1.0).sleep();
    tf_orig2target.setIdentity();
  }
  /* 
    // Print debug info
    double roll, pitch, yaw;
    // Print debug map2orig
    tf2::Matrix3x3(tf_map2orig.getRotation()).getRPY(roll, pitch, yaw);
    ROS_INFO("Static transform Map to Orig [%s -> %s]",
                    map_frame_id_param_.c_str(), frame_in.c_str());
    ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                    tf_map2orig.getOrigin().x(), tf_map2orig.getOrigin().y(), tf_map2orig.getOrigin().z());
    ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                    RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
    // Print debug tf_orig2target
    tf2::Matrix3x3(tf_orig2target.getRotation()).getRPY(roll, pitch, yaw);
    ROS_INFO("Static transform Orig to Target [%s -> %s]",
                    frame_in.c_str(), frame_target.c_str());
    ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                    tf_orig2target.getOrigin().x(), tf_orig2target.getOrigin().y(), tf_orig2target.getOrigin().z());
    ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                    RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
    // Print debug map2target
    tf2::Matrix3x3(tf_map2target.getRotation()).getRPY(roll, pitch, yaw);
    ROS_INFO("Static transform Map to Target [%s -> %s]",
                    map_frame_id_param_.c_str(), frame_target.c_str());
    ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                    tf_map2target.getOrigin().x(), tf_map2target.getOrigin().y(), tf_map2target.getOrigin().z());
    ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                    RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
  */

  // Transform from map to target
  tf_map2target = tf_map2orig * tf_orig2target;
  return tf_map2target;
}

/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf2::Transform tf_position = TransformFromMat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_position_target = TransformToTarget(tf_position, camera_frame_id_param_, target_frame_id_param_);
  
  // Make message
  tf2::Stamped<tf2::Transform> tf_position_target_stamped;
  tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf2::toMsg(tf_position_target_stamped, pose_msg);
  pose_publisher_.publish(pose_msg);
}

/*
FUNCTION TAKEN FROM ORB_SLAM2_ROS.
*/
void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}