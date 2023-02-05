#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <map> //dictionary equivalent
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include<std_msgs/Header.h>
#define PI 3.14159265


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

//include the service
#include <me326_locobot_example/PixtoPoint.h>




class Matching_Pix_to_Ptcld
{
public:
	Matching_Pix_to_Ptcld();
	
	// Make callback functions for subscribers
	void info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg);
	void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
	void color_image_callback(const sensor_msgs::Image::ConstPtr& msg);
	bool service_callback(me326_locobot_example::PixtoPoint::Request &req, me326_locobot_example::PixtoPoint::Response &res);
	void camera_cube_locator_marker_gen();

  private:
	ros::NodeHandle nh;

  // Publisher declarations
	ros::Publisher image_color_filt_pub_;
	ros::Publisher camera_cube_locator_marker_;
	// Subscriber declarations
	ros::Subscriber cam_info_sub_;
	ros::Subscriber depth_sub_;
	ros::Subscriber rgb_image_sub_;

	// Rosservice
	ros::ServiceServer service_;

	//Variables
	geometry_msgs::PointStamped point_3d_cloud_; //Point in pointcloud corresponding to desired pixel
	geometry_msgs::Point uv_pix_; //pixel index
	std::string color_image_topic_; // this string is over-written by the service request
	std::string depth_image_topic_; // this string is over-written by the service request
	std::string depth_img_camera_info_; // this string is over-written by the service request
	image_geometry::PinholeCameraModel camera_model_;

	// TF Listener
	tf2_ros::Buffer tfBuffer;
	std::unique_ptr<tf2_ros::TransformListener> tfListener;
	tf2_ros::TransformBroadcaster tfbroadcast;

};

Matching_Pix_to_Ptcld::Matching_Pix_to_Ptcld() 
{
	//Class constructor
	nh = ros::NodeHandle("~"); //This argument makes all topics internal to this node namespace

	//ROSparam set variables
	nh.param<std::string>("pt_srv_color_img_topic", color_image_topic_, "/locobot/camera/color/image_raw");
	nh.param<std::string>("pt_srv_depth_img_topic", depth_image_topic_, "/locobot/camera/aligned_depth_to_color/image_raw");
	nh.param<std::string>("pt_srv_depth_img_cam_info_topic", depth_img_camera_info_, "/locobot/camera/aligned_depth_to_color/camera_info");

  // Publisher declarations
	ros::Publisher image_color_filt_pub_ = nh.advertise<sensor_msgs::Image>("/locobot/camera/block_color_filt_img",1);
	ros::Publisher camera_cube_locator_marker_ = nh.advertise<visualization_msgs::Marker>("/locobot/camera_cube_locator",1);
	// Subscriber declarations
	ros::Subscriber cam_info_sub_ = nh.subscribe(depth_img_camera_info_,1,&Matching_Pix_to_Ptcld::info_callback,this);
	ros::Subscriber depth_sub_ = nh.subscribe(depth_image_topic_,1,&Matching_Pix_to_Ptcld::depth_callback,this);
	ros::Subscriber rgb_image_sub_ = nh.subscribe(color_image_topic_,1,&Matching_Pix_to_Ptcld::color_image_callback,this);

	//Service
	service_ = nh.advertiseService("pix_to_point_cpp", &Matching_Pix_to_Ptcld::service_callback, this);


}

void Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = point_3d_cloud_.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	// Set the marker scale
	marker.scale.x = 0.05;  //radius of the sphere
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	// Set the marker pose
	marker.pose.position.x = point_3d_cloud_.point.x;
	marker.pose.position.y = point_3d_cloud_.point.y;
	marker.pose.position.z = point_3d_cloud_.point.z;
	// Set the marker color
	marker.color.a = 1.0; //transparency
	marker.color.r = 1.0; //red
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	// Publish the marker
	camera_cube_locator_marker_.publish(marker);
}


void Matching_Pix_to_Ptcld::info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	//create a camera model from the camera info
	camera_model_.fromCameraInfo(msg);
}

void Matching_Pix_to_Ptcld::depth_callback(const sensor_msgs::Image::ConstPtr& msg){

}

void Matching_Pix_to_Ptcld::color_image_callback(const sensor_msgs::Image::ConstPtr& msg){

}


bool Matching_Pix_to_Ptcld::service_callback(me326_locobot_example::PixtoPoint::Request &req, me326_locobot_example::PixtoPoint::Response &res){
	// Put your service logic here:
	// the topic for the rgb_img should be set as a rosparam when the file is launched (this can be done in the launch file, it is not done here since the subscriber is started with the class object instantiation)
	res.ptCld_point = point_3d_cloud_;	
	return true;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"matching_ptcld_serv");
  ros::NodeHandle nh("~");
  Matching_Pix_to_Ptcld ctd_obj;
  ros::spin();
  return 0;
}