/*
Written by: Monroe Kennedy, March 2024
This node is a service that takes in a color image and depth image, and returns the 3D points of the center of the blocks of each color present in the image.
To call this service, use the following command from terminal: $ ros2 service call /pix_to_point_cpp la_msgs/srv/Ptps "{}"  
*/

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <memory>


#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <time.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <map> //dictionary equivalent

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include "tf2/exceptions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //necessary for doTransform


#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "sensor_msgs/image_encodings.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//for pausing so variables can be filled for service
#include <chrono>
#include <thread>

//include the service for this package
#include "la_msgs/srv/ptps.hpp"



class Matching_Pix_to_Ptcld : public rclcpp::Node
{
public:
  Matching_Pix_to_Ptcld(); //constructor declaration

  // Make callback functions for subscribers
  // void info_callback(const sensor_msgs::msg::CameraInfo & msg);
  void info_callback(const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg);
  void depth_callback(const sensor_msgs::msg::Image & msg);
  void color_image_callback(const sensor_msgs::msg::Image & msg);
  void service_callback(const std::shared_ptr<la_msgs::srv::Ptps::Request> req, std::shared_ptr<la_msgs::srv::Ptps::Response> res);
  void camera_cube_locator_marker_gen();
  bool blocks_of_specific_color_present(const std::shared_ptr<cv::Mat>);

  std::vector<geometry_msgs::msg::Point> blob_locator(std::shared_ptr<cv::Mat> & color_image_canvas_ptr,  
                                                      std::shared_ptr<cv::Mat> & mask_ptr);

  std::vector<geometry_msgs::msg::PointStamped> register_rgb_pix_to_depth_pts(const cv_bridge::CvImageConstPtr cv_ptr,
                                                                          std_msgs::msg::Header msg_header, 
                                                                          const std::shared_ptr<std::vector<geometry_msgs::msg::Point>> &uv_pix_list_ptr);

  // geometry_msgs::msg::PointStamped transform_point(geomtry_msgs::msg::PointStamped point_in, std::string desired_frame);



  private:
  
  rclcpp::QoS qos_; //message reliability

  // Publisher declarations
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_color_filt_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_cube_locator_marker_;

  //Subscriber declaration
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

  //Rosservice
  rclcpp::Service<la_msgs::srv::Ptps>::SharedPtr service_;



  //Variables
  geometry_msgs::msg::PointStamped point_3d_cloud_; //Point in pointcloud corresponding to desired pixel
  geometry_msgs::msg::Point uv_pix_; //pixel index
  

  bool red_blocks_present_ = false;
  bool blue_blocks_present_ = false;
  bool green_blocks_present_ = false;
  bool yellow_blocks_present_ = false;


  std::vector<geometry_msgs::msg::PointStamped> red_3d_cloud_; //red block points list
  std::vector<geometry_msgs::msg::PointStamped> blue_3d_cloud_; //blue block points list
  std::vector<geometry_msgs::msg::PointStamped> yellow_3d_cloud_; //yellow block points list
  std::vector<geometry_msgs::msg::PointStamped> green_3d_cloud_; //green block points list

  std::vector<geometry_msgs::msg::Point> red_uv_pix_list_;
  std::vector<geometry_msgs::msg::Point> blue_uv_pix_list_;
  std::vector<geometry_msgs::msg::Point> yellow_uv_pix_list_;
  std::vector<geometry_msgs::msg::Point> green_uv_pix_list_;

  std::string color_image_topic_; // topic for the color image
  std::string depth_image_topic_; // topic for the depth image
  std::string depth_img_camera_info_; // topic for the camera info
  std::string registered_pt_cld_topic_; // topic for the point cloud
  std::string desired_block_frame_; // what desired frame the blocks poses are expressed in
  
  image_geometry::PinholeCameraModel camera_model_; //Camera model, will help us with projecting the ray through the depth image
  bool depth_cam_info_ready_; //This will help us ensure we don't ask for a variable before its ready
  
  // TF Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


};



Matching_Pix_to_Ptcld::Matching_Pix_to_Ptcld() 
    : Node("Matching_Pix_to_Ptcld"), qos_(2) //qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
{
  
  //Class constructor
  //this is how to setup the TF buffer in a class:
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize the topic name parameter with a default value.
  this->declare_parameter<std::string>("pt_srv_color_img_topic", "/locobot/camera_frame_sensor/image_raw");
  this->declare_parameter<std::string>("pt_srv_depth_img_topic", "/locobot/camera_frame_sensor/depth/image_raw");
  this->declare_parameter<std::string>("pt_srv_depth_img_cam_info_topic", "/locobot/camera_frame_sensor/camera_info");
  this->declare_parameter<std::string>("pt_srv_reg_pt_cld_topic", "/locobot/camera_frame_sensor/points");

  // Retrieve the topic name from the parameter server.
  std::string color_image_topic_;
  this->get_parameter("pt_srv_color_img_topic", color_image_topic_);

  std::string depth_image_topic_;
  this->get_parameter("pt_srv_depth_img_topic", depth_image_topic_);

  std::string depth_img_camera_info_;
  this->get_parameter("pt_srv_depth_img_cam_info_topic", depth_img_camera_info_);

  std::string registered_pt_cld_topic_;
  this->get_parameter("pt_srv_reg_pt_cld_topic", registered_pt_cld_topic_);

  std::string desired_block_frame_;
  this->get_parameter("locobot/base_link", desired_block_frame_); //set the desired frame for the blocks to be expressed in the camera frame (default)

  //message reliability
  qos_.reliability(rclcpp::ReliabilityPolicy::BestEffort);


  // Create the publisher using the topic name from the parameter server.
  image_color_filt_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/locobot/camera_frame_sensor/block_color_filt_img",1);
  camera_cube_locator_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/locobot/camera_cube_locator",1);

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_img_camera_info_, qos_, std::bind(&Matching_Pix_to_Ptcld::info_callback, this, std::placeholders::_1));

  
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  depth_image_topic_, qos_, std::bind(&Matching_Pix_to_Ptcld::depth_callback, this, std::placeholders::_1));
  

  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  color_image_topic_, qos_, std::bind(&Matching_Pix_to_Ptcld::color_image_callback, this, std::placeholders::_1));
    
  depth_cam_info_ready_ = false; //set this to false so that depth doesn't ask for camera_model_ until its been set
  
  service_ = this->create_service<la_msgs::srv::Ptps>(
  "pix_to_point_cpp", std::bind(&Matching_Pix_to_Ptcld::service_callback, this, std::placeholders::_1, std::placeholders::_2));
  
}


void Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen(){
// This function will generate a marker array of spheres to represent the 3D location of the blocks in the camera frame
  bool one_block_present = true;

  visualization_msgs::msg::MarkerArray marker_array;

  int marker_id_counter = 0;

  if(red_blocks_present_){
    for (const auto& block_pt_3D : red_3d_cloud_){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = block_pt_3D.header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.id = marker_id_counter;
      marker_id_counter++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Set the marker scale
      marker.scale.x = 0.05;  //radius of the sphere
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      // Set the marker pose
      marker.pose.position.x = block_pt_3D.point.x;
      marker.pose.position.y = block_pt_3D.point.y;
      marker.pose.position.z = block_pt_3D.point.z;
      // Set the marker color
      marker.color.a = 1.0; //transparency
      marker.color.r = 1.0; //red
      marker.color.g = 0.0; //green
      marker.color.b = 0.0; //blue
      // pushback the marker
      marker_array.markers.push_back(marker);
    }
  }

  if(blue_blocks_present_){
    for (const auto& block_pt_3D : blue_3d_cloud_){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = block_pt_3D.header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.id = marker_id_counter;
      marker_id_counter++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Set the marker scale
      marker.scale.x = 0.05;  //radius of the sphere
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      // Set the marker pose
      marker.pose.position.x = block_pt_3D.point.x;
      marker.pose.position.y = block_pt_3D.point.y;
      marker.pose.position.z = block_pt_3D.point.z;
      // Set the marker color
      marker.color.a = 1.0; //transparency
      marker.color.r = 0.0; //red
      marker.color.g = 0.0; //green
      marker.color.b = 1.0; //blue
      // pushback the marker
      marker_array.markers.push_back(marker);
    }
  }

  if(green_blocks_present_){
    for (const auto& block_pt_3D : green_3d_cloud_){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = block_pt_3D.header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.id = marker_id_counter;
      marker_id_counter++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Set the marker scale
      marker.scale.x = 0.05;  //radius of the sphere
      marker.scale.y = 0.05; 
      marker.scale.z = 0.05;
      // Set the marker pose
      marker.pose.position.x = block_pt_3D.point.x;
      marker.pose.position.y = block_pt_3D.point.y;
      marker.pose.position.z = block_pt_3D.point.z;
      // Set the marker color
      marker.color.a = 1.0; //transparency
      marker.color.r = 0.0; //red
      marker.color.g = 1.0; //green
      marker.color.b = 0.0; //blue
      // pushback the marker
      marker_array.markers.push_back(marker);
    }
  }


  if(yellow_blocks_present_){
    for (const auto& block_pt_3D : yellow_3d_cloud_){
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = block_pt_3D.header.frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.id = marker_id_counter;
      marker_id_counter++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      // Set the marker scale
      marker.scale.x = 0.05;  //radius of the sphere
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      // Set the marker pose
      marker.pose.position.x = block_pt_3D.point.x;
      marker.pose.position.y = block_pt_3D.point.y;
      marker.pose.position.z = block_pt_3D.point.z;
      // Set the marker color to yellow
      marker.color.a = 1.0; //transparency
      marker.color.r = 1.0; //red
      marker.color.g = 1.0; //green
      marker.color.b = 0.0; //blue
      // pushback the marker
      marker_array.markers.push_back(marker);
    }
  }


  if(one_block_present){
    camera_cube_locator_marker_->publish(marker_array); //at least one block was present and we can publish the marker array
  }
}


void Matching_Pix_to_Ptcld::info_callback(const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg){
  //create a camera model from the camera info

  camera_model_.fromCameraInfo(msg);
  depth_cam_info_ready_ = true; 
}


std::vector<geometry_msgs::msg::PointStamped> Matching_Pix_to_Ptcld::register_rgb_pix_to_depth_pts(const cv_bridge::CvImageConstPtr cv_ptr,
                                                          std_msgs::msg::Header msg_header, 
                                                          const std::shared_ptr<std::vector<geometry_msgs::msg::Point>> &uv_pix_list_ptr){
  
  // This function will take in the depth image and the list of pixel points and return the 3D points of the center of the blocks in the camera frame

  //generate the depth image as a cv Mat object
  cv::Mat depth_image = cv_ptr->image;

  std::vector<geometry_msgs::msg::PointStamped> general_3d_cloud;

  //Iterate through each point for this given color
  for (const auto& uv_pix : *uv_pix_list_ptr ){

    float depth_value = depth_image.at<float>(uv_pix.y,uv_pix.x);  // access the depth value of the desired pixel " If matrix is of type CV_32F then use Mat.at<float>(y,x)."
    
    //If the pixel that was chosen has non-zero depth, then find the point projected along the ray at that depth value
    
    geometry_msgs::msg::PointStamped point_3d_from_ptcld;

    if (depth_value == 0)
    {
      RCLCPP_WARN(this->get_logger(),"Skipping cause pixel had no depth");
      return general_3d_cloud;
    }else{
      if (depth_cam_info_ready_)
      {
        //Pixel has depth, now we need to find the corresponding point in the pointcloud
        //Use the camera model to get the 3D ray for the current pixel
        cv::Point2d pixel(uv_pix.x, uv_pix.y); // "Returns:3d ray passing through (u,v)"
        
        cv::Point3d ray = camera_model_.projectPixelTo3dRay(pixel);
        //Calculate the 3D point on the ray using the depth value
        cv::Point3d point_3d = ray*depth_value;   
        geometry_msgs::msg::PointStamped point_3d_geom_msg; 
        point_3d_geom_msg.header = msg_header;

        std::string point_frame_id;
        if (desired_block_frame_.empty()) {
          point_frame_id = "locobot/base_link"; // service has not populated this variable yet, so use the base link frame
        } else {
          point_frame_id = desired_block_frame_; // Service has requested a specific frame
        }
        // point_3d_geom_msg.header.frame_id = point_frame_id;
        point_3d_geom_msg.point.x = point_3d.x;
        point_3d_geom_msg.point.y = point_3d.y;
        point_3d_geom_msg.point.z = point_3d.z;
        //Transform the point to the pointcloud frame using tf
        std::string point_cloud_frame = "locobot/camera_depth_link";// This needed to be fixed, camera model frame was incorrect for depth. camera_model_.tfFrame();
        // Get the camera pose in the desired reference frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
               point_frame_id, point_cloud_frame,
              tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
               point_frame_id.c_str(), point_cloud_frame.c_str(), ex.what());
            return general_3d_cloud;
          }
        // Transform a point cloud point
        tf2::doTransform(point_3d_geom_msg, point_3d_from_ptcld, transform); // syntax: (points_in, points_out, transform)
        //Put this point into the list of pointstamps:
        point_3d_from_ptcld.header.frame_id = point_frame_id;
        general_3d_cloud.push_back(point_3d_from_ptcld);
      }
    }
  }
  return general_3d_cloud;
}




void Matching_Pix_to_Ptcld::depth_callback(const sensor_msgs::msg::Image &msg){
  //Take the depth message, using the 32FC1 encoding and define the depth pointer
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"cv_bridge exception: %s", e.what());
    return;
  }

  //reset list of block center point list variables
  red_3d_cloud_.clear(); 
  blue_3d_cloud_.clear(); 
  yellow_3d_cloud_.clear(); 
  green_3d_cloud_.clear(); 

  std_msgs::msg::Header msg_header = msg.header;

  // if blocks of a given color are present, then find the corresponding depth points correlated to their center pixels
  if(red_blocks_present_){
  std::shared_ptr<std::vector<geometry_msgs::msg::Point>> red_uv_pix_list_ptr = std::make_shared<std::vector<geometry_msgs::msg::Point>>(red_uv_pix_list_);
  red_3d_cloud_ = Matching_Pix_to_Ptcld::register_rgb_pix_to_depth_pts(cv_ptr, msg_header, red_uv_pix_list_ptr);  
  }

  if(blue_blocks_present_){
    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> blue_uv_pix_list_ptr = std::make_shared<std::vector<geometry_msgs::msg::Point>>(blue_uv_pix_list_);
    blue_3d_cloud_ = Matching_Pix_to_Ptcld::register_rgb_pix_to_depth_pts(cv_ptr, msg_header, blue_uv_pix_list_ptr);
  }
  
  if(green_blocks_present_){
    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> green_uv_pix_list_ptr = std::make_shared<std::vector<geometry_msgs::msg::Point>>(green_uv_pix_list_);
    green_3d_cloud_ = Matching_Pix_to_Ptcld::register_rgb_pix_to_depth_pts(cv_ptr, msg_header, green_uv_pix_list_ptr);
  }
  
  if(yellow_blocks_present_){
    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> yellow_uv_pix_list_ptr = std::make_shared<std::vector<geometry_msgs::msg::Point>>(yellow_uv_pix_list_);
    yellow_3d_cloud_ = Matching_Pix_to_Ptcld::register_rgb_pix_to_depth_pts(cv_ptr, msg_header, yellow_uv_pix_list_ptr);
  }  
  
  //Now show the cubes' locations with a spherical marker: 
  Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen();
}

bool Matching_Pix_to_Ptcld::blocks_of_specific_color_present(const std::shared_ptr<cv::Mat> mask_img){
  // This function will take in a mask image and return a boolean value indicating if any blocks of a given color are present in the image
  // Determine if any pixel is not black/null [0,0,0]
  bool block_color_present = false;
  for(int i = 0; i < mask_img->rows; i++)
  {
      for(int j = 0; j < mask_img->cols; j++)
      {
          if(mask_img->at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
          {
            block_color_present = true;
          }
      }
  }
  return block_color_present;
}



std::vector<geometry_msgs::msg::Point> Matching_Pix_to_Ptcld::blob_locator(std::shared_ptr<cv::Mat> & color_image_canvas_ptr,  std::shared_ptr<cv::Mat> & mask_ptr){
  // This function will take in a mask image and return a list of the pixel (uv) points of the center of the blobs in the image
  
  // Find blobs and populate the uv_list:
  //Find countours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(*mask_ptr, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  //make a list of the points to return
  std::vector<geometry_msgs::msg::Point> uv_pix_list;

  for (const auto& contour : contours) {
      // Calculate the moments of the contour, then use an averaging algorithm to find the center of the contours
      cv::Moments moments = cv::moments(contour);

      if (moments.m00 != 0) {
          // Calculate the centroid of the blob
          cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
          geometry_msgs::msg::Point center_geom;
          center_geom.x = center.x;
          center_geom.y = center.y;
          uv_pix_list.push_back(center_geom);
          // Draw the center for visualization
          cv::circle(*color_image_canvas_ptr, center, 4, cv::Scalar(0, 255, 0), -1);
      }
  }
  return uv_pix_list;
}



void Matching_Pix_to_Ptcld::color_image_callback(const sensor_msgs::msg::Image & msg){
  // This function will take in a color image and return a list of the pixel (uv) points of the center of the blobs in the image
  //convert sensor_msgs image to opencv image : http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

  cv_bridge::CvImagePtr color_img_ptr;
  try
  {
    color_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //accesses image through color_img_ptr->image
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"cv_bridge exception: %s", e.what());
    return;
  }
  //Convert opencv color imgage to HSV:
  cv::Mat hsv; 
  cv::cvtColor(color_img_ptr->image, hsv, cv::COLOR_RGB2HSV); //example: https://cppsecrets.com/users/203110310511410511510410011599115495764103109971051084699111109/C00-OpenCv-cvcvtColor.php, and https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
  
  // For each color, now generate lower and upper bounds of HSV to filter out cube of particular color 
  //red:
  cv::Mat lower_bound_red = cv::Mat::zeros(hsv.size(), hsv.type());
  cv::Mat upper_bound_red = cv::Mat::zeros(hsv.size(), hsv.type());
  lower_bound_red.setTo(cv::Scalar(0, 100, 20));
  upper_bound_red.setTo(cv::Scalar(5, 255, 255));
  // Now generate and filter to make the mask:
  cv::Mat red_mask;
  cv::inRange(hsv, lower_bound_red, upper_bound_red, red_mask);
  //Blue:
    cv::Mat lower_bound_blue = cv::Mat::zeros(hsv.size(), hsv.type());
  cv::Mat upper_bound_blue = cv::Mat::zeros(hsv.size(), hsv.type());
  lower_bound_blue.setTo(cv::Scalar(100, 210, 20));
  upper_bound_blue.setTo(cv::Scalar(130, 255, 255));
  // Now generate and filter to make the mask:
  cv::Mat blue_mask;
  cv::inRange(hsv, lower_bound_blue, upper_bound_blue, blue_mask);
  //Yellow:
    cv::Mat lower_bound_yellow = cv::Mat::zeros(hsv.size(), hsv.type());
  cv::Mat upper_bound_yellow  = cv::Mat::zeros(hsv.size(), hsv.type());
  lower_bound_yellow.setTo(cv::Scalar(20, 65, 20));
  upper_bound_yellow.setTo(cv::Scalar(40, 255, 255));
  // Now generate and filter to make the mask:
  cv::Mat yellow_mask;
  cv::inRange(hsv, lower_bound_yellow, upper_bound_yellow, yellow_mask);
  //Green:
    cv::Mat lower_bound_green = cv::Mat::zeros(hsv.size(), hsv.type());
  cv::Mat upper_bound_green = cv::Mat::zeros(hsv.size(), hsv.type());
  lower_bound_green.setTo(cv::Scalar(50, 100, 20));
  upper_bound_green.setTo(cv::Scalar(80, 255, 255));
  // Now generate and filter to make the mask:
  cv::Mat green_mask;
  cv::inRange(hsv, lower_bound_green, upper_bound_green, green_mask);


  //leverage these pointers to be more memory efficient:
  std::shared_ptr<cv::Mat> red_mask_ptr = std::make_shared<cv::Mat>(red_mask);
  std::shared_ptr<cv::Mat> blue_mask_ptr = std::make_shared<cv::Mat>(blue_mask);
  std::shared_ptr<cv::Mat> green_mask_ptr = std::make_shared<cv::Mat>(green_mask);
  std::shared_ptr<cv::Mat> yellow_mask_ptr = std::make_shared<cv::Mat>(yellow_mask);

  //determine if blocks of each color are present in the current image, first reset to default values, then update
  red_blocks_present_ = false;
  blue_blocks_present_ = false;
  green_blocks_present_ = false;
  yellow_blocks_present_ = false;
  red_uv_pix_list_.clear();
  blue_uv_pix_list_.clear();
  yellow_uv_pix_list_.clear();
  green_uv_pix_list_.clear();

  //now provide the udpate:
  red_blocks_present_ = Matching_Pix_to_Ptcld::blocks_of_specific_color_present(red_mask_ptr);
  blue_blocks_present_ = Matching_Pix_to_Ptcld::blocks_of_specific_color_present(blue_mask_ptr);
  green_blocks_present_ = Matching_Pix_to_Ptcld::blocks_of_specific_color_present(green_mask_ptr);
  yellow_blocks_present_ = Matching_Pix_to_Ptcld::blocks_of_specific_color_present(yellow_mask_ptr);

  //Define a canvas to draw the tracked points on: 
  cv::Mat color_image_canvas = color_img_ptr->image;
  std::shared_ptr<cv::Mat> color_image_canvas_ptr = std::make_shared<cv::Mat>(color_image_canvas);


  // If blocks exist of each color, then find the blob centers of those blocks and populate their respective lists
  if(red_blocks_present_){
    //Find the countour and blob centers
    red_uv_pix_list_ = Matching_Pix_to_Ptcld::blob_locator(color_image_canvas_ptr, red_mask_ptr); //pass in the canvas image pointer and the list of pixel (uv) points to be populated  
  }

  if(blue_blocks_present_){
    //Find the countour and blob centers
    blue_uv_pix_list_ = Matching_Pix_to_Ptcld::blob_locator(color_image_canvas_ptr, blue_mask_ptr); //pass in the canvas image pointer and the list of pixel (uv) points to be populated  
  }

  if(green_blocks_present_){
    //Find the countour and blob centers
    green_uv_pix_list_ = Matching_Pix_to_Ptcld::blob_locator(color_image_canvas_ptr, green_mask_ptr); //pass in the canvas image pointer and the list of pixel (uv) points to be populated  
  }

  if(yellow_blocks_present_){
    //Find the countour and blob centers
    yellow_uv_pix_list_ = Matching_Pix_to_Ptcld::blob_locator(color_image_canvas_ptr, yellow_mask_ptr); //pass in the canvas image pointer and the list of pixel (uv) points to be populated  
  }
  
  //Publish the image (color img with mask applied)
  cv_bridge::CvImage cv_bridge_mask_image;
  cv_bridge_mask_image.header.stamp = this->get_clock()->now(); //ros::Time::now();
  cv_bridge_mask_image.header.frame_id = msg.header.frame_id;
  cv_bridge_mask_image.encoding = sensor_msgs::image_encodings::RGB8; //::MONO8;
  // cv_bridge_mask_image.image = mask_img;
  //Testing:
  cv_bridge_mask_image.image = color_image_canvas;
  sensor_msgs::msg::Image ros_mask_image; //now convert from cv::Mat image back to ROS sensor_msgs image
  cv_bridge_mask_image.toImageMsg(ros_mask_image);
  image_color_filt_pub_->publish(ros_mask_image);
}


  

void Matching_Pix_to_Ptcld::service_callback(const std::shared_ptr<la_msgs::srv::Ptps::Request> req, std::shared_ptr<la_msgs::srv::Ptps::Response> res){
  // This is the service callback, when requested, it will return the 3D points of the center of the blocks of each color present in the image and booleans for the presence of each color
  // the topic for the rgb_img should be set as a rosparam when the file is launched (this can be done in the launch file, it is not done here since the subscriber is started with the class object instantiation)

  //Set the desired frame for the blocks to be expressed in:
  desired_block_frame_ = req->desired_frame;



  //send the response back to the client
  res->red_present = red_blocks_present_;
  res->red_points = red_3d_cloud_; //.push_back(point_3d_cloud_); //send the point back as a response  

  res->blue_present = blue_blocks_present_;
  res->blue_points = blue_3d_cloud_; 

  res->green_present = green_blocks_present_;
  res->green_points = green_3d_cloud_;

  res->yellow_present = yellow_blocks_present_;
  res->yellow_points = yellow_3d_cloud_;

}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Matching_Pix_to_Ptcld>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

