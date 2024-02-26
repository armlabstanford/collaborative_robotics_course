

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
// #include <sensor_msgs/msg/image_encodings.hpp>
#include "sensor_msgs/image_encodings.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>


//include the service for this package
#include "la_msgs/srv/ptps.hpp"



class Matching_Pix_to_Ptcld : public rclcpp::Node
{
public:
  Matching_Pix_to_Ptcld(); //constructor declaration

  
  // Make callback functions for subscribers
  void info_callback(const sensor_msgs::msg::CameraInfo & msg);
  void depth_callback(const sensor_msgs::msg::Image & msg);
  void color_image_callback(const sensor_msgs::msg::Image & msg);
  void service_callback(const std::shared_ptr<la_msgs::srv::Ptps::Request> req, std::shared_ptr<la_msgs::srv::Ptps::Response> res);
  void camera_cube_locator_marker_gen();
  

  private:
  
  // Publisher declarations
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_color_filt_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr camera_cube_locator_marker_;

  //Subscriber declaration
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

  //Rosservice
  rclcpp::Service<la_msgs::srv::Ptps>::SharedPtr service_;



  //Variables
  geometry_msgs::msg::PointStamped point_3d_cloud_; //Point in pointcloud corresponding to desired pixel
  geometry_msgs::msg::Point uv_pix_; //pixel index
  std::string color_image_topic_; // this string is over-written by the service request
  std::string depth_image_topic_; // this string is over-written by the service request
  std::string depth_img_camera_info_; // this string is over-written by the service request
  std::string registered_pt_cld_topic_; // this string is over-written by the service request
  
  
  image_geometry::PinholeCameraModel camera_model_; //Camera model, will help us with projecting the ray through the depth image
  bool depth_cam_info_ready_; //This will help us ensure we don't ask for a variable before its ready
  
  // TF Listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//{nullptr};


};



Matching_Pix_to_Ptcld::Matching_Pix_to_Ptcld() 
    : Node("Matching_Pix_to_Ptcld")
{
  
  //Class constructor
  //this is how to setup the TF buffer in a class:
  //tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  // Initialize the topic name parameter with a default value.
  this->declare_parameter<std::string>("pt_srv_color_img_topic", "/locobot/camera/color/image_raw");
  this->declare_parameter<std::string>("pt_srv_depth_img_topic", "/locobot/camera/aligned_depth_to_color/image_raw");
  this->declare_parameter<std::string>("pt_srv_depth_img_cam_info_topic", "/locobot/camera/aligned_depth_to_color/camera_info");
  this->declare_parameter<std::string>("pt_srv_reg_pt_cld_topic", "/locobot/camera/depth_registered/points");

  // Retrieve the topic name from the parameter server.
  std::string color_image_topic_;
  this->get_parameter("pt_srv_color_img_topic", color_image_topic_);

  std::string depth_image_topic_;
  this->get_parameter("pt_srv_depth_img_topic", depth_image_topic_);

  std::string depth_img_camera_info_;
  this->get_parameter("pt_srv_depth_img_cam_info_topic", depth_img_camera_info_);

  std::string registered_pt_cld_topic_;
  this->get_parameter("pt_srv_reg_pt_cld_topic", registered_pt_cld_topic_);

  // Create the publisher using the topic name from the parameter server.
  image_color_filt_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/locobot/camera/block_color_filt_img",1);
  camera_cube_locator_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/locobot/camera_cube_locator",1);

  // subscription_ = this->create_subscription<std_msgs::msg::String>(
  // "topic", 10, std::bind(&Matching_Pix_to_Ptcld::topic_callback, this, _1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_img_camera_info_, 1, std::bind(&Matching_Pix_to_Ptcld::info_callback, this, std::placeholders::_1));

  
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  depth_image_topic_, 1, std::bind(&Matching_Pix_to_Ptcld::depth_callback, this, std::placeholders::_1));
  

  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  color_image_topic_, 1, std::bind(&Matching_Pix_to_Ptcld::color_image_callback, this, std::placeholders::_1));
    
  depth_cam_info_ready_ = false; //set this to false so that depth doesn't ask for camera_model_ until its been set
  
  //IS THIS CORRECT?? - DOUBLE check
  service_ = this->create_service<la_msgs::srv::Ptps>(
  "pix_to_point_cpp", std::bind(&Matching_Pix_to_Ptcld::service_callback, this, std::placeholders::_1, std::placeholders::_2));
  
}


void Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen(){
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = point_3d_cloud_.header.frame_id;
  marker.header.stamp = this->get_clock()->now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
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
  camera_cube_locator_marker_->publish(marker);
}


void Matching_Pix_to_Ptcld::info_callback(const sensor_msgs::msg::CameraInfo &msg){
  //create a camera model from the camera info
  camera_model_.fromCameraInfo(msg);
  depth_cam_info_ready_ = true; 
}

void Matching_Pix_to_Ptcld::depth_callback(const sensor_msgs::msg::Image &msg){
  //Take the depth message, using teh 32FC1 encoding and define the depth pointer
  
   cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"cv_bridge exception: %s", e.what());
    // ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  //Access the pixel of interest
  cv::Mat depth_image = cv_ptr->image;
  float depth_value = depth_image.at<float>(uv_pix_.x,uv_pix_.y);  // access the depth value of the desired pixel
  //If the pixel that was chosen has non-zero depth, then find the point projected along the ray at that depth value
  

  if (depth_value == 0)
  {
    RCLCPP_WARN(this->get_logger(),"Skipping cause pixel had no depth");
    return;
  }else{
    if (depth_cam_info_ready_)
    {
      //Pixel has depth, now we need to find the corresponding point in the pointcloud
      //Use the camera model to get the 3D ray for the current pixel
      cv::Point2d pixel(uv_pix_.y, uv_pix_.x);
      cv::Point3d ray = camera_model_.projectPixelTo3dRay(pixel);
      //Calculate the 3D point on the ray using the depth value
      cv::Point3d point_3d = ray*depth_value;   
      geometry_msgs::msg::PointStamped point_3d_geom_msg; 
      point_3d_geom_msg.header = msg.header;
      point_3d_geom_msg.point.x = point_3d.x;
      point_3d_geom_msg.point.y = point_3d.y;
      point_3d_geom_msg.point.z = point_3d.z;
      //Transform the point to the pointcloud frame using tf
      std::string point_cloud_frame = camera_model_.tfFrame();
      // Get the camera pose in the desired reference frame
      geometry_msgs::msg::TransformStamped transform;
      try {
          transform = tf_buffer_->lookupTransform(
            point_cloud_frame, msg.header.frame_id,
            tf2::TimePointZero);

          // transform = tf_buffer_.lookupTransform(point_cloud_frame, msg->header.frame_id, ros::Time(0));
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            point_cloud_frame.c_str(), msg.header.frame_id.c_str(), ex.what());
          return;
        }
      // Transform a point cloud point
      tf2::doTransform(point_3d_geom_msg, point_3d_cloud_, transform); // syntax: (points_in, points_out, transform)
    }
  }

}


void Matching_Pix_to_Ptcld::color_image_callback(const sensor_msgs::msg::Image & msg){
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
  // Now generate lower and upper bounds of HSV to filter out cube of particular color (demonstrated - red filter)  
  cv::Mat lower_bound = cv::Mat::zeros(hsv.size(), hsv.type());
  cv::Mat upper_bound = cv::Mat::zeros(hsv.size(), hsv.type());
  lower_bound.setTo(cv::Scalar(0, 100, 20));
  upper_bound.setTo(cv::Scalar(5, 255, 255));
  // Now generate and filter to make the mask:
  cv::Mat mask;
  cv::inRange(hsv, lower_bound, upper_bound, mask);

  // ***NOW THIS CODE BELOW MAKES THE VERY STRONG ASSUMPTION THAT THERE IS ONLY ONE CUBE IN THE FIELD OF VIEW OF THE DESIRED COLOR IN THE MASK - IT AVERAGES THE MASK PIXELS TO FIND THE CENTER 
  cv::Mat mask_img; //this is the result  //Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color
  cv::bitwise_and(color_img_ptr->image, color_img_ptr->image, mask_img, mask); //https://docs.opencv.org/3.4/d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14  
  // Find the average pixel location
  int count = 0;
  int x = 0;
  int y = 0;
  for(int i = 0; i < mask_img.rows; i++)
  {
      for(int j = 0; j < mask_img.cols; j++)
      {
          if(mask_img.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
          {
              count++;
              x += i;
              y += j;
          }
      }
  }
  if (count == 0) {
  // Can't see red cube anymore
  return;
  }
  x /= count;
  y /= count;
  // Turn the average pixel location white; Make the center point pixel bright so it shows up in this image
  mask_img.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
  //Store this in global variable:  
  uv_pix_.x = x; 
  uv_pix_.y = y;
  
  //Publish the image (color img with mask applied)
  cv_bridge::CvImage cv_bridge_mask_image;
  cv_bridge_mask_image.header.stamp = this->get_clock()->now(); //ros::Time::now();
  cv_bridge_mask_image.header.frame_id = msg.header.frame_id;
  cv_bridge_mask_image.encoding = sensor_msgs::image_encodings::RGB8; //::MONO8;
  cv_bridge_mask_image.image = mask_img;
  sensor_msgs::msg::Image ros_mask_image; //now convert from cv::Mat image back to ROS sensor_msgs image
  cv_bridge_mask_image.toImageMsg(ros_mask_image);
  image_color_filt_pub_->publish(ros_mask_image);
  //Now show the cube location spherical marker: 
  Matching_Pix_to_Ptcld::camera_cube_locator_marker_gen();
}


  

void Matching_Pix_to_Ptcld::service_callback(const std::shared_ptr<la_msgs::srv::Ptps::Request> req, std::shared_ptr<la_msgs::srv::Ptps::Response> res){
  // the topic for the rgb_img should be set as a rosparam when the file is launched (this can be done in the launch file, it is not done here since the subscriber is started with the class object instantiation)
  res->ptcloud_point = point_3d_cloud_; //send the point back as a response  
  // return res;
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Matching_Pix_to_Ptcld>());
  rclcpp::shutdown();
  return 0;
}

