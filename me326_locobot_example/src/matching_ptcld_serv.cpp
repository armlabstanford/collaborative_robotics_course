
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


//include the service
#include <me326_locobot_example/PixtoPoint.h>




class Matching_Pix_to_Ptcld
{
public:
	Matching_Pix_to_Ptcld(ros::NodeHandle &nh);
	~Matching_Pix_to_Ptcld();

private:
	ros::NodeHandle pnh_;

};

Matching_Pix_to_Ptcld::Matching_Pix_to_Ptcld(ros::NodeHandle &nh) : pnh_(nh)
{
	//Class constructor
}

Matching_Pix_to_Ptcld::~Matching_Pix_to_Ptcld(){} //define class destructor


int main(int argc, char **argv)
{
  ros::init(argc,argv,"matching_ptcld_serv");
  ros::NodeHandle nh("~");
  Matching_Pix_to_Ptcld ctd_obj(nh);



  ros::spin();

  return 0;
}