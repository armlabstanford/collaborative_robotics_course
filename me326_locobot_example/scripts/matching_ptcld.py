#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import cv2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

class PixelCloudMatcher:
    def __init__(self, depth_image_topic ="/locobot/camera/aligned_depth_to_color/image_raw", depth_img_camera_info="/locobot/camera/aligned_depth_to_color/camera_info"):
        # subscribe to depth image and camera info topics
        self.depth_sub = rospy.Subscriber(depth_image_topic, Image, self.depth_callback)
        self.info_sub = rospy.Subscriber(depth_img_camera_info, CameraInfo, self.info_callback)




        # create a tf listener
        self.listener = tf.TransformListener()

    def depth_callback(self, depth_msg):
        # convert depth image message to a numpy array
        # print("height,width",depth_msg.height, depth_msg.width)
        # print("data len:",len(depth_msg.data),"first element:",depth_msg.data[0])
        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
        # depth_image = np.frombuffer(depth_msg.data, dtype=np.float64).reshape(depth_msg.height, depth_msg.width)

        # depth_image = np.frombuffer(depth_msg.data).reshape(depth_msg.height, depth_msg.width)

        # print("depth image info: ",type(depth_image),"and size: ",depth_image.shape)

        # loop through all pixels in the image
        for y in range(depth_msg.height):
            for x in range(depth_msg.width):
                # get the depth value for the current pixel
                depth = depth_image[y, x]

                # skip pixels with no depth
                if depth == 0:
                    continue

                # use the camera model to get the 3D ray for the current pixel
                ray = self.camera_model.projectPixelTo3dRay((x, y))

                # calculate the 3D point on the ray using the depth value
                # print("ray ",ray," depth ",depth, "\n type of ray:",type(ray[0]), " type of depth",type(depth))
                point_3d = np.array(ray) * depth

                point_3d_geom_msg = PointStamped()
                point_3d_geom_msg.header = depth_msg.header
                point_3d_geom_msg.point.x = point_3d[0]
                point_3d_geom_msg.point.y = point_3d[1]
                point_3d_geom_msg.point.z = point_3d[2]

                # print("\npoint_3d",point_3d_geom_msg)
                # transform the point to the pointcloud frame using tf
                point_cloud_frame = self.camera_model.tfFrame()
                # print("point cloud frame: ", point_cloud_frame)
                point_3d_cloud = self.listener.transformPoint(point_cloud_frame, point_3d_geom_msg)

                # do something with the point in the pointcloud frame
                rospy.loginfo("Matched pixel (%d, %d) to point in pointcloud frame: %s", x, y, point_3d_cloud)

    def info_callback(self, info_msg):
        # create a camera model from the camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info_msg)

if __name__ == "__main__":
    rospy.init_node("pixel_cloud_matcher")
    matcher = PixelCloudMatcher()
    rospy.spin()

'''
Matching image pixels to a registered pointcloud from a depth image can be done using the following steps:

Use the cv2 library to read in the depth image.
Use the image_geometry package to create a PinholeCameraModel object from your camera's intrinsic parameters.
Use the camera model's projectPixelTo3dRay method to convert the pixel coordinates to 3D rays in the camera frame.
Use the depth value from the depth image to determine the 3D point on the ray.
Use the tf package to get the transformation matrix between the camera frame and the pointcloud frame.
Use the numpy library to transform the 3D point from the camera frame to the pointcloud frame using the transformation matrix.
Once you've done this, you will be able to match pixels in the depth image to points in the pointcloud.

Please note that the depth image should have a 1-to-1 correspondence with the image and the pointcloud should be in the same frame as the camera.
'''