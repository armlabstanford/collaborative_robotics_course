#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import threading

from me326_locobot_example.srv import PixtoPoint, PixtoPointResponse


class PixelCloudMatcher:
    def __init__(self, color_image_topic = "/locobot/camera/color/image_raw", depth_image_topic ="/locobot/camera/aligned_depth_to_color/image_raw", depth_img_camera_info="/locobot/camera/aligned_depth_to_color/camera_info"):
        # subscribe to depth image and camera info topics

        self.bridge = CvBridge()
        self.depth_image_topic = depth_image_topic
        self.depth_img_camera_info = depth_img_camera_info
        
        self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()

        

        self.image_color_filt_pub = rospy.Publisher("/locobot/camera/block_color_filt_img",Image,queue_size=1,latch=True)

        # create a tf listener
        self.listener = tf.TransformListener()

        self.uv_pix = [0,0] #find the pixel index

        self.color_image_topic = color_image_topic #this is over-written by the service request

        self.camera_cube_locator_marker = rospy.Publisher("/locobot/camera_cube_locator",Marker, queue_size=1)

        self.point_3d_cloud = PointStamped()

        self.point_3d_cloud_setbool = False #boolean to check if its been properly set
       
        self.info_sub = rospy.Subscriber(self.depth_img_camera_info, CameraInfo, self.info_callback)
        
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        self.image_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_image_callback)
     

    def camera_cube_locator_marker_gen(self):
        #this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
        marker = Marker()
        self.thread_lock.acquire()
        marker.header.frame_id = self.point_3d_cloud.header.frame_id #"locobot/camera_depth_link"
        self.thread_lock.release()
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        # Set the marker scale
        marker.scale.x = 0.05  # radius of the sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Set the marker pose
        self.thread_lock.acquire()
        marker.pose.position.x = self.point_3d_cloud.point.x
        marker.pose.position.y = self.point_3d_cloud.point.y
        marker.pose.position.z = self.point_3d_cloud.point.z
        self.thread_lock.release()
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.camera_cube_locator_marker.publish(marker)


    def color_image_callback(self,color_msg):
        #double check img
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
        
        hsv = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)
        lower_bound = np.array([0, 100, 20])
        upper_bound = np.array([5, 255, 255])
        #Step 2: prep the mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        index_cube = [[u,v] for u in range(mask.shape[0]) for v in range(mask.shape[1]) if mask[u,v]>0 ]

        '''
        Now the following makes the very strong assumption that there is 
        only one cube of the desired color in the field of view. 
        We then find the center of the singular cube
        '''
        center_cube_index = np.floor(np.mean(index_cube,0))
        self.thread_lock.acquire()
        self.uv_pix =[int(center_cube_index[0]),int(center_cube_index[1])]
        self.thread_lock.release()

        #Step 3: Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color 
        mask_img = cv2.bitwise_and(color_img, color_img, mask = mask)
        

        mask_img[int(center_cube_index[0]),int(center_cube_index[1])] = 100
        self.image_color_filt_pub.publish(self.bridge.cv2_to_imgmsg(mask_img, "rgb8"))

        self.camera_cube_locator_marker_gen()


    def depth_callback(self, depth_msg):
        # convert depth image message to a numpy array
        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        self.thread_lock.acquire()
        x = self.uv_pix[0]
        y = self.uv_pix[1]
        self.thread_lock.release()

        # get the depth value for the current pixel
        depth = depth_image[x, y]

        # skip pixels with no depth
        if depth == 0:
            pass
            rospy.loginfo("Skipping cause pixel had no depth")
        else:
            # use the camera model to get the 3D ray for the current pixel
            ray = self.camera_model.projectPixelTo3dRay((y, x))

            # calculate the 3D point on the ray using the depth value
            point_3d = np.array(ray) * depth

            point_3d_geom_msg = PointStamped()
            point_3d_geom_msg.header = depth_msg.header
            point_3d_geom_msg.point.x = point_3d[0]
            point_3d_geom_msg.point.y = point_3d[1]
            point_3d_geom_msg.point.z = point_3d[2]

            # transform the point to the pointcloud frame using tf
            point_cloud_frame = self.camera_model.tfFrame()
            self.thread_lock.acquire()
            self.point_3d_cloud = self.listener.transformPoint(point_cloud_frame, point_3d_geom_msg)
            self.point_3d_cloud_setbool = True
            self.thread_lock.release()

            # do something with the point in the pointcloud frame
            # rospy.loginfo("Matched pixel (%d, %d) to point in pointcloud frame: %s", x, y, self.point_3d_cloud)

    def info_callback(self, info_msg):
        # create a camera model from the camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info_msg)

    def service_callback(self,req):
        #this function takes in a requested topic for the image, and returns pixel point
        self.color_image_topic = req.rgb_img_topic
        #now call the other subscribers
        resp = PixtoPointResponse()
        resp.ptCld_point = self.point_3d_cloud
        return resp

if __name__ == "__main__":
    rospy.init_node("pixel_cloud_matcher_service")
    matcher = PixelCloudMatcher()
    s = rospy.Service('pix_to_point',PixtoPoint,matcher.service_callback)
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