#!/usr/bin/env python3
'''
Written by: Monroe Kennedy, Date: 1/2/2023
Docs: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

Example of using moveit for grasping example
'''

import sys
import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import cv2

from std_msgs.msg import Float64


class BoxPickExample(object):
	"""docstring for BoxPickExample"""
	def __init__(self, arg):
		super(BoxPickExample, self).__init__()
		self.arg = arg
		

class OrientCamera(object):
	"""docstring for OrientCamera"""
	def __init__(self, tilt_topic = "/locobot/tilt_controller/command", pan_topic = "/locobot/pan_controller/command"):		
		self.orient_pub = rospy.Publisher(tilt_topic, Float64, queue_size=1, latch=True)
		self.pan_pub = rospy.Publisher(pan_topic, Float64, queue_size=1, latch=True)

	def tilt_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.orient_pub.publish(msg)
		# print("cause orientation, msg: ", msg)

	def pan_camera(self,angle=0.5):
		msg = Float64()
		msg.data = angle
		self.pan_pub.publish(msg)


class DepthToPointCloud:
    def __init__(self,depth_topic="depth_topic", cloud_topic = "cloud"):
        self.sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.pub = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1)

    def depth_callback(self, msg):
        # Convert the depth image to a point cloud
        cloud = pc2.create_cloud_xyz32(msg.header, point_cloud2.read_points(msg, skip_nans=True))
        self.pub.publish(cloud)

        #this is done with /locobot/camera/depth_registered/points (PointCloud2 message)

# if __name__ == '__main__':
#     rospy.init_node("depth_to_pointcloud")
#     dtpc = DepthToPointCloud()
#     rospy.spin()



# import rospy
# import sensor_msgs.point_cloud2 as pc2
# import cv2
# import numpy as np
# from sensor_msgs.msg import PointCloud2

class ObjectDetection:
    def __init__(self,cloud_topic="cloud"):
        self.sub = rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_callback)
        self.sift = cv2.xfeatures2d.SIFT_create() #requires: $ pip install opencv-contrib-python
        self.descriptor_object = None
        self.descriptor_scene = None
        self.keypoints_object = None
        self.keypoints_scene = None

    def cloud_callback(self, msg):
        # Convert the point cloud to a numpy array

        #NOTE: changed skip-nans to false so that img size would be divisable, 
        #but then this causes issues in two lines for detectAndCompute
        cloud_points = np.array(list(pc2.read_points(msg, skip_nans=False)), dtype=np.float32)

        # Convert the numpy array to an OpenCV image
        cloud_image = np.reshape(cloud_points, (msg.height, msg.width, -1)) #PROBLEM HERE WITH RESHAPE , NUM PTS DONT MATCH

        # Detect keypoints in the point cloud
        self.keypoints_scene, self.descriptor_scene = self.sift.detectAndCompute(cloud_image, None)

        # Match the descriptors between the object and the scene
        bf = cv2.BFMatcher()
        matches = bf.match(self.descriptor_object, self.descriptor_scene)
        matches = sorted(matches, key = lambda x:x.distance)
        # Draw the matches
        img_matches = cv2.drawMatches(self.object_image, self.keypoints_object, cloud_image, self.keypoints_scene, matches[:10], None, flags=2)
        cv2.imshow("Matches", img_matches)
        cv2.waitKey(1)

# if __name__ == '__main__':
#     rospy.init_node("object_detection")
#     od = ObjectDetection()
#     rospy.spin()




def main():

	# cls_obj = BoxPickExample()

	rospy.init_node('locobot_moveit_example_node_py')
	moveit_commander.roscpp_initialize(sys.argv)

	robot = moveit_commander.RobotCommander() #this needs to be launched in the namespace of the robot (in this example, this is done in the launch file using 'group')
	scene = moveit_commander.PlanningSceneInterface()
	arm_group_name = "interbotix_arm" #interbotix_arm and interbotix_gripper (can see in Rviz)
	arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)

	display_trajectory_publisher = rospy.Publisher('/locobot/move_group/display_planned_path',
	                                               moveit_msgs.msg.DisplayTrajectory,
	                                               queue_size=20)


	# We can get the name of the reference frame for this robot:
	planning_frame = arm_move_group.get_planning_frame()
	print("============ Planning frame: %s" % planning_frame)

	# We can also print the name of the end-effector link for this group:
	eef_link = arm_move_group.get_end_effector_link()
	print("============ End effector link: %s" % eef_link)

	jnt_names = arm_move_group.get_active_joints()
	print("============ Armgroup joint names: %s" % jnt_names)

	# We can get a list of all the groups in the robot:
	group_names = robot.get_group_names()
	print("============ Available Planning Groups:", robot.get_group_names())

	# Sometimes for debugging it is useful to print the entire state of the
	# robot:
	print("============ Printing robot state")
	print(robot.get_current_state())
	print("\n")


	'''
	In the terminal, you can see the pose: $ rosrun tf tf_echo /locobot/base_link locobot/ee_gripper_link
	monitors the trsnform from base_link to ee_gripper_link (pose provided below)
	'''

	'''
	name: 
  - elbow
  - forearm_roll
  - gripper
  - left_finger
  - pan
  - right_finger
  - shoulder
  - tilt

  - waist
  - wheel_left_joint
  - wheel_right_joint
  - wrist_angle
  - wrist_rotate
  position: [1.058371284458718, 
  			-0.05608022936825474, 
  			-0.0028329082922571303, 
  			0.03592248883715672, 
  			0.007454119000619208, 
  			-0.035767646624090786, 
  			-0.5313552376357276, 
  			0.3490191369514095, 

  			-0.1115207331248822, 
  			0.15463634539290183, 
  			0.013049850307824684, 
  			0.9302728070281328, 
  			-0.14247350829385486]
	'''


	#start here
	joint_goal = arm_move_group.get_current_joint_values() 
	#['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
	joint_goal[0] = -0.1115207331248822 #waist
	joint_goal[1] = -0.5313552376357276 #shoulder
	joint_goal[2] = 1.058371284458718 #elbow
	joint_goal[3] = -0.05608022936825474 #forearm_roll
	joint_goal[4] = 0.9302728070281328 #wrist_angle
	joint_goal[5] = -0.14247350829385486 #wrist_rotate

	# The go command can be called with joint values, poses, or without any
	# parameters if you have already set the pose or joint target for the group
	arm_move_group.go(joint_goal, wait=True)


	# Point the camera toward the blocks
	camera_orient_obj = OrientCamera()
	camera_orient_obj.tilt_camera()



	rospy.spin()

	# orient_pub = rospy.Publisher("/locobot/tilt_controller/command", Float64, queue_size=1, latch=True)
	# msg = Float64()
	# msg.data = 0.5
	# orient_pub.publish(msg)
	# print("cause orientation, msg: ", msg)

	

	#camera encoding: 32FC1
	#color image topic: /locobot/camera/color/image_raw type: sensor_msgs/Image, size hxw = 480x640 


	# od = ObjectDetection(cloud_topic="/locobot/camera/depth_registered/points")
	# rospy.spin()

	'''
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = 0.5
	pose_goal.position.y = 0.0
	pose_goal.position.z = 0.4

	arm_move_group.set_pose_target(pose_goal)

	# now we call the planner to compute and execute the plan
	plan = arm_move_group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	arm_move_group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	arm_move_group.clear_pose_targets()
	'''

    # dtpc = DepthToPointCloud(depth_topic="depth_topic", cloud_topic = "cloud")
    # rospy.spin()

    #could also just use the following for pointclouds: 



if __name__ == '__main__':
	main()
