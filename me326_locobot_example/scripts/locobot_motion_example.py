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

class MoveLocobotArm(object):
	"""docstring for MoveLocobotArm"""
	def __init__(self,moveit_commander=None):
		self.moveit_commander = moveit_commander
		self.robot = self.moveit_commander.RobotCommander() #this needs to be launched in the namespace of the robot (in this example, this is done in the launch file using 'group')
		self.scene = self.moveit_commander.PlanningSceneInterface()
		self.gripper_group_name = "interbotix_gripper"
		self.gripper_move_group = self.moveit_commander.MoveGroupCommander(self.gripper_group_name)

		self.arm_group_name = "interbotix_arm" #interbotix_arm and interbotix_gripper (can see in Rviz)
		self.arm_move_group = self.moveit_commander.MoveGroupCommander(self.arm_group_name)
		self.display_trajectory_publisher = rospy.Publisher('/locobot/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)
		# We can get the name of the reference frame for this robot:
		self.planning_frame = self.arm_move_group.get_planning_frame()
		# We can also print the name of the end-effector link for this group:
		self.eef_link = self.arm_move_group.get_end_effector_link()
		self.jnt_names = self.arm_move_group.get_active_joints()
		# We can get a list of all the groups in the robot:
		self.group_names = self.robot.get_group_names()


	def display_moveit_info(self):
		# We can get the name of the reference frame for this robot:
		print("============ Planning frame: %s" % self.planning_frame)
		# We can also print the name of the end-effector link for this group:
		print("============ End effector link: %s" % self.eef_link)
		print("============ Armgroup joint names: %s" % self.jnt_names)
		# We can get a list of all the groups in the robot:
		print("============ Available Planning Groups:", self.robot.get_group_names())
		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print("============ Printing robot state")
		print(self.robot.get_current_state())
		print("\n")
		
	def close_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		print("grippers",gripper_goal)
		gripper_goal[0] = 0.037
		gripper_goal[1] = -0.037
		self.gripper_move_group.go(gripper_goal, wait=True)

	def open_gripper(self):
		gripper_goal = self.gripper_move_group.get_current_joint_values()
		gripper_goal[0] = -0.037
		gripper_goal[1] = 0.037
		self.gripper_move_group.go(gripper_goal, wait=True)


	def move_arm_down_for_camera(self):
		#start here
		joint_goal = self.arm_move_group.get_current_joint_values() 
		#['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
		joint_goal[0] = -0.1115207331248822 #waist
		joint_goal[1] = -0.5313552376357276 #shoulder
		joint_goal[2] = 1.058371284458718 #elbow
		joint_goal[3] = -0.05608022936825474 #forearm_roll
		joint_goal[4] = 0.9302728070281328 #wrist_angle
		joint_goal[5] = -0.14247350829385486 #wrist_rotate

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.arm_move_group.go(joint_goal, wait=True)	

	def move_gripper_down_to_grasp(self):
		pose_goal = geometry_msgs.msg.Pose()

		pose_goal.position.x = 0.5
		pose_goal.position.y = 0.0
		pose_goal.position.z = 0.03

		v = np.matrix([0,1,0]) #pitch about y-axis
		th = 10*np.pi/180. #pitch by 45deg
		#note that no rotation is th= 0 deg

		pose_goal.orientation.x = v.item(0)*np.sin(th/2)
		pose_goal.orientation.y = v.item(1)*np.sin(th/2)
		pose_goal.orientation.z = v.item(2)*np.sin(th/2)
		pose_goal.orientation.w = np.cos(th/2)

		self.arm_move_group.set_pose_target(pose_goal)
		# now we call the planner to compute and execute the plan
		plan = self.arm_move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.arm_move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		self.arm_move_group.clear_pose_targets()




def main():


	rospy.init_node('locobot_motion_example')

	moveit_commander.roscpp_initialize(sys.argv)
	move_arm_obj = MoveLocobotArm(moveit_commander=moveit_commander)
	move_arm_obj.display_moveit_info()
	move_arm_obj.move_arm_down_for_camera()

	#Uncomment below to move gripper down for grasping (note the frame is baselink; z=0 is the ground (hitting the ground!))
	# move_arm_obj.open_gripper()
	# move_arm_obj.move_gripper_down_to_grasp()
	# move_arm_obj.close_gripper()
	# move_arm_obj.open_gripper()

	# Point the camera toward the blocks
	camera_orient_obj = OrientCamera()
	camera_orient_obj.tilt_camera()

	rospy.spin()


if __name__ == '__main__':
	main()
