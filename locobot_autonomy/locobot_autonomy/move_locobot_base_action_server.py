#!/usr/bin/env python3
'''
Written by: Monroe Kennedy, Date: 2/17/2024

This script demonstrates how motion of the base can be done through an action-server: 
https://docs.ros.org/en/galactic/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

'''


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from locobot_autonomy.action import MoveBase

import geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#odom from gazebo is "best effort", so this is needed for the subscriber
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

import numpy as np

#we need this since we have an action server and subscriber together
from rclpy.executors import MultiThreadedExecutor

import time


class SharedData:
    '''
    this allows us to have 'global' variables between the classes and threads
    '''
    target_pose_global = None #global variable (Pose() type)
    distance_to_goal_global = None


class MoveBaseActionServer(Node):

    def __init__(self):
        super().__init__('movebase_action_server')
        self._action_server = ActionServer(
            self,
            MoveBase,
            'movebase',
            self.execute_callback)

        #set targets for when a goal is reached: 
        self.goal_reached_error = 0.01
        #optional if you want to have a timeout:
        self.loop_duration = 10 #execute the command for up to 10 seconds or until the goal is reached


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        #obtain the goal pose provided by the client (Pose type)
        SharedData.target_pose_global = goal_handle.request.target_pose
        print("\n target_pose_global from action-server:",SharedData.target_pose_global)
        #Pause for a moment to let the faster rate odom catch up with the new target
        time.sleep(0.5)

        feedback_msg = MoveBase.Feedback() #Feedback message for action server

        distance_to_goal = SharedData.distance_to_goal_global

        while distance_to_goal is None:
            distance_to_goal = SharedData.distance_to_goal_global
            self.get_logger().info("waiting for distance to goal to be set")
            time.sleep(0.1)

        #provide feedback (distance to goal):
        while (distance_to_goal > self.goal_reached_error):# and (time_duration<self.loop_duration):
            distance_to_goal = SharedData.distance_to_goal_global
            feedback_msg.distance = distance_to_goal
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)        	
            #optional code if you want ot have a timout:
            # time_duration  = time.time() - start_time

        #return result (was goal reached)
        result = MoveBase.Result()
        if distance_to_goal < self.goal_reached_error:
            result.done = True
            goal_handle.succeed()
        else:
            result.done = False

        return result



#Node for control
class LocobotExample(Node):
    """Class for example operations for locobot control

    This script demonstrates how to move to a point using proportional/integral Control
    The pose of the Locobot is defined by its (x,y) position 
    """
    def __init__(self):
        """
        Input: Target_pose - ROS geometry_msgs.msg.Pose type
        To see what it looks like, put the following in the terminal: $ rosmsg show geometry_msgs/Pose
        """
        super().__init__('move_A_to_B_py')

        #Define the publishers
        self.mobile_base_vel_publisher = self.create_publisher(Twist,"/locobot/diffdrive_controller/cmd_vel_unstamped", 1) #this is the topic we will publish to in order to move the base
        
        # use this if odometry message is reliable: 
        #using "/locobot/sim_ground_truth_pose" because "/odom" is from wheel commands in sim is unreliable
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/locobot/sim_ground_truth_pose",
            self.odom_mobile_base_callback,
            qos_profile=qos_profile_sensor_data  # Best effort QoS profile for sensor data [usual would be queue size: 1]
            ) #this says: listen to the odom message, of type odometry, and send that to the callback function specified
        self.odom_subscription  # prevent unused variable warning


        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        #set targets for when a goal is reached: 
        self.goal_reached_error = 0.01
        self.integrated_error = np.matrix([[0],[0]]) #this is the integrated error for Proportional, Integral (PI) control
        # self.integrated_error_factor = 1.0 #multiply this by accumulated error, this is the Ki (integrated error) gain
        self.integrated_error_list = []
        self.length_of_integrated_error_list = 20



    def odom_mobile_base_callback(self, data):
   
        # Step 1: Calculate the point P location (distance L on the x-axis), and publish the marker so it can be seen in Rviz
        #first determine the relative angle of the mobile base in the world xy-plane, this angle is needed to determine where to put the point P
        #the rotation will be about the world/body z-axis, so we will only need the qw, and qz quaternion components. We can then use knoweldge of the 
        #relationship between quaternions and rotation matricies to see how we must rotate the Lx vector into the world (odom) frame and add it to the base position
        #to obtain the point P (for more info on quaterions, see a primer at the bottom of this page: https://arm.stanford.edu/resources/armlab-references)

        # self.get_logger().info("In the odom callback")
        x_data = data.pose.pose.position.x
        y_data = data.pose.pose.position.y
        z_data = data.pose.pose.position.z
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z

        #run this only if the target pose has been set
        target_pose = None
        target_pose = SharedData.target_pose_global


        if target_pose is not None:
            # self.get_logger().info("Target pose global is set in odom")

            #frame rotation:
            R11 = qw**2 + qx**2 - qy**2 -qz**2
            R12 = 2*qx*qz + 2*qw*qz
            R21 = 2*qx*qz - 2*qw*qz
            R22 = qw**2 - qx**2 + qy**2 -qz**2

            point_P = Point()
            #NOTE: the following assumes that when at the origin, the baselink and odom/world frame are aligned, and the z-axis points up. If this is not true then there is not a simple rotation about the z-axis as shown below
            point_P.x = x_data + self.L*R11
            point_P.y = y_data + self.L*R21
            point_P.z = 0.1 #make it hover just above the ground (10cm)



            # Step 2: Calculate the error between the target pose for position control (this will relate to the proportoinal gain matrix, the P in PID control)
            err_x = target_pose.position.x - point_P.x
            err_y = target_pose.position.y - point_P.y
            error_vect = np.matrix([[err_x],[err_y]]) #this is a column vector (2x1); equivalently, we could use the transpose operator (.T): np.matrix([err_x ,err_y]).T  

            Kp_mat = 1*np.eye(2) #proportional gain matrix, diagonal with gain of 0.2 (for PID control)
            Ki_mat = 0.2*np.eye(2)

            #We will deal with this later (once we reached the position (x,y) goal), but we can calculate the angular error now - again this assumes there is only planar rotation about the z-axis, and the odom/baselink frames when aligned have x,y in the plane and z pointing upwards
            Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
            R_det = np.linalg.det(Rotation_mat)
            R_rounded = np.round(Rotation_mat,decimals=3)
     
            
            # This Angle is selected because its the frame rotation angle, how does Base appear from world?
            current_angle = np.arctan2(Rotation_mat[0,1],Rotation_mat[1,1]) #this is also the angle about the z-axis of the base
            # This is the angle error: how should frame Base move to go back to world frame?
            angle_error = current_angle #access the first row, second column to get angular error (skew sym matrix of the rotation axis - here only z component, then magnitude is angle error between the current pose and the world/odom pose which we will return to both at points A and B) 
            
            Kp_angle_err = 0.2 #gain for angular error (here a scalar because we are only rotating about the z-axis)

            '''
            We do not do perform derivative control here because we are doing velocity control, 
            and an input of velocity is feedforward control, not feedback (same derivative order as input is feedforward)
            Since we are tracking a point B, we will just use PI (proportional, integral). 
            But if a trajectory were specified (we were told at each time, what the position, velocity should be) 
            then we could use feedforward control, and if we were controlling acceleration as oppose to velocity, 
            then we could use the derivative component of PID (acceleration is 2nd order control, velocity is 1st order derivative)
            '''        

            # Step 3: now put it all together to calculate the control input (velocity) based on the position error and integrated error
            self.integrated_error_list.append(error_vect)
            if len(self.integrated_error_list) > self.length_of_integrated_error_list:
                self.integrated_error_list.pop(0) #remove last element
            #now sum them
            self.integrated_error = np.matrix([[0],[0]])
            for err in self.integrated_error_list:
                self.integrated_error = self.integrated_error + err



            point_p_error_signal = Kp_mat*error_vect + Ki_mat*self.integrated_error
            #The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
            non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
            #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
            control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L
       


            #find the magnitude of the positional error to determine if its time to focus on orientation or switch targets
            err_magnitude = np.linalg.norm(error_vect)
            net_error_magnitude = np.linalg.norm(point_p_error_signal)
            
            #send the distance to the global variable:
            SharedData.distance_to_goal_global = err_magnitude

            #now let's turn this into the message type and publish it to the robot:
            control_msg = Twist()
            control_msg.linear.x = float(control_input.item(0)) #extract these elements then cast them in float type
            control_msg.angular.z = float(control_input.item(1))
            #now publish the control output:
            self.mobile_base_vel_publisher.publish(control_msg)

   




def main(args=None):
    rclpy.init(args=args)

    try:
        movebase_action_server = MoveBaseActionServer()
        locobot_control = LocobotExample()

        executor = MultiThreadedExecutor(2)
        executor.add_node(movebase_action_server)
        executor.add_node(locobot_control)


        try:
            # rclpy.spin(movebase_action_server,executor=executor)
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # node.destroy_node()
            executor.shutdown()
            movebase_action_server.destroy_node()
    finally:
        #shutdown
        rclpy.shutdown()


if __name__ == '__main__':
    main()
