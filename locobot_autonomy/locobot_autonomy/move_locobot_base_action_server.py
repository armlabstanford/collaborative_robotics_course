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
    angle_error_global = None
    control_base_angle_bool_global = False
    angular_error_focused = False


    goal_reached_error = 0.01
    goal_stopping_distance = 0.03 #if the robot is within this distance of the goal, then we will stop the action server

    angular_goal_reached_error = 2*np.pi/180 #2 degrees

    action_server_loop_duration = 10 #execute the command for up to 10 seconds or until the goal is reached


class MoveBaseActionServer(Node):

    def __init__(self):
        super().__init__('movebase_action_server')
        self._action_server = ActionServer(
            self,
            MoveBase,
            'movebase',
            self.execute_callback)

        #set targets for when a goal is reached: 
        self.goal_reached_error = SharedData.goal_reached_error 
        self.angular_goal_reached_error = SharedData.angular_goal_reached_error

        self.stuck_distance = 0.01 #if the robot is stuck, then we will stop the action server
        self.previous_distance_to_goal = None
        #optional if you want to have a timeout:
        self.loop_duration = SharedData.action_server_loop_duration #execute the command for up to N seconds or until the goal is reached
        


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        #obtain the goal pose provided by the client (Pose type)
        SharedData.target_pose_global = goal_handle.request.target_pose
        print("\n target_pose_global from action-server:",SharedData.target_pose_global)
        #specify if the angle of the base should be controlled
        SharedData.control_base_angle_bool_global = goal_handle.request.control_base_angle_bool
        #Pause for a moment to let the faster rate odom catch up with the new target
        time.sleep(0.5)

        feedback_msg = MoveBase.Feedback() #Feedback message for action server

        distance_to_goal = SharedData.distance_to_goal_global

        if type(self.previous_distance_to_goal) == type(None):
            self.previous_distance_to_goal = distance_to_goal

        angular_distance_to_goal = np.abs(SharedData.angle_error_global)

        while distance_to_goal is None:
            distance_to_goal = SharedData.distance_to_goal_global
            self.get_logger().info("waiting for distance to goal to be set")
            time.sleep(0.1)


        #timeout code:
        start_time = time.time()

        dist_update_counter = 0

        #provide feedback (distance to goal):
        while (distance_to_goal > self.goal_reached_error) and (SharedData.angular_error_focused  == False):# and (time_duration<self.loop_duration):
            distance_to_goal = SharedData.distance_to_goal_global
            feedback_msg.distance = distance_to_goal
            self.get_logger().info('Position Feedback: {0}'.format(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)        	
            #optional code if you want ot have a timout:
            time_duration  = time.time() - start_time
            diff_distance = np.abs(distance_to_goal - self.previous_distance_to_goal)

            #if the robot is stuck, then we will stop the action server
            dist_update_counter += 1
            if dist_update_counter > 10:
                self.previous_distance_to_goal = distance_to_goal
                dist_update_counter = 0

            #determine if the robot is stuck
            if time_duration > self.loop_duration and (diff_distance < self.stuck_distance):
                result = MoveBase.Result()
                result.done = False
                self.get_logger().info('Action Server Timeout: {0}'.format(time_duration))
                goal_handle.abort()
                return result
            

        start_time = time.time()
        # while SharedData.control_base_angle_bool_global and (distance_to_goal < SharedData.goal_stopping_distance) and (angular_distance_to_goal > self.angular_goal_reached_error):
        while SharedData.control_base_angle_bool_global and (angular_distance_to_goal > self.angular_goal_reached_error):
            angular_distance_to_goal = np.abs(SharedData.angle_error_global)
            feedback_msg.angular_distance = angular_distance_to_goal
            self.get_logger().info('Angular Feedback: {0}'.format(feedback_msg.angular_distance))
            goal_handle.publish_feedback(feedback_msg)
            time_duration  = time.time() - start_time
            if time_duration > self.loop_duration:
                result = MoveBase.Result()
                result.done = False
                self.get_logger().info('Action Server Timeout in Angular control: {0}'.format(time_duration))
                goal_handle.abort()
                return result

        #return result (was goal reached)
        result = MoveBase.Result()


        if (distance_to_goal < SharedData.goal_stopping_distance) and (angular_distance_to_goal < self.angular_goal_reached_error) and (SharedData.control_base_angle_bool_global==True):
            #the goal_stopping_distance is used to determine if the robot is close enough to the goal to stop the action server, as motion of the angular control may introduce some position error
            result.done = True
            goal_handle.succeed()
        elif (SharedData.control_base_angle_bool_global==False) and (distance_to_goal < SharedData.goal_stopping_distance):
            #the goal_stopping_distance is used to determine if the robot is close enough to the goal to stop the action server, 
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


        self.L = 0.02 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base
        #set targets for when a goal is reached: 
        self.goal_reached_error = SharedData.goal_reached_error #0.01
        self.integrated_error = np.matrix([[0],[0]]) #this is the integrated error for Proportional, Integral (PI) control

        self.Kp_mat = 0.5*np.eye(2) #proportional gain matrix, diagonal with gain of 0.2 (for PID control)
        self.Ki_mat = 0.3*np.eye(2) #gain for integral control (for PI control)


        self.integrated_error_angle = 0
        self.max_velocity = 0.5 #m/s
        self.Ki_angle = 0.4 # 0.08 #WAS 0.05 #integral gain for angular error

        self.integrated_error_list = []
        self.integrated_error_angle_list = []
        self.length_of_integrated_error_list = 15 #25 #50
        self.length_of_integrated_error_list_angle = 30 #20

        #For the angular error:
        self.angular_error_focused = False

        self.log_stop_cond_bool = True
       
    def angle_error_calculator(self,angle_des,angle_current):
        '''
        This function calculates the error between the desired angle and the current angle
        '''
        error = angle_des - angle_current
        # Normalize the error to be between -pi and pi
        if error > np.pi:
            error -= 2 * np.pi
        elif error < -np.pi:
            error += 2 * np.pi
        return error
    
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


            # target pose
            qw_des = target_pose.orientation.w
            qx_des = target_pose.orientation.x
            qy_des = target_pose.orientation.y
            qz_des = target_pose.orientation.z
            
            #obtain the rotation matrix for the desired orientation
            R11_des = qw_des**2 + qx_des**2 - qy_des**2 -qz_des**2
            R12_des = 2*qx_des*qz_des + 2*qw_des*qz_des
            R21_des = 2*qx_des*qz_des - 2*qw_des*qz_des
            R22_des = qw_des**2 - qx_des**2 + qy_des**2 -qz_des**2
            Rotation_mat_des = np.matrix([[R11_des,R12_des],[R21_des,R22_des]])
            
            # Step 2: Calculate the error between the target pose for position control (this will relate to the proportoinal gain matrix, the P in PID control)
            err_x = target_pose.position.x - point_P.x
            err_y = target_pose.position.y - point_P.y
            error_vect = np.matrix([[err_x],[err_y]]) #this is a column vector (2x1); equivalently, we could use the transpose operator (.T): np.matrix([err_x ,err_y]).T  

            
            #We will deal with this later (once we reached the position (x,y) goal), but we can calculate the angular error now - again this assumes there is only planar rotation about the z-axis, and the odom/baselink frames when aligned have x,y in the plane and z pointing upwards
            Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
            R_det = np.linalg.det(Rotation_mat)
            R_rounded = np.round(Rotation_mat,decimals=3)

            # This Angle is selected because its the frame rotation angle, how does Base appear from world?
            current_angle = np.arctan2(Rotation_mat[0,1],Rotation_mat[1,1]) #this is also the angle about the z-axis of the base

            angle_des = np.arctan2(Rotation_mat_des[0,1],Rotation_mat_des[1,1]) #this is also the angle about the z-axis of the base
            # Find the error between the desired angle and the current angle
            angle_error = self.angle_error_calculator(angle_des,current_angle)
            
            # self.get_logger().info('/* log */ angle_des: {0}'.format(angle_des))
            # self.get_logger().info('/* log */ current_angle: {0}'.format(current_angle))
            # self.get_logger().info('/* log */ angle_error: {0}'.format(angle_error))
            
            SharedData.angle_error_global = angle_error
            Kp_angle_err = 1 #gain for angular error (here a scalar because we are only rotating about the z-axis)

            '''
            We do not do perform derivative control here because we are doing velocity control, 
            and an input of velocity is feedforward control, not feedback (same derivative order as input is feedforward)
            Since we are tracking a point B, we will just use PI (proportional, integral). 
            But if a trajectory were specified (we were told at each time, what the position, velocity should be) 
            then we could use feedforward control, and if we were controlling acceleration as oppose to velocity, 
            then we could use the derivative component of PID (acceleration is 2nd order control, velocity is 1st order derivative)
            '''        

            #find the magnitude of the positional error to determine if its time to focus on orientation or switch targets
            err_magnitude = np.linalg.norm(error_vect)
            
            #send the distance to the global variable:
            SharedData.distance_to_goal_global = err_magnitude

            # Step 3: now put it all together to calculate the control input (velocity) based on the position error and integrated error
            self.integrated_error_list.append(error_vect)
            if len(self.integrated_error_list) > self.length_of_integrated_error_list:
                self.integrated_error_list.pop(0) #remove last element
            #now sum them
            self.integrated_error = np.matrix([[0],[0]])
            for err in self.integrated_error_list:
                self.integrated_error = self.integrated_error + err

            #for the angle integrated error:
            self.integrated_error_angle_list.append(angle_error)
            if len(self.integrated_error_angle_list) > self.length_of_integrated_error_list_angle:
                self.integrated_error_angle_list.pop(0)
            #now sum them
            self.integrated_error_angle = 0
            for err in self.integrated_error_angle_list:
                self.integrated_error_angle = self.integrated_error_angle + err

            #if the magnitude of the error is low enough, then set the integrated error to zero
            if err_magnitude < SharedData.goal_reached_error:
                self.integrated_error = np.matrix([[0],[0]])
                self.integrated_error_list = []
            if np.abs(angle_error) < SharedData.angular_goal_reached_error:
                self.integrated_error_angle = 0
                self.integrated_error_angle_list = []

            # self.get_logger().info('/* log */ integrated_error_angle: {0}'.format(self.integrated_error_angle))
            # self.get_logger().info('/* log */ integrated_error: {0}'.format(self.integrated_error))

            point_p_error_signal = self.Kp_mat*error_vect + self.Ki_mat*self.integrated_error
            #ensure that the velocity is not greater than the maximum velocity
            point_p_error_signal_magnitude = np.linalg.norm(point_p_error_signal)

            if point_p_error_signal_magnitude > self.max_velocity:
                point_p_adjusted_norm = self.max_velocity
                self.get_logger().info('/* log */ velocity magnitude capped:: {0}'.format(point_p_error_signal_magnitude))
                point_p_error_signal = point_p_error_signal/point_p_error_signal_magnitude*point_p_adjusted_norm
            #The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
            non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
            #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
            control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L

            #now let's turn this into the message type and publish it to the robot:
            control_msg = Twist()

            #if we have reached the goal, then we will focus on the orientation
            if (err_magnitude < self.goal_reached_error) and (SharedData.control_base_angle_bool_global):
                self.angular_error_focused = True
                SharedData.angular_error_focused = True

            #if we are far from the goal, then we will not focus on the orientation
            if err_magnitude > SharedData.goal_stopping_distance: 
                self.angular_error_focused = False
                SharedData.angular_error_focused = False

            if (SharedData.control_base_angle_bool_global) and (self.angular_error_focused == True):    
                # self.get_logger().info('/* log */ Goal Reached, now focusing on orientation')
                #we have reached the goal, now focus on the orientation
                control_input[0] = 0 #stop moving forward
                # self.get_logger().info('/* log */ angle_error: {0}'.format(angle_error))
                control_input[1] =  Kp_angle_err*angle_error + self.Ki_angle* self.integrated_error_angle #use the angle error to control the angular velocity
            # self.get_logger().info("error magnitude: {0}".format(err_magnitude))

            #check the condition for performing angluar control
            # self.get_logger().info('/* log */ SharedData.angular_error_focused: {0}'.format(SharedData.angular_error_focused))

            #check if there is a stopping condition, if so command the robot to stop
            if (err_magnitude < SharedData.goal_stopping_distance) and (np.abs(angle_error) < SharedData.angular_goal_reached_error) and self.angular_error_focused == True:
                control_input[0] = 0
                control_input[1] = 0
                if self.log_stop_cond_bool == True:
                    self.get_logger().info("Stopping condition met")
                    self.log_stop_cond_bool = False
                # self.get_logger().info("Stopping condition met")
                self.integrated_error_angle_list = []
                self.integrated_error_list = []
            else:
                self.log_stop_cond_bool = True

            #now publish the control output:
            control_msg.linear.x = float(control_input.item(0)) #extract these elements then cast them in float type
            control_msg.angular.z = float(control_input.item(1))
            # self.get_logger().info('/* log */ control_msg.linear.x: {0}'.format(control_msg.linear.x))
            # self.get_logger().info('/* log */ control_msg.angular.z: {0}'.format(control_msg.angular.z))
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
