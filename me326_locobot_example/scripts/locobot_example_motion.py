#!/usr/bin/env python3
'''
Written by: Monroe Kennedy, Date: 1/2/2023

This script starts at the bottom "if __name__ == '__main__':" which is a function that calls "main():" function.
The main function then instanties a class object (Locobot_example), which takes in a target pose, then listens to the topic 
of the robots odometry (pose), and then calculates the control action to take, then commands the velocity to the robot

This script shows how the robots can go between two points A and B, where A is the initial point (origin), and B is a specified point. 
The robot is first controlled to point B, then is instructed to return to point A.

read more about rospy publishers/subscribers here: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
'''
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


class LocobotExample(object):
    """Class for example operations for locobot control
    Point A: the origin (x,y,z) = (0,0,0)
    Point B: the point (x,y,z) = (1,0,0)

    This script demonstrates how to move from A to B using proportional Control
    The pose of the Locobot is defined by its (x,y) position 
    """
    def __init__(self):
        self.current_target = "B" #points A and B, if A its the origin, if B it is the second specified point. This sript
        self.target_pose_reached_bool = False
        self.target_pose = None

        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1) #this is the topic we will publish to in order to move the base
        self.point_P_control_point_visual = rospy.Publisher("/locobot/mobile_base/control_point_P",Marker,queue_size=1) #this can then be visualized in RVIZ (ros visualization)

        self.target_pose_visual = rospy.Publisher("/locobot/mobile_base/target_pose_visual",Marker, queue_size=1)

        self.L = 0.1 #this is the distance of the point P (x,y) that will be controlled for position. The locobot base_link frame points forward in the positive x direction, the point P will be on the positive x-axis in the body-fixed frame of the robot mobile base

 
        #set targets for when a goal is reached: 
        self.goal_reached_error = 0.005
 


    def pub_point_P_marker(self):
        #this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
        marker = Marker()
        marker.header.frame_id = "locobot/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        # Set the marker scale
        marker.scale.x = 0.1  # radius of the sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the marker pose
        marker.pose.position.x = self.L  # center of the sphere in base_link frame
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.point_P_control_point_visual.publish(marker)


    def pub_target_point_marker(self):
        #this is putting the marker in the world frame (http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29)
        marker = Marker()
        marker.header.frame_id = "locobot/odom" #this will be the world frame for the real robot
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.ARROW
        # Set the marker scale
        marker.scale.x = 0.3  # arrow length
        marker.scale.y = 0.1 #arrow width
        marker.scale.z = 0.1 #arrow height

        # Set the marker pose
        marker.pose.position.x = self.target_pose.position.x  # center of the sphere in base_link frame
        marker.pose.position.y = self.target_pose.position.y
        marker.pose.position.z = self.target_pose.position.z
        marker.pose.orientation.x = self.target_pose.orientation.x
        marker.pose.orientation.y = self.target_pose.orientation.y
        marker.pose.orientation.z = self.target_pose.orientation.z
        marker.pose.orientation.w = self.target_pose.orientation.w

        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0 #red
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        self.target_pose_visual.publish(marker)


    def mobile_base_callback(self, data):
        """
        This function is the callback for the message that is publishing the pose of mobile base.
        Note: the locobot bases are non-holonomic, meaning that like a wheelchair, they cannot 
        instantaneously move side-to-side, so to control the (x,y) position of the base, we must control a point (we will call P) 
        that is in front or behind the non-holonomic line (the line that connects the center of the wheels). It turns out 
        that it is possible to control this point in any direction we choose, but we cannot simultaneosuly control the position (x,y) and the planar angle (theta).
        So the strategy will be to move to the desired (x,y) location, then rotate to the desired angle sequentially (in this script, we will focus on just the xy motion, and leave as an exercise the rotation to the student)

        Note: this message topic /locobot/mobile_base/odom is published at about 30Hz, and we are using it to trigger our control script (so we control at the 
        frequency we obtain the state information)
        """


        # Step 1: Calculate the point P location (distance L on the x-axis), and publish the marker so it can be seen in Rviz
        #first determine the relative angle of the mobile base in the world xy-plane, this angle is needed to determine where to put the point P
        #the rotation will be about the world/body z-axis, so we will only need the qw, and qz quaternion components. We can then use knoweldge of the 
        #relationship between quaternions and rotation matricies to see how we must rotate the Lx vector into the world (odom) frame and add it to the base position
        #to obtain the point P (for more info on quaterions, see a primer at the bottom of this page: https://arm.stanford.edu/resources/armlab-references)
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz - 2*qw*qz
        R21 = 2*qx*qz + 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        point_P = Point()
        #NOTE: the following assumes that when at the origin, the baselink and odom/world frame are aligned, and the z-axis points up. If this is not true then there is not a simple rotation about the z-axis as shown below
        point_P.x = data.pose.pose.position.x + self.L*R11
        point_P.y = data.pose.pose.position.y + self.L*R21
        point_P.z = 0.1 #make it hover just above the ground (10cm)

        #publish the point P and target pose markers for visualization in Rviz:
        self.pub_point_P_marker()
        self.pub_target_point_marker()


        # Step 2: Calculate the error between the target pose for position control (this will relate to the proportoinal gain matrix, the P in PID control)
        err_x = self.target_pose.position.x - point_P.x
        err_y = self.target_pose.position.y - point_P.y
        error_vect = np.matrix([[err_x],[err_y]]) #this is a column vector (2x1); equivalently, we could use the transpose operator (.T): np.matrix([err_x ,err_y]).T  

        Kp_mat = 1*np.eye(2) #proportional gain matrix, diagonal with gain of 0.2 (for PID control)

        #We will deal with this later (once we reached the position (x,y) goal), but we can calculate the angular error now - again this assumes there is only planar rotation about the z-axis, and the odom/baselink frames when aligned have x,y in the plane and z pointing upwards
        Rotation_mat = np.matrix([[R11,R12],[R21,R22]])
        #We can use matrix logarithm (inverse or Rodrigues Formula and exponential map) to get an axis-angle representation:
        axis_angle_mat = sp.linalg.logm(Rotation_mat)
        
        # This is the angle error: how should frame Base move to go back to world frame?
        angle_error = axis_angle_mat[0,1] #access the first row, second column to get angular error (skew sym matrix of the rotation axis - here only z component, then magnitude is angle error between the current pose and the world/odom pose which we will return to both at points A and B) 
        
        # This Angle is selected because its the frame rotation angle, how does Base appear from world?
        current_angle = axis_angle_mat[1,0] #this is also the angle about the z-axis of the base
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
        point_p_error_signal = Kp_mat*error_vect #+ Ki_mat*pos_err_integrated
        #The following relates the desired motion of the point P and the commanded forward and angular velocity of the mobile base [v,w]
        non_holonomic_mat = np.matrix([[np.cos(current_angle), -self.L*np.sin(current_angle)],[np.sin(current_angle),self.L*np.cos(current_angle)]])
        #Now perform inversion to find the forward velocity and angular velcoity of the mobile base.
        control_input = np.linalg.inv(non_holonomic_mat)*point_p_error_signal #note: this matrix can always be inverted because the angle is L
        #now let's turn this into the message type and publish it to the robot:
        control_msg = Twist()
        control_msg.linear.x = float(control_input.item(0)) #extract these elements then cast them in float type
        control_msg.angular.z = float(control_input.item(1))
        #now publish the control output:
        self.mobile_base_vel_publisher.publish(control_msg)

        #find the magnitude of the positional error to determine if its time to focus on orientation or switch targets
        err_magnitude = np.linalg.norm(error_vect)
   
        #Step 4: Finally, once point B has been reached, then return back to point A and vice versa      
        if err_magnitude < self.goal_reached_error:
            # switch targets so the locobot goes back and forth between points A and B
            # print("reached TARGET! A current target:",self.current_target == 'A')
            if self.current_target == 'A':
                self.current_target = 'B'
            else:
                self.current_target = 'A'

        if self.current_target == 'A':
            #if current target is A, then set it as the goal pose
            self.target_pose.position.x = 0.0
            self.target_pose.position.y = 0.0
        if self.current_target == 'B':
            self.target_pose.position.x = 1.0
            self.target_pose.position.y = 0.0

        

    def go_to_pose(self,target_pose=None):
        """
        Input: Target_pose - ROS geometry_msgs.msg.Pose type
        To see what it looks like, put the following in the terminal: $ rosmsg show geometry_msgs/Pose
        """
        
        #Step 1: Obtain or specify the goal pose (in sim, its between locobot/odom (world) and locobot/base_link (mobile base))
        if type(target_pose) == type(None):
            target_pose = Pose()
            target_pose.position.x = 1.0
            target_pose.position.y = 0.0
            #specify the desired pose to be the same orientation as the origin
            target_pose.orientation.x = 0
            target_pose.orientation.y = 0
            target_pose.orientation.z = 0
            target_pose.orientation.w = 1 # cos(theta/2)
            self.target_pose = target_pose
        elif type(target_pose) != type(Pose()):
            rospy.logerr("Incorrect type for target pose, expects geometry_msgs Pose type") #send error msg if wrong type is send to go_to_pose
        else:
            self.target_pose = target_pose

        #Step 2: Subscribe to pose of the base, and use 'Point P control' (point right in front of the non-holonomic base's line) to move to the position, then orient to the target orinetation once the position is reached

        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.mobile_base_callback) #this says: listen to the odom message, of type odometry, and send that to the callback function specified
        rospy.spin() #This is ros python's way of 'always listening' for the subscriber messages, and when it 



def main():
    rospy.init_node('locobot_back_and_forth')
    cls_obj = LocobotExample() #instantiate object of the class (to call and use the functions)
    cls_obj.go_to_pose()



if __name__ == '__main__':
    #this is where the script begins, calls the main function
    main()

