#!/usr/bin/env python3
# Written by: Monroe Kennedy, Date: 1/2/2023
import rospy

# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()





def main():
    rospy.init_node('locobot_example')
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        print("node running")
        rate.sleep()
    pass



if __name__ == '__main__':
    #this is where the script begins, calls the main function
    main()

