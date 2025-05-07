#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odom_callback(odom_msg:Odometry):
    pose_msg = PoseStamped()
    pose_msg.header = odom_msg.header
    pose_msg.pose.position = odom_msg.pose.pose.position
    pose_msg.pose.orientation = odom_msg.pose.pose.orientation
    
    pose_publisher.publish(pose_msg)

def odom_to_pose():
    rospy.init_node('odom_to_pose', anonymous=True)
    
    rospy.Subscriber('odom', Odometry, odom_callback)
    
    global pose_publisher
    pose_publisher = rospy.Publisher('pose', PoseStamped, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    odom_to_pose()