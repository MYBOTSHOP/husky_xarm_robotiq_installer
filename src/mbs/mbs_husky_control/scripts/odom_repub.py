#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def callback(data, pub):
    print(data)
    pub.publish(data)
    return

def listener():
    rospy.init_node('odom_republisher')
    pub = rospy.Publisher("odom", Odometry, queue_size=1)
    rospy.Subscriber("husky_velocity_controller/odom", Odometry, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()