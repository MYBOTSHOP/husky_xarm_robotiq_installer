#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nmea_msgs.msg import Sentence

def callback(data):
    print(data.sentence)
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/gps/nmea_sentence", Sentence, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()