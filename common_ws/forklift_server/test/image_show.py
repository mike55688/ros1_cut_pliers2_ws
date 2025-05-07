#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image

def image_callback(msg):
    rospy.loginfo("Received image with header: %s", msg.header)
    rospy.loginfo("Image size: %d x %d", msg.height, msg.width)

if __name__ == '__main__':
    rospy.init_node('image_subscriber')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()