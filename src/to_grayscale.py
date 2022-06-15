#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
Node to transform an input Image topic into
an output grayscale Image topic.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GrayScaler(object):
    def __init__(self, name_topic_in, name_topic_out):
        self.cv_bridge = CvBridge()
        rospy.loginfo("Converting Images from topic " + name_topic_in +
                      " to grayscale, output topic: " + name_topic_out)
        self.pub = rospy.Publisher(name_topic_out, Image, queue_size=5)
        self.sub = rospy.Subscriber(
            name_topic_in, Image, self.image_cb, queue_size=5)

    def image_cb(self, img_msg):
        # Transform to cv2/numpy image
        img_in_cv2 = self.cv_bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')
        # Transform to grayscale,
        # available encodings: http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        if "rgb" in img_msg.encoding:
            gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_RGB2GRAY)
        elif "bgr" in img_msg.encoding:
            gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_BGR2GRAY)
        # Transform back to Image message
        gray_img_msg = self.cv_bridge.cv2_to_imgmsg(
            gray_img, encoding="mono8")
        self.pub.publish(gray_img_msg)


if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    if len(argv) != 3:
        print("Usage:")
        print(argv[0] + " name_topic_in name_topic_out")
        print("Converts a RGB/BGR name_topic_in Image topic into a Grayscale name_topic_out Image topic")
        exit(0)
    rospy.init_node("image_to_grayscale", anonymous=True)

    gs = GrayScaler(argv[1], argv[2])
    rospy.spin()
