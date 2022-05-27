#! /usr/bin/env python

from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy
import roslib
import sys


def main(args):
    rospy.init_node('click')
    pub = rospy.Publisher('click', String, queue_size=1)

    try:
        if input("Press any key to activate grasp..."):
            pub.publish(String("clicked"))
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == '__main__':
    main(sys.argv)