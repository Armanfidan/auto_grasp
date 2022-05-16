#!/usr/bin/env python
import rospy
from vision_msgs.msg import Detection2DArray 

def callback(data):
    detection = data.detections[0]
    rospy.loginfo("Object: " + str(detection.results[0].id) + \
     " (score: " + str(detection.results[0].score) + ")\nBounding Box:\n" + \
     str(detection.bbox))
    
def listener():

    rospy.init_node('create_cuboid')
    rospy.Subscriber("/detectnet/detections", Detection2DArray, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()
