#!/usr/bin/env python
import rospy
from vision_msgs.msg import Detection2DArray 

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    rospy.init_node('create_cuboid')
    rospy.Subscriber("/detectnet/detections", Detection2DArray, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()
