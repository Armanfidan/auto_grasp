#! /usr/bin/env python

from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import Image
import rospy
import roslib
import sys


class ImageAccumulator:
    def __init__(self):
        self.depth_msgs = []
        self.offset = 0
        
        rospy.init_node('image_accumulator', anonymous=True)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth, queue_size=1)
        self.detectnet_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray, self.callback_detectnet, queue_size=1)
        self.pub = rospy.Publisher("/auto_grasp/grasp_data", Detection2D, queue_size=1)

        print("Image accumulator node initialised, waiting for messages...")
        rospy.spin()

    def callback_depth(self, depth_msg):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if self.offset == 0:
            self.offset = depth_msg.header.seq
        self.depth_msgs.append(depth_msg)
        print("Depth message " + str(depth_msg.header.seq) + " received and logged")
        
    def callback_detectnet(self, detections_msg):
        # We get a list of detections and take the first one, because we will only concentrate on one object
        # per image.
        detection = detections_msg.detections[0]
        # The sequence number is used to identify the source image. There are different headers: For the detections
        # array (which represents the sequence number of the image itself) and for each detection, which we don't care about.
        seq = detections_msg.header.seq
        print("Received detection sequence number is " + str(seq))

        # We extract the correct depth message and reset the depth_msgs and offset.
        index = seq - self.offset
        if index < 0 or index >= len(self.depth_msgs):
            print("Corresponding index would be " + str(index) + ". A corresponding depth image could not be found for the given detection.")
            sys.exit(1)
        depth_msg = self.depth_msgs[index]
        self.depth_msgs = []
        self.offset = 0

        # We extract the required information from the depth message.
        depth_data, height, width = depth_msg.data, depth_msg.height, depth_msg.width
        bbox = detection.results[0].bbox
        if bbox.center.x <= width and bbox.center.y <= height:
            try:
                # The depth data is an array, so we have to get the correct number using the height and width.
                object_surface_depth = depth_data[bbox.center.y * width + bbox.center.x]
            except IndexError:
                print("The detected object was out of image bounds.")
        # When calculating the object depth, we will not use the surface depth. Instead, we will calculate the depth
        # of the object centre. I will assume that the object depth is the maximum of its height and width.
        z = object_surface_depth + max(bbox.size_x, bbox.size_y) // 2

        # Creating the grasp message.
        grasp_msg = Detection2D()
        grasp_msg.header = detection.header
        grasp_msg.results = detection.results
        # I will use grasp_msg.pose.pose.position.z to carry the depth information to avoid switching from BoundingBox2D
        # to BoundingBox3D and adding complexity.
        grasp_msg.results[0].pose.pose.position.z = z
        print("Generated sphere: (" + str(bbox.center.x) + "," + str(bbox.center.y) + "," + str(z) + "), w=" + str(bbox.size_x) + ", h=" + str(bbox.size_y))
        self.pub.publish(grasp_msg)


def main(args):
    ImageAccumulator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == '__main__':
    main(sys.argv)
