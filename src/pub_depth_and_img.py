from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox3D
from sensor_msgs.msg import CameraInfo, Image
import rospy
import sys
import numpy as np

def callback(depth_msg, args):
    offset = depth_msg.header.seq
    while n:
        offset += counter
        counter
        if called:
            counter = 0
    depth_msgs = args[0]
    depth_msgs[seq] = depth_msg



class ImageAccumulator:
    def __init__(self):
        self.depth_msgs = []
        self.offset = 0
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth)
        self.detectnet_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray, self.callback_detectnet)
        self.pub = rospy.Publisher("/auto_grasp/grasp_data", Detection2D)
        rospy.spin()

    def callback_depth(self, depth_msg):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if self.offset == 0:
            self.offset = depth_msg.header.seq
        self.depth_msgs.append(depth_msg)
        
    def callback_detectnet(self, detections):
        # We get a list of detections and take the first one, because we will only concentrate on one object
        # per image.
        detection = detections[0]
        # The sequence number is used to identify the source image. There are different headers: For the detections
        # array (which represents the sequence number of the image itself) and for each detection, which we don't care about.
        seq = detections.header.seq
        try:
            # We extract the correct depth message and reset the depth_msgs and offset.
            depth_msg = self.depth_msgs[seq - self.offset]
            self.depth_msgs = []
            self.offset = 0
        except IndexError:
            print("A corresponding depth image could not be found for the given detection.")
            sys.exit(1)

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
        self.pub.publish(grasp_msg)


def main(args):
    ImageAccumulator()
    rospy.init_node('detection_processor', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == '__main__':
    main(sys.argv)
