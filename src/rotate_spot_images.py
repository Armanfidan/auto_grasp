#! /usr/bin/env python3

from sensor_msgs.msg import Image, CameraInfo
import rospy
import sys
import numpy as np


class ImageRotator:
    def __init__(self, camera):
        self.recevied_info = False
        
        self.sub_info = rospy.Subscriber("/camera/" + camera + "/camera_info", CameraInfo, self.info_callback, queue_size=1)
        self.sub_img = rospy.Subscriber("/camera/" + camera + "/image", Image, self.img_depth_callback, ["image"], queue_size=1)
        self.sub_depth = rospy.Subscriber("/depth/" + camera + "/image", Image, self.img_depth_callback, ["depth"], queue_size=1)

        self.pub_img = rospy.Publisher("/camera/" + camera + "/image_rotated", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("/depth/" + camera + "/image_rotated", Image, queue_size=1)

        print("Image rotator node initialised, waiting for messages...")
        print("Listening to:")
        print("     /camera/" + camera + "/camera_info")
        print("     /camera/" + camera + "/image")
        print("     /depth/" + camera + "/image")
        print("Published topics:")
        print("     /camera/" + camera + "/image_rotated")
        print("     /depth/" + camera + "/image_rotated")
    
    def info_callback(self, info):
        print("received info!")
        if self.recevied_info:
            return
        self.width = info.width
        self.height = info.height
        print(self.width)
        print(self.height)
        self.recevied_info = True

    def img_depth_callback(self, image, args):
        if not self.recevied_info:
            return
        print(args[0])
        print(len(image.data))
        print(image.step)
        stride = image.step // self.width
        print(stride)
        two_d = np.reshape(np.array(list(image.data)), (self.height, self.width * stride))
        two_d_rotated = np.rot90(two_d, 3)
        flattened_rotated = list(np.flatten(two_d_rotated))
        rotated_msg = Image()
        rotated_msg.header = image.header
        rotated_msg.width = self.height
        rotated_msg.height = self.width
        rotated_msg.encoding = image.encoding
        rotated_msg.is_bigendian = image.is_bigendian
        rotated_msg.step = image.step
        rotated_msg.data = one_d_rotated
        if args[0] == "image":
            self.pub_img.publish(rotated_msg)
        elif args[0] == "depth":
            self.pub_depth.publish(rotated_msg)
        else:
            print("Invalid publisher selected.")
            sys.exit(1)


def main(args):
    rospy.init_node("image_rotator")
    if len(args) < 2:
        print("Please specify camera (frontleft or frontright) as an argument.")
        sys.exit(1)
    camera = args[1]
    rotator = ImageRotator(camera)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    # This can be either "frontleft" or "frontright".
    main(sys.argv)