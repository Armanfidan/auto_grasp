#!/usr/bin/env python3

import argparse
import sys

from bosdyn.api import image_pb2
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
from bosdyn.api import image_pb2
import cv2
import numpy as np

from google.protobuf.timestamp_pb2 import Timestamp

from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


class ImageRotator:
    def __init__(self, options):
        self.seq = 1

        self.sdk = bosdyn.client.create_standard_sdk('image_capture')
        self.robot = self.sdk.create_robot(options.hostname)
        self.robot.authenticate(options.username, options.password)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync()

        self.image_client = self.robot.ensure_client(options.image_service)

        self.bridge = CvBridge()
        self.publishers = {}
        
        if options.image_sources:
            self.image_sources = options.image_sources
            for source in options.image_sources:
                pub_name = "/camera/" + source + "_rotated"
                self.publishers[source] = rospy.Publisher(pub_name, Image, queue_size=1)
                rospy.loginfo("Publishing images in " + pub_name)

        # Optionally list image sources on robot.
        if options.list:
            image_sources = self.image_client.list_image_sources()
            rospy.loginfo("Image sources:")
            for source in image_sources:
                rospy.loginfo("\t" + source.name)
    
    @property
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self.robot.time_sync.endpoint.clock_skew

    def robotToLocalTime(self, timestamp):
            """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time
            Args:
                timestamp: google.protobuf.Timestamp
            Returns:
                google.protobuf.Timestamp
            """

            rtime = Timestamp()

            rtime.seconds = timestamp.seconds - self.time_skew.seconds
            rtime.nanos = timestamp.nanos - self.time_skew.nanos
            if rtime.nanos < 0:
                rtime.nanos = rtime.nanos + 1000000000
                rtime.seconds = rtime.seconds - 1

            # Workaround for timestamps being incomplete
            if rtime.seconds < 0:
                rtime.seconds = 0
                rtime.nanos = 0

            return rtime
    
    def publish_rotated_images(self):
        # Optionally capture one or more images.
        if self.image_sources:
            # Capture and save images to disk
            image_responses = self.image_client.get_image_from_sources(self.image_sources)

            for image in image_responses:
                dtype = np.uint8
                if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                    num_bytes = 1
                    dtype = np.uint16
                    encoding = "16UC1"
                    is_bigendian = False
                elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                    num_bytes = 3
                    encoding = "rgb8"
                    is_bigendian = True
                elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                    num_bytes = 4
                    encoding = "rgba8"
                    is_bigendian = True
                elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                    num_bytes = 1
                    encoding = "mono8"
                    is_bigendian = True
                elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                    num_bytes = 2
                    encoding = "16UC1"
                    is_bigendian = True

                img = np.frombuffer(image.shot.image.data, dtype=dtype)
                if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    try:
                        # Attempt to reshape array into a RGB rows X cols shape.
                        img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
                        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                            num_bytes = 2

                    except ValueError:
                        # Unable to reshape the image data, trying a regular decode.
                        img = cv2.imdecode(img, -1)
                else:
                    img = cv2.imdecode(img, -1)

                if image.source.name[0:5] == "front":
                    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                elif image.source.name[0:5] == "right":
                    img = cv2.rotate(img, cv2.ROTATE_180)

                try:
                    image_msg = self.bridge.cv2_to_imgmsg(img, "passthrough")
                except CvBridgeError as e:
                    rospy.logerr(e)
                    sys.exit(1)
                
                # Build ROS message
                local_time = self.robotToLocalTime(image.shot.acquisition_time)
                # image_msg.header.seq =   # Just a way to store data
                self.seq += 1
                image_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
                image_msg.header.frame_id = image.source.name
                image_msg.encoding = encoding
                image_msg.height = image.shot.image.cols
                # print("Depth scale:", image.source.depth_scale)
                # image_msg.height = image.source.depth_scale
                image_msg.width = image.shot.image.rows
                image_msg.is_bigendian = is_bigendian
                image_msg.step = num_bytes * image.shot.image.rows
                
                self.publishers[image.source.name].publish(image_msg)


def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    # This adds the --username and --password arguments. These will be placed in the launch file.
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('--list', help='list image sources', action='store_true')
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright', action='store_true')
    parser.add_argument('--image-sources', help='Get image from source(s)', action='append')
    parser.add_argument('--image-service', help='Name of the image service to query.',
                        default=ImageClient.default_service_name)
    options, unknown = parser.parse_known_args(argv)

    # Raise exception if no actionable argument provided
    if not options.list and not options.image_sources:
        parser.error('Must provide actionable argument (list or image-sources).')

    # Initialise the node
    rospy.init_node("expose_rotated_images")
    rotator = ImageRotator(options)
    while not rospy.is_shutdown():
        try:
            rotator.publish_rotated_images()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS Image detection processor module")
            return False


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)