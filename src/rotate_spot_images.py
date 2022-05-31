#! /usr/bin/env python3.7

import argparse
import sys

from bosdyn.api import image_pb2
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
from bosdyn.api import image_pb2
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


class ImageRotator:
    def __init__(self, options):
        self.sdk = bosdyn.client.create_standard_sdk('image_capture')
        self.robot = sdk.create_robot(options.hostname)
        self.robot.authenticate(options.username, options.password)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync()

        self.image_client = robot.ensure_client(options.image_service)

        self.bridge = CvBridge()
        self.publishers = {}
        
        if options.image_sources:
            for source in options.image_sources:
                self.publishers[source] = rospy.Publisher("/camera/" + source + "/image_rotated", Image, queue_size=1)
    
    @property
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self.robot.time_sync.endpoint.clock_skew

    def robotToLocaTime(self, timestamp):
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
        if options.image_sources:
            # Capture and save images to disk
            image_responses = image_client.get_image_from_sources(options.image_sources)

            for image in image_responses:
                num_bytes = 1  # Assume a default of 1 byte encodings.
                if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                    dtype = np.uint16
                    extension = ".png"
                else:
                    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                        num_bytes = 3
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                        num_bytes = 4
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                        num_bytes = 1
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                        num_bytes = 2
                    dtype = np.uint8
                    extension = ".jpg"

                img = np.frombuffer(image.shot.image.data, dtype=dtype)
                if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    try:
                        # Attempt to reshape array into a RGB rows X cols shape.
                        img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
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
                    print(e)
                    sys.exit(1)
                
                local_time = self.robotToLocalTime(image.shot.acquisition_time)
                image_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
                image_msg.header.frame_id = data.source.name
                image_msg.height = data.shot.image.rows
                image_msg.width = data.shot.image.cols
                self.publishers[]


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
    options = parser.parse_args(argv)

    # Raise exception if no actionable argument provided
    if not options.list and not options.image_sources:
        parser.error('Must provide actionable argument (list or image-sources).')

    # Optionally list image sources on robot.
    if options.list:
        image_sources = image_client.list_image_sources()
        print("Image sources:")
        for source in image_sources:
            print("\t" + source.name)

    # Initialise the node
    rospy.init_node("image_rotator")
    rotator = ImageRotator(options)
    try:
        rotator.publish_rotated_images()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")

    return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)