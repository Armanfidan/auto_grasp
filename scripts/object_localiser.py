#! /usr/bin/env python3

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from tf2_msgs.msg import TFMessage
import rospy
import tf
import sys
import math
import numpy as np
from bosdyn.client.math_helpers import Quat, SE3Pose
from cv_bridge import CvBridge

sources = ['frontleft', 'frontright']

source = sources[1]

depth_image_topic = '/depth/' + source + '/image'
detection_topic = '/detectnet/detections'
camera_info_topic = '/depth/' + source + '/camera_info'

final_frame_id = "/vision_odometry_frame"


def get_distance_to_bbox_centre(raw_depth_image, bbox_centre, roi_size, depth_scale):
    
    """Create a region of interest (ROI) and get the average distance of the ROI.

    Args:
        raw_depth_image (np.array): Matrix of depth pixels
        bbox_centre (tuple): Centre point of the object bounding box
        roi_size (int): How many pixels to explore in each direction to find a valid depth value
        depth_scale (float): Depth scale of the image to convert from sensor value to meters
    """
    x, y = bbox_centre
    roi = []
    depths = []
    for row in range(y - roi_size, y + roi_size + 1):
        for col in range(x - roi_size, x + roi_size + 1):
            raw_depth = raw_depth_image[row][col]
            if raw_depth != 0 and raw_depth is not None:
                depth = raw_depth / depth_scale
                depths.append(depth)

    return np.mean(depths)


class ObjectLocaliser:
    def __init__(self, roi_size=1):
        self.rotated_bbox_centre = None
        self.detection = None
        self.object_region = []

        self.roi_size = roi_size

        self.intrinsics_exist = False
        self.camera_intrinsics = {}

        rospy.init_node('object_localiser', anonymous=True)

        self.depth_sub = rospy.Subscriber(depth_image_topic, Image, self.depth_image_callback, queue_size=1)
        self.detectnet_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.detection_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        # rospy.loginfo("Publishing object positions on /object_point")
        self.object_point_camera_frame_pub = rospy.Publisher("/object_point/camera_frame", PointStamped, queue_size=1)
        self.object_point_odometry_frame_pub = rospy.Publisher("/object_point/odometry_frame", PointStamped, queue_size=1)
        self.listener = tf.TransformListener()

    def get_object_position(self, raw_depth_image):
        """
        Given a bounding box extracted from a 90-degree clockwise rotated image and camera intrinsics,
        return the real-world object position with respect to the camera frame.

        Args:
            depth_image_pixels (np.array): The raw depth image in a NumPy array
        """
        # The image used for detection is rotated 90 degrees clockwise. We correct the rotation by rotating
        # the bounding box points in the other direction.
        x, y = self.rotated_bbox_centre
        bbox_centre = y, self.height - x

        # print("Rotated bounding box centre:", self.rotated_bbox_centre)
        # print("Corrected bounding box centre:", bbox_centre)
        # unpack the images.
        try:
            bridge = CvBridge()
            # Get the depth data from the region in the bounding box. ROI is 1, depth scale is 999 (From image info)
            depth = get_distance_to_bbox_centre(raw_depth_image, bbox_centre, self.roi_size, 999)

            if depth >= 4.0:
                # Not enough depth data.
                print('Not enough depth data.')
                return False

            tform_x = \
                depth * (bbox_centre[0] - self.camera_intrinsics['principal_x']) / self.camera_intrinsics['focal_x']
            tform_y = \
                depth * (bbox_centre[1] - self.camera_intrinsics['principal_y']) / self.camera_intrinsics['focal_y']
            tform_z = depth
            obj_tform_camera = SE3Pose(tform_x, tform_y, tform_z, Quat()).position

            return obj_tform_camera
        except Exception as exc:  # pylint: disable=broad-except
            print(f'Error getting object position: {exc}')
            return

    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp

    def camera_info_callback(self, camera_info_msg):
        if self.intrinsics_exist:
            return
        self.frame_id = camera_info_msg.header.frame_id
        self.width, self.height = camera_info_msg.width, camera_info_msg.height
        self.camera_intrinsics = {
            'focal_x': camera_info_msg.K[0],
            'focal_y': camera_info_msg.K[4],
            'principal_x': camera_info_msg.K[2],
            'principal_y': camera_info_msg.K[5]
        }
        self.intrinsics_exist = True

    def depth_image_callback(self, depth_image_msg):
        if not (self.rotated_bbox_centre and self.intrinsics_exist):
            return

        bridge = CvBridge()
        raw_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        
        object_in_camera_frame = self.get_object_position(raw_depth_image)

        object_point_camera_frame = PointStamped()
        object_point_camera_frame.header.stamp = self.tf_timestamp
        object_point_camera_frame.header.frame_id = self.frame_id
        object_point_camera_frame.point = Point(
            x=object_in_camera_frame.x,
            y=object_in_camera_frame.y,
            z=object_in_camera_frame.z
        )
        self.object_point_camera_frame_pub.publish(object_point_camera_frame)
        try:
            self.listener.waitForTransform(self.frame_id, final_frame_id, self.tf_timestamp, rospy.Duration(4.0))
            object_point_odometry_frame = self.listener.transformPoint(final_frame_id, object_point_camera_frame)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
        
        self.object_point_odometry_frame_pub.publish(object_point_odometry_frame)

        if not object_point_odometry_frame.point:
            print("The object is not in a valid position.")
            self.bbox = None
            return
        if math.isnan(object_point_odometry_frame.point.x) or \
                math.isnan(object_point_odometry_frame.point.y) or \
                math.isnan(object_point_odometry_frame.point.z):
            print("The point %s could not be transformed.", str(object_point_odometry_frame.point))
            self.bbox = None
            return
        
        self.bbox = None
  
    def detection_callback(self, detections_msg):
        detection = detections_msg.detections[0]
        self.rotated_bbox_centre = (
            int(detection.bbox.center.x),
            int(detection.bbox.center.y)
        )


def main(args):
    ObjectLocaliser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
