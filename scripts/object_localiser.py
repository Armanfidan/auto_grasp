#! /usr/bin/env python3

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from tf2_msgs.msg import TFMessage
import rospy
import tf
import tf2_ros
import sys
import math
import numpy as np
from bosdyn.client.math_helpers import Quat, SE3Pose
from cv_bridge import CvBridge

sources = ['frontleft', 'frontright']
source = sources[1]

# CHANGE TO FALSE TO SWITCH TO SPOT'S CAMERAS
use_arm_camera = False

detection_topic = '/detectnet/detections'

# Very important - use non-rectified images for both detection and raycasting, otherwise transforms won't work.
kinova_depth_image_topic = '/camera/depth/image_raw'
kinova_camera_info_topic = '/camera/depth/camera_info'
spot_depth_image_topic = '/depth/' + source + '/image'
spot_camera_info_topic = '/depth/' + source + '/camera_info'

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
        self.raw_bbox_centre = None
        self.detection = None
        self.object_region = []

        self.roi_size = roi_size

        self.intrinsics_exist = False
        self.camera_intrinsics = {}

        rospy.init_node('object_localiser', anonymous=True)

        self.kinova_depth_sub = rospy.Subscriber(kinova_depth_image_topic, Image, self.kinova_depth_image_callback, queue_size=1)
        self.spot_depth_sub = rospy.Subscriber(spot_depth_image_topic, Image, self.spot_depth_image_callback, queue_size=1)

        self.kinova_camera_info_sub = rospy.Subscriber(kinova_camera_info_topic, CameraInfo, self.kinova_camera_info_callback, queue_size=1)
        self.spot_camera_info_sub = rospy.Subscriber(spot_camera_info_topic, CameraInfo, self.spot_camera_info_callback, queue_size=1)

        self.detectnet_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.detection_callback, queue_size=1)
        
        self.kinova_object_point_camera_frame_pub = rospy.Publisher("/object_point/kinova/camera_frame", PointStamped, queue_size=1)
        self.kinova_object_point_odometry_frame_pub = rospy.Publisher("/object_point/kinova/odometry_frame", PointStamped, queue_size=1)
        self.spot_object_point_camera_frame_pub = rospy.Publisher("/object_point/spot/camera_frame", PointStamped, queue_size=1)
        self.spot_object_point_odometry_frame_pub = rospy.Publisher("/object_point/spot/odometry_frame", PointStamped, queue_size=1)

        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.listener = tf.TransformListener()

    def get_object_position_spot(self, raw_depth_image):
        """
        Given a bounding box extracted from a 90-degree clockwise rotated image and camera intrinsics,
        return the real-world object position with respect to the camera frame.

        Args:
            depth_image_pixels (np.array): The raw depth image in a NumPy array
        """
        # The image used for detection is rotated 90 degrees clockwise. We correct the rotation by rotating
        # the bounding box points in the other direction.
        x, y = self.raw_bbox_centre
        bbox_centre = y, self.height - x

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

    def get_object_position_kinova(self, raw_depth_image):
        """
        Given a bounding box extracted from the Kinova arm and camera intrinsics,
        return the real-world object position with respect to the camera frame.

        Args:
            depth_image_pixels (np.array): The raw depth image in a NumPy array
        """

        # How much to shrink bbox coordinates
        colour_to_depth_size_ratio = self.width / 1280

        bbox_camera_frame = PointStamped()
        bbox_camera_frame.header.stamp = self.tf_timestamp
        bbox_camera_frame.header.frame_id = 'kinova_camera_color_frame'
        bbox_camera_frame.point = Point(
            x=self.raw_bbox_centre[0] * colour_to_depth_size_ratio,
            y=self.raw_bbox_centre[1] * colour_to_depth_size_ratio
        )
        try:
            self.listener.waitForTransform('kinova_camera_color_frame', self.depth_frame_id, self.tf_timestamp, rospy.Duration(4.0))
            bbox_depth_frame = self.listener.transformPoint(self.depth_frame_id, bbox_camera_frame)
            # print(bbox_depth_frame)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)

        x = bbox_depth_frame.point.x
        y = bbox_depth_frame.point.y

        try:
            bridge = CvBridge()
            # Get the depth data from the region in the bounding box. ROI is 1, depth scale is 999 (From image info)
            depth = get_distance_to_bbox_centre(raw_depth_image, (int(x), int(y)), self.roi_size, 999)

            if depth >= 4.0:
                # Not enough depth data.
                print('Not enough depth data.')
                return False

            tform_x = \
                depth * (x - self.camera_intrinsics['principal_x']) / self.camera_intrinsics['focal_x']
            tform_y = \
                depth * (y - self.camera_intrinsics['principal_y']) / self.camera_intrinsics['focal_y']
            tform_z = depth
            obj_tform_camera = SE3Pose(tform_x, tform_y, tform_z, Quat()).position

            return obj_tform_camera
        except Exception as exc:  # pylint: disable=broad-except
            print(f'Error getting object position: {exc}')
            return

    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp

    def kinova_camera_info_callback(self, camera_info_msg):
        if self.intrinsics_exist:
            return
        self.depth_frame_id = camera_info_msg.header.frame_id
        self.width, self.height = camera_info_msg.width, camera_info_msg.height
        self.camera_intrinsics = {
            'focal_x': camera_info_msg.K[0],
            'focal_y': camera_info_msg.K[4],
            'principal_x': camera_info_msg.K[2],
            'principal_y': camera_info_msg.K[5]
        }
        self.intrinsics_exist = True
    
    def spot_camera_info_callback(self, camera_info_msg):
        if self.intrinsics_exist:
            return
        self.depth_frame_id = camera_info_msg.header.frame_id
        self.width, self.height = camera_info_msg.width, camera_info_msg.height
        self.camera_intrinsics = {
            'focal_x': camera_info_msg.K[0],
            'focal_y': camera_info_msg.K[4],
            'principal_x': camera_info_msg.K[2],
            'principal_y': camera_info_msg.K[5]
        }
        self.intrinsics_exist = True

    def kinova_depth_image_callback(self, depth_image_msg):
        if not (self.raw_bbox_centre and self.intrinsics_exist):
            return

        bridge = CvBridge()
        raw_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        
        object_in_camera_frame = self.get_object_position_kinova(raw_depth_image)
        self.localise_object(object_in_camera_frame, 'kinova')

    def spot_depth_image_callback(self, depth_image_msg):
        if not (self.raw_bbox_centre and self.intrinsics_exist):
            return

        bridge = CvBridge()
        raw_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        
        object_in_camera_frame = self.get_object_position_spot(raw_depth_image)
        self.localise_object(object_in_camera_frame, 'spot')

    def localise_object(object_in_camera_frame, source_robot)
        object_point_camera_frame = PointStamped()
        object_point_camera_frame.header.stamp = self.tf_timestamp
        object_point_camera_frame.header.frame_id = self.depth_frame_id
        object_point_camera_frame.point = Point(
            x=object_in_camera_frame.x,
            y=object_in_camera_frame.y,
            z=object_in_camera_frame.z
        )
        self.object_point_camera_frame_pub.publish(object_point_camera_frame)
        try:
            self.listener.waitForTransform(self.depth_frame_id, final_frame_id, self.tf_timestamp, rospy.Duration(4.0))
            object_point_odometry_frame = self.listener.transformPoint(final_frame_id, object_point_camera_frame)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            print("The point could not be transformed:", e)
            return
        
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
        
        if source_robot == 'kinova':
            self.kinova_object_point_odometry_frame_pub.publish(object_point_odometry_frame)
        elif source_robot == 'spot':
            self.spot_object_point_odometry_frame_pub.publish(object_point_odometry_frame)
        self.bbox = None
  
    def detection_callback(self, detections_msg):
        for detection in detections_msg.detections:
            if not use_arm_camera or detection.results[0].id == 88 or detection.results[0].id == 11:
                self.raw_bbox_centre = (
                    int(detection.bbox.center.x),
                    int(detection.bbox.center.y)
                )
                break


def main(args):
    ObjectLocaliser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
