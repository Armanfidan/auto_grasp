#! /usr/bin/env python3

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
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

detection_topic = '/detectnet/detections'

spot_depth_image_topic = '/depth/' + source + '/image'
spot_camera_info_topic = '/depth/' + source + '/camera_info'

# Very important - use non-rectified images for both detection and raycasting, otherwise transforms won't work.
kinova_depth_image_topic = '/camera/depth/image_raw'
kinova_camera_info_topic = '/camera/depth/camera_info'

switch_detectnet_topic = '/switch_detectnet'

initial_detectnet_source = 'kinova'

final_frame_id = "/vision_odometry_frame"



def get_distance_to_bbox_centre(raw_depth_image, bbox_centre, roi_size, depth_scale):
    
    """Create a region of interest (ROI) and get the average distance of the ROI.

    Args:
        raw_depth_image (np.array): Matrix of depth pixels
        bbox_centre (tuple): Centre point of the object bounding box
        roi_size (int): How many pixels to explore in each direction to find a valid depth value
        depth_scale (float): Depth scale of the image to convert from sensor value to meters
    """
    # print("BBox centre:", bbox_centre)
    # print("Depth image shape:", raw_depth_image.shape)
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
    def __init__(self, current_detectnet_source, depth_frame_id=None, roi_size=1):
        self.raw_bbox_centre = None
        self.detection = None
        self.object_region = []
        self.current_detectnet_source = current_detectnet_source
        self.tf_timestamp = None

        self.odom_points = []
        self.camera_points = []

        self.depth_frame_id = depth_frame_id

        self.depth_image_topic = kinova_depth_image_topic if current_detectnet_source == 'kinova' else spot_depth_image_topic
        self.camera_info_topic = kinova_camera_info_topic if current_detectnet_source == 'kinova' else spot_camera_info_topic

        self.roi_size = roi_size
        self.intrinsics_exist = False
        self.camera_intrinsics = {}

        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback, queue_size=1)
        self.detectnet_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.detection_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        self.switch_detectnet_sub = rospy.Subscriber(switch_detectnet_topic, String, self.switch_detectnet_callback, queue_size=1)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        
        self.object_point_camera_frame_pub = rospy.Publisher("/object_point/camera_frame", PointStamped, queue_size=1)
        self.object_point_base_link_pub = rospy.Publisher("/object_point/base_link", PointStamped, queue_size=1)
        self.object_point_odometry_frame_pub = rospy.Publisher("/object_point/odometry_frame", PointStamped, queue_size=1)
        self.listener = tf.TransformListener()

    def switch_detectnet_callback(self, switch):
        new_detectnet_source = switch.data
        assert new_detectnet_source == 'kinova' or new_detectnet_source == 'spot'
        if new_detectnet_source != self.current_detectnet_source:
            self.__init__(new_detectnet_source, self.depth_frame_id)

    def camera_info_callback(self, camera_info_msg):
        if self.intrinsics_exist or self.depth_frame_id == camera_info_msg.header.frame_id:
            return
        print("Camera intrinsics belong to", self.current_detectnet_source)
        self.depth_frame_id = camera_info_msg.header.frame_id
        print("Frame:", self.depth_frame_id)
        self.width, self.height = camera_info_msg.width, camera_info_msg.height
        self.camera_intrinsics = {
            'focal_x': camera_info_msg.K[0],
            'focal_y': camera_info_msg.K[4],
            'principal_x': camera_info_msg.K[2],
            'principal_y': camera_info_msg.K[5]
        }
        self.intrinsics_exist = True

    def get_object_position_spot(self, raw_depth_image):
        """
        Given a bounding box extracted from a 90-degree clockwise rotated image and camera intrinsics,
        return the real-world object position with respect to the camera frame.

        Args:
            depth_image_pixels (np.array): The raw depth image in a NumPy array
        """
        # print("SPOTSPOT")
        # The image used for detection is rotated 90 degrees clockwise. We correct the rotation by rotating
        # the bounding box points in the other direction.
        x, y = self.raw_bbox_centre
        bbox_centre = y, self.height - x

        try:
            bridge = CvBridge()
            # Get the depth data from the region in the bounding box. ROI is 1, depth scale is 999 (From image info)
            depth = get_distance_to_bbox_centre(raw_depth_image, bbox_centre, self.roi_size, 999)
            # print(bbox_centre)
            # print(depth)

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
        # print("KINOVAKINOVA")
        if not self.tf_timestamp:
            return
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

    def depth_image_callback(self, depth_image_msg):
        if not (self.raw_bbox_centre and self.intrinsics_exist):
            return

        bridge = CvBridge()
        raw_depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        object_in_camera_frame = self.get_object_position_kinova(raw_depth_image) \
            if self.current_detectnet_source == 'kinova' \
            else self.get_object_position_spot(raw_depth_image)
        # print(object_in_camera_frame)
        # print("Got object position from", self.current_detectnet_source)

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
            self.listener.waitForTransform(self.depth_frame_id, '/base_link', self.tf_timestamp, rospy.Duration(4.0))
            object_point_base_link = self.listener.transformPoint('/base_link', object_point_camera_frame)
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
        
        self.object_point_odometry_frame_pub.publish(object_point_odometry_frame)
        self.object_point_base_link_pub.publish(object_point_base_link)

        # # Let's make it accumulate the positions over 1000 tries and find the error!
        # if len(self.odom_points) < 1000:
        #     print(len(self.odom_points))
        #     self.odom_points.append(object_point_odometry_frame)
        #     self.camera_points.append(object_point_camera_frame)
        # else:
        #     odom_x = [point.point.x for point in self.odom_points]
        #     odom_y = [point.point.y for point in self.odom_points]
        #     odom_z = [point.point.z for point in self.odom_points]
        #     camera_x = [point.point.x for point in self.camera_points]
        #     camera_y = [point.point.y for point in self.camera_points]
        #     camera_z = [point.point.z for point in self.camera_points]
        #     odom_max_difference_x = max(odom_x) - min(odom_x)
        #     odom_max_difference_y = max(odom_y) - min(odom_y)
        #     odom_max_difference_z = max(odom_z) - min(odom_z)
        #     camera_max_difference_x = max(camera_x) - min(camera_x)
        #     camera_max_difference_y = max(camera_y) - min(camera_y)
        #     camera_max_difference_z = max(camera_z) - min(camera_z)
        #     odom_max_difference_x_p = odom_max_difference_x / np.mean(odom_x)
        #     odom_max_difference_y_p = odom_max_difference_y / np.mean(odom_y)
        #     odom_max_difference_z_p = odom_max_difference_z / np.mean(odom_z)
        #     camera_max_difference_x_p = camera_max_difference_x / np.mean(camera_x)
        #     camera_max_difference_y_p = camera_max_difference_x / np.mean(camera_x)
        #     camera_max_difference_z_p = camera_max_difference_x / np.mean(camera_x)
        #     print("Average distance over 1000 trials (m):")
        #     print("Odometry frame (x, y, z, norm):", np.mean(odom_x), np.mean(odom_y), np.mean(odom_z), np.sqrt(np.mean(odom_x) ** 2 + np.mean(odom_y) ** 2 + np.mean(odom_z) ** 2), sep="\n")
        #     print("Camera frame (x, y, z, norm):", np.mean(camera_x), np.mean(camera_y), np.mean(camera_z), np.sqrt(np.mean(camera_x) ** 2 + np.mean(camera_y) ** 2 + np.mean(camera_z) ** 2), sep="\n")
        #     print("Odometry frame absolute uncertainties over 1000 trials (m):")
        #     print(odom_max_difference_x)
        #     print(odom_max_difference_y)
        #     print(odom_max_difference_z)
        #     print("Camera frame absolute uncertainties over 1000 trials (m):")
        #     print(camera_max_difference_x)
        #     print(camera_max_difference_y)
        #     print(camera_max_difference_z)
        #     print("Odometry frame relative uncertainties over 1000 trials (%):")
        #     print(odom_max_difference_x_p)
        #     print(odom_max_difference_y_p)
        #     print(odom_max_difference_z_p)
        #     print("Camera frame relative uncertainties over 1000 trials (%):")
        #     print(camera_max_difference_x_p)
        #     print(camera_max_difference_y_p)
        #     print(camera_max_difference_z_p)
        #     self.odom_points = []
        #     self.camera_points = []


        self.bbox = None
  
    def detection_callback(self, detections_msg):
        for detection in detections_msg.detections:
            if (self.current_detectnet_source == 'kinova' and (detection.results[0].id == 88 or detection.results[0].id == 11)) or \
                (self.current_detectnet_source == 'spot'):
                self.raw_bbox_centre = (
                    int(detection.bbox.center.x),
                    int(detection.bbox.center.y)
                )
                break


def main(args):
    rospy.init_node('object_localiser', anonymous=True)
    localiser = ObjectLocaliser(current_detectnet_source=initial_detectnet_source)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down object localiser module...")


if __name__ == "__main__":
    main(sys.argv)