#! /usr/bin/env python

from pickle import NONE
from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from tf2_msgs.msg import TFMessage
import sensor_msgs.point_cloud2 as pc2
import rospy
import tf
import sys
import math

sources = ['frontleft', 'frontright']

source = sources[1]

depth_image_topic = '/depth/' + source + '/image'
detection_topic = '/detectnet/detections'
camera_info_topic = '/depth/' + source + '/camera_info'


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


def rotate_about_origin(point, angle, origin=(0, 0)):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    Args:
        origin (tuple): Origin to rotate the point around
        point (tuple): Point to rotate
        angle (float): Angle in radians
    """
    orig_x, orig_y = origin
    pnt_x, pnt_y = point

    ret_x = orig_x + math.cos(angle) * (pnt_x - orig_x) - math.sin(angle) * (pnt_y - orig_y)
    ret_y = orig_y + math.sin(angle) * (pnt_x - orig_x) + math.cos(angle) * (pnt_y - orig_y)
    return int(ret_x), int(ret_y)


def rotate_image_point_90_degrees_counterclockwise(point, image_size):
    """
    Rotate a pixel in an image counterclockwise by 90 degrees.

    Args:
        point (tuple): Point to rotate
        image_size (tuple): Size of image in pixels
    """
    # x and y values are counted from the top left corner. Thus, the image is in the fourth quadrant.
    # We subtract the image height from y to make it correspond to a cartesian plane.
    h, w = image_size
    x, y = point[0], point[1] - h
    x, y = rotate_about_origin((x, y), math.pi / 2)
    return x, w - y


def get_object_position(raw_depth_image, depth_image_size, rotated_bbox_centre, camera_intrinsics, roi_size):
    """
    Given a bounding box extracted from a 90-degree clockwise rotated image and camera intrinsics,
    return the real-world object position with respect to the camera frame.

    Args:
        depth_image_pixels (np.array): The raw depth image in a NumPy array
        depth_image_size (tuple): Height and width of the depth image
        rotated_bbox_centre (tuple): Centre of the object bounding box from rotated image
        camera_intrinsics (dict): Intrinsics of the camera used
    """
    # The image used for detection is rotated 90 degrees clockwise. We correct the rotation by rotating
    # the bounding box points in the other direction.
    bbox_centre = rotate_image_point_90_degrees_counterclockwise(rotated_bbox_centre, depth_image_size)

    # unpack the images.
    try:
        bridge = CvBridge()
        # Get the depth data from the region in the bounding box. ROI is 1, depth scale is 999 (From image info)
        depth = get_distance_to_bbox_centre(raw_depth_image, bbox_centre, 1, 999)

        if depth >= 4.0:
            # Not enough depth data.
            print('Not enough depth data.')
            return False

        tform_x = \
            depth * (bbox_centre[0] - camera_intrinsics['principal_x']) / camera_intrinsics['focal_x']
        tform_y = \
            depth * (bbox_centre[1] - camera_intrinsics['principal_y']) / camera_intrinsics['focal_y']
        tform_z = depth
        obj_tform_camera = SE3Pose(tform_x, tform_y, tform_z, Quat()).position

        return obj_tform_camera
    except Exception as exc:  # pylint: disable=broad-except
        print(f'Error getting object position: {exc}')
        return


class ObjectLocaliser:
    def __init__(self, roi_size=1):
        self.rotated_bbox_centre = None
        self.detection = None
        self.object_region = []

        self.camera_intrinsics = {}

        rospy.init_node('object_localiser', anonymous=True)

        self.depth_sub = rospy.Subscriber(depth_image_topic, Image, self.callback_depth_image, roi_size, queue_size=1)
        self.detectnet_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.callback_detectnet, queue_size=1)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.object_point_pub = rospy.Publisher("/object_position", PointStamped, queue_size=1)
        self.listener = tf.TransformListener()

    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp

    def camera_info_callback(self, camera_info_msg):
        if self.intrinsics_exist:
            return
        
        self.width, self.height = camera_info_msg.width, camera_info_msg.height
        self.camera_intrinsics = {
            'focal_x': camera_info_msg.K[0],
            'focal_y': camera_info_msg.K[4],
            'principal_x': camera_info_msg.K[2],
            'principal_y': camera_info_msg.K[5]
        }
        self.intrinsics_exist = True

    def callback_depth_image(self, depth_image_msg, args):
        if not self.rotated_bbox_centre:
            return

        bridge = CvBridge()
        raw_depth_image = bridge.imgmsg_to_cv2(depth_image_msg.data)

        object_in_camera_frame = get_object_position(raw_depth_image, depth_image_size, self.rotated_bbox_centre, camera_intrinsics, args[0])

        object_point = PointStamped()
        try:
            object_point.header.stamp = self.tf_timestamp
            object_point.point = self.listener.transformPoint("/vision_odometry_frame", centre)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)

        if not object_point.point:
            print("The object is not in a valid position.")
            self.bbox = None
            sys.exit(1)
        if math.isnan(object_point.point.x) or math.isnan(object_point.point.y) or math.isnan(object_point.point.z):
            print("The point %s could not be transformed.", str(centre.point))
            self.bbox = None
            sys.exit(1)
        
        self.object_point_pub.publish(object_point)
        
        self.bbox = None
  
    def callback_detectnet(self, detections_msg, args):
        detection = detections_msg.detections[0]
        self.rotated_bbox_centre = (
            detection.bbox.center.x,
            detection.bbox.center.y
        )


def main(args):
    ObjectLocaliser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
