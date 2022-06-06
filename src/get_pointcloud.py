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

# Mapping from visual to depth data
VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE = {
    'frontleft_fisheye_image': 'frontleft_depth_in_visual_frame',
    'frontright_fisheye_image': 'frontright_depth_in_visual_frame'
}

ROTATION_ANGLES = {
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102
}


def get_distance_to_closest_object_depth(x_min, x_max, y_min, y_max, depth_scale, raw_depth_image,
                                         histogram_bin_size=0.20, minimum_number_of_points=100,
                                         max_distance=4.0):
    """Make a histogram of distances to points in the cloud and take the closest distance with
    enough points.

    Args:
        origin (tuple): Origin to rotate the point around
        x_min (int): minimum x coordinate (column) of object to find
        x_max (int): maximum x coordinate (column) of object to find
        y_min (int): minimum y coordinate (row) of object to find
        y_max (int): maximum y coordinate (row) of object to find
        depth_scale (float): depth scale of the image to convert from sensor value to meters
        raw_depth_image (np.array): matrix of depth pixels
        histogram_bin_size (float): size of each bin of distances
        minimum_number_of_points (int): minimum number of points before returning depth
        max_distance (float): maximum distance to object in meters
    """
    num_bins = math.ceil(max_distance / histogram_bin_size)
    depths = []

    for row in range(y_min, y_max):
        for col in range(x_min, x_max):
            raw_depth = raw_depth_image[row][col]
            if raw_depth != 0 and raw_depth is not None:
                depth = raw_depth / depth_scale
                depths.append(depth)

    hist, hist_edges = np.histogram(depths, bins=num_bins, range=(0, max_distance))

    edges_zipped = zip(hist_edges[:-1], hist_edges[1:])
    # Iterate over the histogram and return the first distance with enough points.
    for entry, edges in zip(hist, edges_zipped):
        if entry > minimum_number_of_points:
            return statistics.mean([d for d in depths if d > edges[0] and d > edges[1]])

    return max_distance


def get_object_position(world_tform_cam, visual_image, depth_image, bounding_box, rotation_angle):
    """
    Extract the bounding box, then find the mode in that region.

    Args:
        world_tform_cam (SE3Pose): SE3 transform from world to camera frame
        visual_image (ImageResponse): From a visual camera
        depth_image (ImageResponse): From a depth camera corresponding to the visual_image
        bounding_box (list): Bounding box from tensorflow
        rotation_angle (float): Angle (in degrees) to rotate depth image to match cam image rotation
    """

    # Make sure there are two images.
    if visual_image is None or depth_image is None:
        # Fail.
        return

    # Rotate bounding box back to original frame
    points = [(bounding_box[1], bounding_box[0]), (bounding_box[3], bounding_box[0]),
              (bounding_box[3], bounding_box[2]), (bounding_box[1], bounding_box[2])]

    origin = (visual_image.shot.image.cols / 2, visual_image.shot.image.rows / 2)

    points_rot = [rotate_about_origin_degrees(origin, point, rotation_angle) for point in points]

    # Get the bounding box corners.
    y_min = max(0, min([point[1] for point in points_rot]))
    x_min = max(0, min([point[0] for point in points_rot]))
    y_max = min(visual_image.shot.image.rows, max([point[1] for point in points_rot]))
    x_max = min(visual_image.shot.image.cols, max([point[0] for point in points_rot]))

    # Check that the bounding box is valid.
    if (x_min < 0 or y_min < 0 or x_max > visual_image.shot.image.cols or
            y_max > visual_image.shot.image.rows):
        print(f'Bounding box is invalid: ({x_min}, {y_min}) | ({x_max}, {y_max})')
        print(f'Bounds: ({visual_image.shot.image.cols}, {visual_image.shot.image.rows})')
        return

    # Unpack the images.
    try:
        if depth_image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(depth_image.shot.image.data, dtype=dtype)
        if depth_image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(depth_image.shot.image.rows, depth_image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)
        depth_image_pixels = img

        # Get the depth data from the region in the bounding box.
        depth = get_distance_to_closest_object_depth(x_min, x_max, y_min, y_max,
                                                     depth_image.source.depth_scale,
                                                     depth_image_pixels)

        if depth >= 4.0:
            # Not enough depth data.
            print('Not enough depth data.')
            return False

        # Calculate the transform to the center point of the object using camera intrinsics
        # and depth calculated earlier in the function
        focal_x = depth_image.source.pinhole.intrinsics.focal_length.x
        principal_x = depth_image.source.pinhole.intrinsics.principal_point.x

        focal_y = depth_image.source.pinhole.intrinsics.focal_length.y
        principal_y = depth_image.source.pinhole.intrinsics.principal_point.y

        center_x = round((x_max - x_min) / 2.0 + x_min)
        center_y = round((y_max - y_min) / 2.0 + y_min)

        tform_x = depth * (center_x - principal_x) / focal_x
        tform_y = depth * (center_y - principal_y) / focal_y
        tform_z = depth
        obj_tform_camera = SE3Pose(tform_x, tform_y, tform_z, Quat())

        return world_tform_cam * obj_tform_camera
    except Exception as exc:  # pylint: disable=broad-except
        print(f'Error getting object position: {exc}')
        return


class GetDepth:
    def __init__(self, region_size):
        self.bbox = None
        self.detection = None
        self.object_region = []
        rospy.init_node('image_accumulator', anonymous=True)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback_pc, queue_size=1)
        self.detectnet_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray, self.callback_detectnet, region_size, queue_size=1)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.pub = rospy.Publisher("/auto_grasp/grasp_data", Detection2D, queue_size=1)
        self.listener = tf.TransformListener()

    def tf_listener(self, msg):
        print("Message:", msg)
        self.tf_timestamp = msg.transforms[0].header.stamp.secs


    def callback_pc(self, pointcloud):
        if not self.bbox:
            return
        
        points = pc2.read_points(pointcloud, field_names=('x', 'y', 'z'), uvs=self.object_region)
        centre = PointStamped()
        centre.header = pointcloud.header
        centre.point = None

        untransformable = False
        for point in points:
            x, y, z = point
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                print(untransformable)
                centre.point = Point(x, y, z)
                try:
                    centre.header.stamp.secs = self.tf_timestamp
                    centre = self.listener.transformPoint("/base_link", centre)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)
                    print(centre.header)

                    untransformable = True
                    continue
                untransformable = False
                break

        if not centre.point:
            print("The object is not in a valid position.")
            self.bbox = None
            sys.exit(1)
        if untransformable or math.isnan(centre.point.x) or math.isnan(centre.point.y) or math.isnan(centre.point.z):
            print("The point %s could not be transformed.", str(centre.point))
            self.bbox = None
            sys.exit(1)
        

        
        # TODO: Uncomment this, commented only for debugging purposes
        # z = z + max(self.bbox.size_x, self.bbox.size_y) // 2

        # Creating the grasp message.
        grasp_msg = Detection2D()
        grasp_msg.header = self.detection.header
        grasp_msg.results = self.detection.results
        grasp_msg.bbox = self.bbox
        # I will use grasp_msg.pose.pose.position.z to carry the depth information to avoid switching from BoundingBox2D
        # to BoundingBox3D and adding complexity.
        grasp_msg.results[0].pose.pose.position.z = centre.point.z
        print("Generated sphere: (" + str(self.bbox.center.x) + "," + str(self.bbox.center.y) + "," + str(z) + "), w=" + str(self.bbox.size_x) + ", h=" + str(self.bbox.size_y))
        self.pub.publish(grasp_msg)
        
        self.bbox = None
  
    def callback_detectnet(self, detections_msg, args):
        region_size = args[0]
        self.detection = detections_msg.detections[0]
        self.bbox = self.detection.bbox
        
        self.object_region = []

        y = int(self.bbox.center.y)
        x = int(self.bbox.center.x)

        for j in range(y - region_size, y + region_size + 1):
            for i in range(x - region_size, x + region_size + 1):
                self.object_region.append((i, j))


def main(args):
    GetDepth([2])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
