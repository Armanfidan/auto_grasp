#! /usr/bin/env python3

from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
import sensor_msgs.point_cloud2 as pc2
import rospy
import tf
import sys
import math


class GetDepth:
    def __init__(self, region_size):
        self.bbox = None
        self.detection = None
        self.object_region = []
        rospy.init_node('image_accumulator', anonymous=True)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback_pc, queue_size=1)
        self.detectnet_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray, self.callback_detectnet, region_size, queue_size=1)
        self.pub = rospy.Publisher("/auto_grasp/grasp_data", Detection2D, queue_size=1)
        self.listener = tf.TransformListener()

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
                    centre = self.listener.transformPoint("/base_link", centre)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)
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


