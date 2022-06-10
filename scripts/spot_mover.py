#! /usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Vector3, PoseArray, Pose, PoseStamped

import rospy
import tf
import sys
import math
import numpy as np

from std_msgs.msg import String

from spot_driver.msg import KinematicState
import spot_driver.srv
from tf2_msgs.msg import TFMessage

sources = ['frontleft', 'frontright']

source = sources[1]

rviz = False

depth_image_topic = '/depth/' + source + '/image'
detection_topic = '/detectnet/detections'
camera_info_topic = '/depth/' + source + '/camera_info'

final_frame_id = "vision_odometry_frame"
# rviz_frame_id = "frontleft_fisheye_image"
rviz_frame_id = "kinova_camera_color_frame"


class SpotMover:
    def __init__(self):
        self.pose = None
        self.prev_pose = None

        rospy.init_node('spot_mover', anonymous=True)

        self.kinematic_state_sub = rospy.Subscriber('/kinematic_state', KinematicState, self.kinematic_state_callback, queue_size=1)
        self.object_point_sub = rospy.Subscriber('/object_point/odometry_frame', PointStamped, self.object_point_callback, queue_size=1)
        self.spot_move_sub = rospy.Subscriber('/move_spot', String, self.spot_move_callback, queue_size=1)

        self.proximity_pub = rospy.Publisher('/waypoints/self.proximity', PoseStamped, queue_size=1)
        self.artificial_object_point_pub = rospy.Publisher('/move_spot/artificial_object_point', PointStamped, queue_size=1)
        
        self.trajectory_cmd = rospy.ServiceProxy('trajectory_cmd', spot_driver.srv.Trajectory)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.listener = tf.TransformListener()

        # Create trajectory request for service
        self.trajectory = spot_driver.srv.TrajectoryRequest()
        self.goal_pose_2d = Pose()
        self.trajectory.waypoints = PoseArray()
        self.trajectory.waypoints.poses.append(self.goal_pose_2d)
    
    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp
    
    def spot_move_callback(self, _):
        if not self.proximity:
            return

        print(self.trajectory.waypoints)
        self.prev_pose = self.pose

        try:
            rospy.wait_for_service('trajectory_cmd', timeout=2.0)
            self.trajectory_cmd(self.trajectory)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e, end='')
            return

    def kinematic_state_callback(self, kinematic_state):
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'vision_odometry_frame'
        self.pose.pose.position.x = kinematic_state.vision_tform_body.translation.x
        self.pose.pose.position.y = kinematic_state.vision_tform_body.translation.y
        self.pose.pose.position.z = kinematic_state.vision_tform_body.translation.z
        self.pose.pose.orientation.x = kinematic_state.vision_tform_body.rotation.x
        self.pose.pose.orientation.y = kinematic_state.vision_tform_body.rotation.y
        self.pose.pose.orientation.z = kinematic_state.vision_tform_body.rotation.z
        self.pose.pose.orientation.w = kinematic_state.vision_tform_body.rotation.w

    def object_point_callback(self, object_point):
        if not self.pose:
            return

        print("\n\n-----------------------\n-----------------------\n\nSpot's position:\n", self.pose.pose)
        print("\n\nObject position:\n", object_point.point)
        
        # Vector from Spot to the object
        vector = Vector3()
        vector.x = object_point.point.x - self.pose.pose.position.x
        vector.y = object_point.point.y - self.pose.pose.position.y
        vector.z = object_point.point.z - self.pose.pose.position.z

        heading = math.atan2(vector.y, vector.x)

        print("\nVector:\n", vector)

        # L2 distance to the object
        ground_distance_to_object = np.sqrt(vector.x ** 2 + vector.y ** 2)
        if ground_distance_to_object < 1.75:
            print("\nDistance between Spot and object is {:.2f} metres.".format(ground_distance_to_object))
            print("Spot needs to be at least 1.75 metres away from the object.")
            return

        # Now we create an artificial position for the object in front of spot.
        # This is where the arm will be moving towards.
        distance_correction_ratio = (ground_distance_to_object - 1.75) / ground_distance_to_object

        artificial_object_point = PointStamped()
        artificial_object_point.header = object_point.header
        artificial_object_point.point.x = self.pose.pose.position.x + distance_correction_ratio * vector.x
        artificial_object_point.point.y = self.pose.pose.position.y + distance_correction_ratio * vector.y
        artificial_object_point.point.z = self.pose.pose.position.y + distance_correction_ratio * vector.z

        self.artificial_object_point_pub.publish(artificial_object_point)

        self.proximity = PoseStamped()
        self.proximity.header = object_point.header
        self.proximity.pose.position.x = object_point.point.x - distance_correction_ratio * vector.x
        self.proximity.pose.position.y = object_point.point.y - distance_correction_ratio * vector.y

        self.proximity.pose.orientation.x = heading
        self.proximity.pose.orientation.y = 0
        self.proximity.pose.orientation.z = 0
        self.proximity.pose.orientation.w = 0

        print("\nProximity point:\n", self.proximity.pose)
        
        # RViz does not display points in the vision odometry frame, so transform them to the camera frame before publishing
        if rviz:
            try:
                self.listener.waitForTransform(final_frame_id, rviz_frame_id, rospy.Time.now(), rospy.Duration(4.0))
                halfway = self.listener.transformPose(rviz_frame_id, halfway)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                return

        point_to_move = input("\n-----------------------\n-----------------------\n\n    \
Key legend:\n \
    1 - Grasp!\n \
    2 - Move to grasping distance from the object,\n" + \
("    3 - Move to the previous position,\n" if self.prev_pose else "") + \
"     Any other key - Refresh positions\n\n")
        print("\n\n-----------------------\n-----------------------\n\n")

        if point_to_move == '1':
            print("Moving Spot and grasping...")
            self.goal_pose_2d.position = object_point.point
            self.proximity_pub.publish(self.proximity)
        elif point_to_move == '2':
            print("Moving Spot to grasping distance from the object...")
            self.goal_pose_2d = self.proximity.pose
        elif point_to_move == '3' and self.prev_pose:
            print("Moving Spot to its previous position...")
            self.goal_pose_2d = self.prev_pose.pose
        else:
            print("Skipping the object...")
        

def main(args):
    SpotMover()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
