#! /usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Vector3, PoseArray, Pose, PoseStamped

import rospy
import tf
import sys
import math
import numpy as np

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

        self.halfway_pub = rospy.Publisher('/waypoints/halfway', PoseStamped, queue_size=1)
        self.proximity_pub = rospy.Publisher('/waypoints/proximity', PoseStamped, queue_size=1)
        
        self.trajectory_cmd = rospy.ServiceProxy('trajectory_cmd', spot_driver.srv.Trajectory)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.listener = tf.TransformListener()
    
    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp

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

        proximity = PoseStamped()
        proximity.header = object_point.header
        proximity.pose.position.x = object_point.point.x - distance_correction_ratio * vector.x
        proximity.pose.position.y = object_point.point.y - distance_correction_ratio * vector.y

        proximity.pose.orientation.x = heading
        proximity.pose.orientation.y = 0
        proximity.pose.orientation.z = 0
        proximity.pose.orientation.w = 0

        print("\nProximity point:\n", proximity.pose)

        # Halfway point between spot and the object
        halfway = PoseStamped()
        halfway.header = object_point.header
        halfway.pose.position.x = self.pose.pose.position.x + 0.5 * vector.x
        halfway.pose.position.y = self.pose.pose.position.y + 0.5 * vector.y
        halfway.pose.orientation.x = self.pose.pose.orientation.x
        halfway.pose.orientation.y = self.pose.pose.orientation.y
        halfway.pose.orientation.z = self.pose.pose.orientation.z
        # halfway.point.z = self.pose.pose.position.z + 0.5 * vector.z

        print("\nHalfway point:\n", halfway.pose)

        # RViz does not display points in the vision odometry frame, so transform them to the camera frame before publishing
        if rviz:
            try:
                self.listener.waitForTransform(final_frame_id, rviz_frame_id, rospy.Time.now(), rospy.Duration(4.0))
                halfway = self.listener.transformPose(rviz_frame_id, halfway)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                return

        self.halfway_pub.publish(halfway)
        self.proximity_pub.publish(proximity)

        # Create trajectory request for service
        trajectory = spot_driver.srv.TrajectoryRequest()
        waypoints = PoseArray()
        goal_pose_2d = Pose()

        point_to_move = input("\n-----------------------\n-----------------------\n\n    \
To move Spot, press:\n \
    - 1 to move to the object,\n \
    - 2 to move halfway to the object,\n \
    - 3 to move to grasping distance from the object,\n" + \
("    - 4 to move to the previous position,\n" if self.prev_pose else "") + \
"     - Any other key to skip this object.\n\n")
        print("\n\n-----------------------\n-----------------------\n\n")

        if point_to_move == '1':
            print("Moving Spot to the object...")
            goal_pose_2d.position = object_point.point
        elif point_to_move == '2':
            print("Moving Spot halfway to the object...")
            goal_pose_2d = halfway.pose
        elif point_to_move == '3':
            print("Moving Spot to grasping distance from the object...")
            goal_pose_2d = proximity.pose
        elif point_to_move == '4' and self.prev_pose:
            print("Moving Spot to its previous position...")
            goal_pose_2d = self.prev_pose.pose
        else:
            print("Skipping the object...")
            return

        waypoints.poses.append(goal_pose_2d)
        trajectory.waypoints = waypoints

        print(trajectory.waypoints)

        self.prev_pose = self.pose

        try:
            rospy.wait_for_service('trajectory_cmd', timeout=2.0)
            self.trajectory_cmd(trajectory)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e, end='')
            return
        

def main(args):
    SpotMover()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
