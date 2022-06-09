#! /usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Vector3, PoseArray, Pose

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

rviz = True

depth_image_topic = '/depth/' + source + '/image'
detection_topic = '/detectnet/detections'
camera_info_topic = '/depth/' + source + '/camera_info'

final_frame_id = "vision_odometry_frame"
# rviz_frame_id = "frontleft_fisheye_image"
rviz_frame_id = "kinova_camera_color_frame"


class SpotMover:
    def __init__(self):
        self.position = None

        rospy.init_node('spot_mover', anonymous=True)

        self.kinematic_state_sub = rospy.Subscriber('/kinematic_state', KinematicState, self.kinematic_state_callback, queue_size=1)
        self.object_point_sub = rospy.Subscriber('/object_point/odometry_frame', PointStamped, self.object_point_callback, queue_size=1)
        rospy.loginfo("Publishing halfway point to /halfway_point.")
        self.halfway_pub = rospy.Publisher('/halfway_point', PointStamped, queue_size=1)
        self.trajectory_cmd = rospy.ServiceProxy('trajectory_cmd', spot_driver.srv.Trajectory)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_listener, queue_size=1)
        self.listener = tf.TransformListener()
    
    def tf_listener(self, tf_msg):
        self.tf_timestamp = tf_msg.transforms[0].header.stamp

    def kinematic_state_callback(self, kinematic_state):
        self.position = PointStamped()
        self.position.header.frame_id = 'vision_odometry_frame'
        self.position.point.x = kinematic_state.odom_tform_body.translation.x
        self.position.point.y = kinematic_state.odom_tform_body.translation.y
        self.position.point.z = kinematic_state.odom_tform_body.translation.z
        

    def object_point_callback(self, object_point):
        if not self.position:
            return

        print("\n\n-----------------------\n-----------------------\n\nSpot's position:\n", self.position)
        print("\n---\n\nObject position:\n", object_point)
        
        vector = Vector3()
        vector.x = object_point.point.x - self.position.point.x
        vector.y = object_point.point.y - self.position.point.y
        vector.z = object_point.point.z - self.position.point.z

        print("\n---\n\nVector:\n", vector)

        halfway = PointStamped()
        halfway.header = object_point.header
        halfway.point.x = self.position.point.x + 0.5 * vector.x
        halfway.point.y = self.position.point.y + 0.5 * vector.y
        # halfway.point.z = self.position.point.z + 0.5 * vector.z

        print("\n---\n\nHalfway point:\n", halfway)

        if rviz:
            try:
                self.listener.waitForTransform(final_frame_id, rviz_frame_id, rospy.Time.now(), rospy.Duration(4.0))
                halfway = self.listener.transformPoint(rviz_frame_id, halfway)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)

        self.halfway_pub.publish(halfway)

        trajectory = spot_driver.srv.TrajectoryRequest()
        point_to_move = input("Press 1 to move Spot to the object. Press 5 to move him halfway to the object.\n")

        if point_to_move == '1':
            waypoints = PoseArray()
            object_point_2d = Pose()
            object_point_2d.position.x = object_point.point.x
            object_point_2d.position.y = object_point.point.y
            waypoints.poses.append(object_point_2d)
            trajectory.waypoints = waypoints

            print(trajectory.waypoints)

            try:
                rospy.wait_for_service('trajectory_cmd', timeout=2.0)
                self.trajectory_cmd(trajectory)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e, end='')

        # print("Position halfway between Spot and the bear:\n", halfway, '\n')

        # trajectory_msg = Trajectory
        # print(trajectory_msg.final_pose)
        # trajectory_msg.waypoints.poses[0].position = halfway

        # response = self.trajectory_cmd()
        # print(trajectory_msg, '\n')


def main(args):
    SpotMover()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image detection processor module")


if __name__ == "__main__":
    main(sys.argv)
