#! /usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Vector3

import rospy
import tf
import sys
import math
import numpy as np

from spot_driver.msg import KinematicState
from spot_driver.srv import Trajectory
from tf2_msgs.msg import TFMessage

sources = ['frontleft', 'frontright']

source = sources[1]

depth_image_topic = '/depth/' + source + '/image'
detection_topic = '/detectnet/detections'
camera_info_topic = '/depth/' + source + '/camera_info'

final_frame_id = "/vision_odometry_frame"


class SpotMover:
    def __init__(self):
        self.position = None

        rospy.init_node('spot_mover', anonymous=True)

        self.kinematic_state_sub = rospy.Subscriber('/kinematic_state', KinematicState, self.kinematic_state_callback, queue_size=1)
        self.object_point_sub = rospy.Subscriber('/object_point/camera_frame', PointStamped, self.object_point_callback, queue_size=1)
        rospy.loginfo("Publishing halfway point to /halfway_point.")
        self.halfway_pub = rospy.Publisher('/halfway_point', PointStamped, queue_size=1)
        self.trajectory_cmd = rospy.ServiceProxy('trajectory_cmd', Trajectory)
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
        try:
            self.listener.waitForTransform('vision_odometry_frame', 'frontleft_fisheye_image', rospy.Time.now(), rospy.Duration(4.0))
            self.position = self.listener.transformPoint('frontleft_fisheye_image', self.position)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
        

    def object_point_callback(self, object_point):
        if not self.position:
            return
        
        vector = Vector3()
        vector.x = object_point.point.x - self.position.point.x
        vector.y = object_point.point.y - self.position.point.y
        vector.z = object_point.point.z - self.position.point.z

        # print("Vector from Spot to the bear:\n", vector)

        halfway = PointStamped()
        halfway.header = object_point.header
        halfway.point.x = self.position.point.x + 0.5 * vector.x
        halfway.point.y = self.position.point.y + 0.5 * vector.y
        halfway.point.z = self.position.point.z + 0.5 * vector.z

        self.halfway_pub.publish(halfway)

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
