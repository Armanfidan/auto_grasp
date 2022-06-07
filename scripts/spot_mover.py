#! /usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Vector3

import rospy
import tf
import sys
import math
import numpy as np

from spot_driver.msg import KinematicState
from spot_driver.srv import Trajectory

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
        self.object_point_sub = rospy.Subscriber('/object_point/odometry_frame', PointStamped, self.object_point_callback, queue_size=1)
        self.trajectory_cmd = rospy.ServiceProxy('trajectory_cmd', Trajectory)

    def kinematic_state_callback(self, kinematic_state):
        self.position = Point()
        self.position.x = kinematic_state.odom_tform_body.translation.x
        self.position.y = kinematic_state.odom_tform_body.translation.y
        self.position.z = kinematic_state.odom_tform_body.translation.z

    def object_point_callback(self, object_point):
        if not self.position:
            return
        
        vector = Vector3()
        vector.x = object_point.point.x - self.position.x
        vector.y = object_point.point.y - self.position.y
        vector.z = object_point.point.z - self.position.z

        print("Vector from Spot to the bear:\n", vector)

        halfway = Point()
        halfway.x = self.position.x + 0.5 * vector.x
        halfway.y = self.position.y + 0.5 * vector.y
        halfway.z = self.position.z + 0.5 * vector.z

        print("Position halfway between Spot and the bear:\n", halfway, '\n')

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
