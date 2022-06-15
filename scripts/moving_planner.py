#! /usr/bin/env python3

import os
import io
import sys
import copy
import math
import numpy as np

import rospy
import tf
import tf2_ros
import moveit_commander
from actionlib import SimpleActionClient

from std_msgs.msg import String
from moveit_msgs.msg import Grasp, Constraints, OrientationConstraint, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes

from std_msgs.msg import String, Header, Bool
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, TransformStamped, PoseStamped, Vector3, Transform
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs.point_cloud2 as pc2

from vision_msgs.msg import Detection2DArray, Detection2D

from world import Object, World
from kinova_gen3 import KinovaGen3
from auto_grasp.msg import CurrentState
from auto_grasp.srv import AddFloorPlane, RemoveFloorPlane
from behaviour_helpers import known_joint_positions, spok_top_grasp_pose
from spherical_grasps_server import SphericalGrasps

from spot_driver.srv import Stand, StandRequest
from cv_bridge import CvBridge

from bosdyn.client.math_helpers import Quat, SE3Pose


object_point_topic = '/object_point/odometry_frame'
artificial_object_point_topic = '/move_spot/artificial_object_point'
spot_move_topic = '/move_spot/'


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name



class Planner():
    def __init__(self):
        
        self.object_class = ""
        self.grasp = False
        self.first_stage_complete = False

        # Create robot object
        self.kinova_gen3 = KinovaGen3(rospy.get_namespace(), True, "kinova_finger_joint", 7)

        # Create world object
        self.world = World()
        
        self.scene = moveit_commander.PlanningSceneInterface()
        # Get the links of the end effector excluded from collisions
        self.touch_links = self.kinova_gen3.robot.get_link_names(group="gripper")

        self.artificial_object_point_sub = \
            rospy.Subscriber(artificial_object_point_topic, PointStamped, self.artificial_object_point_callback, queue_size=1)
        self.object_point_sub = \
            rospy.Subscriber(object_point_topic, PointStamped, self.object_point_callback, queue_size=1)

        self.spot_move_pub = rospy.Publisher(spot_move_topic, String, queue_size=1)
        self.switch_detectnet_pub = rospy.Publisher('/switch_detectnet', String, queue_size=1)

        rospy.init_node('planner', anonymous=True)
        
        self.listener = tf.TransformListener()

        rospy.loginfo("Initalizing grasp generator...")
        self.sg = SphericalGrasps()
        rospy.loginfo("Grasp generator initialised.")

        rospy.loginfo("Connecting to pickup action server...")
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Connected to pickup action server.")
        
        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        self.clear_octomap.wait_for_service()
        rospy.loginfo("Connected to clear octomap service.")

        self.stand_srv_prox = rospy.ServiceProxy("stand_cmd", Stand)
        self.stand_srv_req = StandRequest()

        print("Going home...")
        self.go_to_joint_position(known_joint_positions['home2'], tolerance=0.01)
        print("I'm home.")

    def artificial_object_point_callback(self, object_position):
        if self.first_stage_complete:
            reset = input("Reset grasp? y/n")
            if reset == 'y':
                self.first_stage_complete = False
            else:
                return
        print("\nRECEIVED ARTIFICIAL OBJECT POINT. GRASPING!\n")
        self.grasp_object(object_position)

    def object_point_callback(self, object_point):
        # print("object point received. Is the first stage complete?", self.first_stage_complete)
        if not self.first_stage_complete:
            return
        print("\nOBJECT POINT RECEIVED:\n")
        grasp = False
        switch = False
        while not (grasp or switch):
            print(object_point.point)
            inp = input("\nGrasp this point?\ny to grasp,\ns to switch to Kinova camera and reset grasp,\nany other key to skip\n")
            grasp  = inp == 'y'
            switch = inp == 's'
        if switch:
            self.switch_detectnet_pub.publish(String("kinova"))
            self.first_stage_complete = False
        else:
            self.grasp_object(object_point)

    def grasp_object(self, _pointStamped, object_class="TeddyBear"):
        rospy.loginfo("Removing any previous objects")
        self.scene.remove_attached_object("kinova_end_effector_link")
        self.scene.remove_world_object(object_class)
        self.scene.remove_world_object("floor")
        #self.scene.remove_world_object("table")
        rospy.loginfo("Clearing octomap")
        #self.clear_octomap()
        rospy.sleep(2) # Removing is fast

        rospy.loginfo("Adding new object: %s", object_class)

        _pose = PoseStamped()
        _pose.header = _pointStamped.header
        _pose.pose.position = _pointStamped.point
        
        # Correct offset added by the depth sensor and/or hand-camera calibration
        # _pose.pose.position.z += 0.05
        # _pose.pose.position.y += 0.065
        # _pose.pose.position.x += 0.05
        
        # Adjusted empirically, might not be correct since odometry frame might have moved.
        # Tweak as required.
        _pose.pose.position.z += 0.01
        _pose.pose.position.y -= 0.02
        # _pose.pose.position.x -= 0.02
        _pose.pose.orientation = Quaternion()
        rospy.loginfo("Object pose: %s", _pose.pose)

        #Add object description in scene
        self.add_collision_objects(_pose, object_class)
        #Pick
        self.pick(_pose, object_class)
    
    def pick(self, pose_stamped, object_class):
        rospy.loginfo("Opening gripper")
        self.kinova_gen3.reach_gripper_position(0.1)

        # Create an array of graps to be attempted
        grasps = []
        grasps = self.sg.create_grasps_from_object_pose(pose_stamped)

        # print(grasps[0])

        goal = self.createPickupGoal("arm", object_class, pose_stamped, grasps, self.touch_links)

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        self.spot_move_pub.publish(String("Time to move Spot!"))
        
        rospy.loginfo("Waiting for result")
        self.pickup_ac.wait_for_result()

        result = self.pickup_ac.get_result()
        rospy.logdebug("Using arm result: " + str(result))
        rospy.loginfo("Pick result: " + str(moveit_error_dict[result.error_code.val]))

        # Resume octomap updates 
        self.republish = True
        
        if self.first_stage_complete:
            rospy.loginfo("Going home...")
            self.go_to_joint_position(known_joint_positions['home2'], tolerance=0.01)
            #self.go_to_joint_position(known_joint_positions['home'], tolerance=0.01)
            rospy.loginfo("Opening gripper")

        self.kinova_gen3.reach_gripper_position(0.1)

        self.first_stage_complete = True


        return result.error_code.val
            
    def add_collision_objects(self, pose_stamped, object_class):

        self.scene.add_box("object_space", pose_stamped, size=(0.10, 0.10, 0.10))
        self.scene.add_sphere(object_class, pose_stamped, radius=0.013)

        success = False
        while not success:
            success = self.wait_for_state_update(object_class, object_is_known=True) and \
                    self.wait_for_state_update("object_space", object_is_known=True)
        
        # self.add_floor_plane()
        rospy.sleep(5)
       
        self.scene.remove_world_object("object_space")
        #Stop octomap updates
        self.republish = False
        rospy.sleep(5)
    
    def add_floor_plane(self):
        # Manually define the floor plane
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = "base_link"
        # floor at lower position
        #plane_pose.pose.position = Point(x=0.0, y=0.0, z=-0.475)
        # floor at resting position
        #plane_pose.pose.position = Point(x=0.0, y=0.0, z=-0.2)
        # floor at walking position
        plane_pose.pose.position = Point(x=0.0, y=0.0, z=-0.65)
        # Tool box
        #plane_pose.pose.position = Point(x=0.0, y=0.0, z=0.075)
        plane_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1)
        
        self.scene.add_box("floor", plane_pose, size=(2.0, 2.0, 0.01))

        return self.wait_for_state_update("floor", object_is_known=True)
    
    def remove_floor_plane(self):
        self.scene.remove_world_object("floor")
        return self.wait_for_state_update("floor")

    def wait_for_state_update(self, object_name, object_is_known=False, object_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the plane is in the scene.
            # Note that attaching the plane will remove it from known_objects
            is_known = object_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (object_is_attached == is_attached) and (object_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def go_to_joint_position(self, joint_position_array, tolerance):
        return self.kinova_gen3.reach_joint_angles(joint_position_array, tolerance)
    
    def createPickupGoal(self, group="arm", target="coloredCube",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
        """ Create a PickupGoal with the provided data"""
        print(links_to_allow_contact)
        pug = PickupGoal()
        pug.target_name = target
        pug.group_name = group
        pug.possible_grasps.extend(possible_grasps)
        pug.allowed_planning_time = 35.0
        pug.planning_options.planning_scene_diff.is_diff = True
        pug.planning_options.planning_scene_diff.robot_state.is_diff = True
        pug.planning_options.plan_only = False
        pug.planning_options.replan = True
        pug.planning_options.replan_attempts = 1  # 10
        pug.allowed_touch_objects = []
        pug.attached_object_touch_links = ['<octomap>']
        pug.attached_object_touch_links.extend(links_to_allow_contact)
        
        return pug


if __name__ == "__main__":

    entity_ranking_ros = Planner()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
