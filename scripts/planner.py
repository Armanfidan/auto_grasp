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
from spot_driver.msg import KinematicState


click_topic = '/detectnet/overlay_mouse_left'
object_point_topic = '/object_point/odometry_frame'


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name



class Planner():
    def __init__(self):
        self.pose = None
        self.object_class = ""
        self.grasp = False

        # Create robot object
        self.kinova_gen3 = KinovaGen3(rospy.get_namespace(), True, "kinova_finger_joint", 7)

        # Create world object
        self.world = World()
        
        self.scene = moveit_commander.PlanningSceneInterface()
        # Get the links of the end effector excluded from collisions
        self.touch_links = self.kinova_gen3.robot.get_link_names(group="gripper")

        self.click_topic_sub = rospy.Subscriber(click_topic, Point, self.mouseclick_callback, queue_size=1)
        self.object_point_sub = rospy.Subscriber(object_point_topic, PointStamped, self.object_point_callback, queue_size=1)

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

        # print("Going home...")
        # self.go_to_joint_position(known_joint_positions['home2'], tolerance=0.01)
        # print("I'm home.")

    def kinematic_state_callback(self, kinematic_state):
        if not self.pose:
            self.pose = PoseStamped()
        self.pose.header.frame_id = 'vision_odometry_frame'
        self.pose.pose.position.x = kinematic_state.vision_tform_body.translation.x
        self.pose.pose.position.y = kinematic_state.vision_tform_body.translation.y
        self.pose.pose.position.z = kinematic_state.vision_tform_body.translation.z
        self.pose.pose.orientation.x = kinematic_state.vision_tform_body.rotation.x
        self.pose.pose.orientation.y = kinematic_state.vision_tform_body.rotation.y
        self.pose.pose.orientation.z = kinematic_state.vision_tform_body.rotation.z
        self.pose.pose.orientation.w = kinematic_state.vision_tform_body.rotation.w

    def mouseclick_callback(self, object_position):
        print("Clicked!")
        self.grasp = True

    def object_point_callback(self, object_point):
        if not self.grasp:
            return
        self.grasp = False
        print("Received object point:\n", object_point)
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
        # _pose.pose.position.z += 0.075
        # _pose.pose.position.y += 0.065
        # _pose.pose.position.x += 0.05
        
        # Adjusted empirically, might not be correct since odometry frame might have moved.
        # Tweak as required.
        _pose.pose.position.z -= 0.02
        _pose.pose.position.y -= 0.02
        _pose.pose.position.x -= 0.02
        _pose.pose.orientation = Quaternion()
        rospy.loginfo("Object pose: %s", _pose.pose)

        #Add object description in scene
        self.add_collision_objects(_pose, object_class)
        #Pick
        self.pick(_pose, object_class)
    
    def grasp_with_constraints(self, _pointStamped, object_class="TeddyBear"):
        rospy.loginfo("Removing any previous objects")
        self.scene.remove_attached_object("kinova_end_effector_link")
        self.scene.remove_world_object(object_class)
        rospy.sleep(2) # Removing is fast

        # Get current pose
        actual_pose = self.kinova_gen3.get_cartesian_pose()
        # Set target position
        actual_pose.position = _pointStamped.point

        # Orientation constraint (we want the end effector to stay the same orientation)
        constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.orientation = actual_pose.orientation
        constraints.orientation_constraints.append(orientation_constraint)

        rospy.loginfo("Opening gripper")
        self.kinova_gen3.reach_gripper_position(0.1)

        _pose = PoseStamped()
        _pose.header = _pointStamped.header
        _pose.pose.position = _pointStamped.point
        # Correct offset added by the depth sensor and/or hand-camera calibration
        _pose.pose.position.z += 0.075
        _pose.pose.position.y += 0.065
        _pose.pose.position.x += 0.05
        _pose.pose.orientation = Quaternion()
        rospy.loginfo("Object pose: %s", _pose.pose)

        #Add object description in scene
        self.add_collision_objects(_pose, object_class)

        #self.kinova_gen3.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
        self.kinova_gen3.reach_cartesian_pose(pose=actual_pose, tolerance=0.01)
        
        rospy.loginfo("Closing the gripper 95%...")
        print(self.kinova_gen3.reach_gripper_position(0.95))

        # Resume octomap updates 
        self.republish = True

        rospy.loginfo("Going home...")
        self.go_to_joint_position(known_joint_positions['home2'], tolerance=0.01)
        rospy.loginfo("Opening gripper")
        self.kinova_gen3.reach_gripper_position(0.1)

    def pick(self, pose_stamped, object_class):
        rospy.loginfo("Opening gripper")
        self.kinova_gen3.reach_gripper_position(0.1)

        # Create an array of graps to be attempted
        grasps = []
        grasps = self.sg.create_grasps_from_object_pose(pose_stamped)
        for grasp in grasps:
            # pass
            # grasp.grasp_pose.pose.orientation.x = self.pose.pose.orientation.x
            grasp.grasp_pose.pose.orientation.y = self.pose.pose.orientation.y

        goal = self.createPickupGoal("arm", object_class, pose_stamped, grasps, self.touch_links)

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        rospy.loginfo("Waiting for result")
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.logdebug("Using arm result: " + str(result))
        rospy.loginfo("Pick result: " + str(moveit_error_dict[result.error_code.val]))

        # Resume octomap updates 
        self.republish = True

        rospy.loginfo("Going home...")
        self.go_to_joint_position(known_joint_positions['home2'], tolerance=0.01)
        #self.go_to_joint_position(known_joint_positions['home'], tolerance=0.01)
        rospy.loginfo("Opening gripper")
        self.kinova_gen3.reach_gripper_position(0.1)

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
    
    def look_at_object(self, object_pose):
        current_pose = self.kinova_gen3.arm_group.get_current_pose().pose
        dir_vector = Vector3(object_pose.position.x - current_pose.position.x, 
                            object_pose.position.y - current_pose.position.y, 
                            object_pose.position.z - current_pose.position.z)
        target_pose = current_pose
        target_pose.orientation = self.look_rotation(forward=dir_vector, up=Vector3(0,0,1))
        return self.kinova_gen3.reach_cartesian_pose(pose=target_pose, tolerance=0.02)

    # from https://github.com/UTNuclearRoboticsPublic/look_at_pose/blob/kinetic/nodes/look_at_pose_server
    def normalise(self, input_vector):
        mag = (input_vector.x**2+input_vector.y**2+input_vector.z**2)**0.5
        unit_vector = Vector3()
        unit_vector.x = input_vector.x/mag
        unit_vector.y = input_vector.y/mag
        unit_vector.z = input_vector.z/mag
        return unit_vector
    
    # from https://github.com/UTNuclearRoboticsPublic/look_at_pose/blob/kinetic/nodes/look_at_pose_server
    def cross_product(self, u, v):
        cross = Vector3()
        cross.x = u.y*v.z-v.y*u.z
        cross.y = v.x*u.z-u.x*v.z
        cross.z = u.x*v.y-u.y*v.x
        return cross

    # from http://answers.unity3d.com/questions/467614/what-is-the-source-code-of-quaternionlookrotation.html
    def look_rotation(self, forward, up):
        forward = self.normalise(forward)
        right = self.normalise(self.cross_product(up,forward))
        up = self.cross_product(forward, right)
        
        m00 = right.x
        m01 = right.y
        m02 = right.z
        m10 = up.x
        m11 = up.y
        m12 = up.z
        m20 = forward.x
        m21 = forward.y
        m22 = forward.z
        
        num8 = (m00 + m11) + m22
        quaternion = Quaternion()
        if (num8 > 0.0):
            num = (num8 + 1.0)**0.5
            quaternion.w = num * 0.5
            num = 0.5 / num
            quaternion.x = (m12 - m21) * num
            quaternion.y = (m20 - m02) * num
            quaternion.z = (m01 - m10) * num
            return quaternion
            
        if ((m00 >= m11) and (m00 >= m22)):
            num7 = (((1.0 + m00) - m11) - m22)**0.5
            num4 = 0.5 / num7
            quaternion.x = 0.5 * num7
            quaternion.y = (m01 + m10) * num4
            quaternion.z = (m02 + m20) * num4
            quaternion.w = (m12 - m21) * num4
            return quaternion
        
        if (m11 > m22):
            num6 = (((1.0 + m11) - m00) - m22)**0.5
            num3 = 0.5 / num6
            quaternion.x = (m10 + m01) * num3
            quaternion.y = 0.5 * num6
            quaternion.z = (m21 + m12) * num3
            quaternion.w = (m20 - m02) * num3
            return quaternion
        
        num5 = (((1.0 + m22) - m00) - m11)**0.5
        num2 = 0.5 / num5
        quaternion.x = (m20 + m02) * num2
        quaternion.y = (m21 + m12) * num2
        quaternion.z = 0.5 * num5
        quaternion.w = (m01 - m10) * num2
        return quaternion

    def spot_height(self, value):
        tf = Transform()
        tf.translation.z = value
        
        self.stand_srv_req.body_pose.translation = tf.translation
        self.stand_srv_req.body_pose.rotation = tf.rotation

        try:
            rospy.wait_for_service("stand_cmd", timeout=2.0)
            self.stand_srv_prox(self.stand_srv_req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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
