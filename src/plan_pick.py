#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseArray


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MotionPlanner(object):
    def __init__(self):
        super(MotionPlanner, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('motion_planner', anonymous=True)
        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        # Provide remote interface for getting, setting and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        # Interface to a planning group (group of joints). This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',  moveit_msgs.msg.DisplayTrajectory,   queue_size=20)

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()

        print("Planning frame:", self.planning_frame)
        print("End effector link:", self.eef_link)
        print("Available Planning Groups:", self.group_names)
        print("Robot state:", robot.get_current_state())

    def go_to_joint_state(self):
        # Get the robot out of a singularity.
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()  # To prevent residual movement

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        print(plan)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through. If executing interactively in a
        # Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the eef_step in Cartesian translation.
        # We will disable the jump threshold by setting it to 0.0, ignoring the check for infeasible jumps in joint space, which is sufficient for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the group.plan() method does this automatically.
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory. We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        self.move_group.execute(plan, wait=True)


def callback(grasp_poses, args):
    try:
        print(len(grasp_poses.poses), "poses received.")
        input("Press enter to plan trajectory for the first pose.")
        print("Planning...")
        planner = args[0]
        planner.go_to_joint_state()
        planner.go_to_pose_goal(grasp_poses.poses[0])
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("Interrupted, cleaning up...")
        return
   
    # cartesian_plan, fraction = planner.plan_cartesian_path()
    # planner.display_trajectory(cartesian_plan)
    # planner.execute_plan(cartesian_plan)
    # cartesian_plan, fraction = planner.plan_cartesian_path(scale=-1)
    # planner.execute_plan(cartesian_plan)


def main():
    try:
        planner = MotionPlanner()
        sub_pose = rospy.Subscriber("/auto_grasp/grasp_poses", PoseArray, callback, [planner])
        rospy.spin()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("Interrupted, cleaning up...")
        return


if __name__ == '__main__':
    main()
