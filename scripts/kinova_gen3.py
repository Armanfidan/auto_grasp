import sys
import moveit_commander
from behaviour_helpers import NamedTarget

import rospy

DEBUG = False

class KinovaGen3(object):

  def __init__(self,
               namespace, 
               is_gripper_present,
               gripper_joint_name,
               degrees_of_freedom=7,
               max_velocity_scaling_factor=0.45
  ):

    super(KinovaGen3, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    
    self.namespace = namespace
    self.is_gripper_present = is_gripper_present
    self.gripper_joint_name = gripper_joint_name
    self.degrees_of_freedom = degrees_of_freedom
    self.max_velocity_scaling_factor = max_velocity_scaling_factor

    self.robot = moveit_commander.RobotCommander()

    self.arm_group = moveit_commander.MoveGroupCommander("arm", ns=self.namespace)
    self.arm_group.set_max_velocity_scaling_factor(self.max_velocity_scaling_factor)
    self.arm_group.set_planning_time(45.0)

    if self.is_gripper_present:
      self.hand_group = moveit_commander.MoveGroupCommander("gripper", ns=self.namespace)
      self.hand_group.set_max_velocity_scaling_factor(self.max_velocity_scaling_factor)

  def go_to_named_target(self, target):
    """Move the arm to the given named position.
       target: target named position. Must be of type behaviour_helpers.NamedTarget
    """
    if isinstance(target, NamedTarget):
      self.arm_group.set_named_target(target.value)
      plan = self.arm_group.plan()
      return self.arm_group.execute(plan, wait=True)
    else:
      sys.exit()

  def go_to_target(self, target_pose):
    """Move the arm to a target position (relative to "base_link")
    """
    success = True    
    self.arm_group.set_pose_target(target_pose)
    success &= self.arm_group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    self.arm_group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm_group.clear_pose_targets()
    return success
  
  def reach_cartesian_pose(self, pose, tolerance, constraints=None):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)
  
  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_gripper_position(self, relative_position):
    """Moves the gripper to a relative position between 0 and 1
       1: fully closed
       0: fully open
    """
    if self.is_gripper_present:
      gripper_group = self.hand_group
    
      # We only have to move this joint because all others are mimic!
      gripper_joint = self.robot.get_joint(self.gripper_joint_name)
      gripper_max_absolute_pos = gripper_joint.max_bound()
      gripper_min_absolute_pos = gripper_joint.min_bound()

      try:
        val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
        return val
      except:
        return False
    
    else:
      sys.exit()
  
  def reach_joint_angles(self, joint_position_array, tolerance):
    arm_group = self.arm_group
    success = True

    if DEBUG:
      # Get the current joint positions
      joint_positions = arm_group.get_current_joint_values()
      rospy.loginfo("Printing current joint positions before movement :")
      for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    arm_group.set_joint_value_target(joint_position_array)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    if DEBUG:
      # Show joint positions after movement
      new_joint_positions = arm_group.get_current_joint_values()
      rospy.loginfo("Printing current joint positions after movement :")
      for p in new_joint_positions: rospy.loginfo(p)
    return success
