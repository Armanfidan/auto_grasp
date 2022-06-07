
class Object():
  """Definition of an object in the world
  """
  def __init__(self, name, pose):
    """Initialise object

    Args:
      name (string): Name of the object
      pose: Pose message, PoseStamped message, list of 6 floats [x, y, z, rot_x, rot_y, rot_z]
            or a list of 7 floats [x, y, z, qx, qy, qz, qw] representing the pose of the object
            relative to "base_link".
    """
    self.name = name
    self.pose = pose

  def update(self, pose):
    """Update the pose of this object
    """
    self.pose = pose

class World():
  """Contains objects that the robot will interact with and avoid 
  """
  def __init__(self):
    self.tracked_objects_map = dict()

  def get_pose(self, name):
    """ Get pose for an object
    Args:
      name: name of the object

    Returns:
      pose: Pose message, PoseStamped message, list of 6 floats [x, y, z, rot_x, rot_y, rot_z]
            or a list of 7 floats [x, y, z, qx, qy, qz, qw] representing the pose of the object
            relative to "base_link".
    """
    return self.tracked_objects_map[name].pose

  def register_object(self, name, pose):
    """Register a new object in this world
    """
    obj = Object(name, pose)
    self.tracked_objects_map[name] = obj

  def get_object_from_name(self, name):
    """ Given the name of the object, returns the object
    """
    return self.tracked_objects_map[name]
