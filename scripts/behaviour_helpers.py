from enum import Enum

class BlockColor(Enum):
  RED = 1
  YELLOW = 2
  GREEN = 3
  ORANGE = 4
  BLUE = 5
  PURPLE = 6

class NamedTarget(Enum):
  """Named targets provided by the Kinova Gen3 Moveit config file
     for movegroup "arm"
  """ 
  HOME = "home"
  RETRACT = "retract"

home_joints = [-0.03171984478831291, -2.2357845306396484, 3.0441691875457764, -2.5050241947174072, 0.05331023037433624, -0.8105213642120361, 1.4876351356506348]
spok_top_grasp_pose = [1.4391455496142718, 1.8363940663544607, 0.29265280609614763, 0.7520303021466087, 0.6571603691234085, 0.03164780743351599, 0.039863394129426]

known_joint_positions = {
  "home" : [-0.03171984478831291, -2.2357845306396484, 3.0441691875457764, -2.5050241947174072, 0.05331023037433624, -0.8105213642120361, 1.4876351356506348],
  "home2" : [0.010713356547057629, -2.2281205654144287, 3.065701723098755, -2.556910991668701, 0.01152026280760765, -0.18579968810081482, 1.4895310401916504],
  "home3" : [0.010716111399233341, -1.7220138311386108, 3.0657029151916504, -1.6357226371765137, 0.01152026280760765, -0.9384607076644897, 1.4895310401916504],
  "home4" : [0.035481225699186325, -2.005342721939087, -3.128127336502075, -2.1735429763793945, -0.01113254576921463, -1.29158353805542, 1.4877082109451294],
  "home5" : [0.02397255226969719, -2.0024824142456055, -3.135423183441162, -2.1615514755249023, 0.002233522478491068, 0.2674437463283539, 1.5349910259246826],
  "top_grasp" : [-0.021463479846715927, -1.959319829940796, -3.120980739593506, -1.5620877742767334, -0.04217648133635521, -1.8811092376708984, 1.398183822631836]
}

known_poses = {
  "top_grasp_pose" : [1.4391455496142718, 1.8363940663544607, 0.29265280609614763, 0.7520303021466087, 0.6571603691234085, 0.03164780743351599, 0.039863394129426]
              }

def get_block_id(color, id_tag="00"):
  return "%s_block_%s"%(id_tag, color)

def get_block_name(index, block_colors):
  return get_block_id(block_colors[index])