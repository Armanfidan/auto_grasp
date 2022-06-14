#! /usr/bin/env python3

from std_msgs.msg import String
import rospy
import roslaunch

switch_detectnet_topic = "/switch_detectnet"
kinova_detectnet_launch_name = "detectnet.ros1.launch"
spot_detectnet_launch_name = "teddy_detectnet.ros1.launch"

def switch_detectnet_callback(switch):
    node_to_use = switch.data
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    assert node_to_use == 'kinova' or node_to_use == 'spot'

    launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/arfi/catkin_ws/src/ros_deep_learning/launch/" + \
            kinova_detectnet_launch_name if node_to_use == 'kinova' else spot_detectnet_launch_name])
    launch.start()
    rospy.loginfo("detectnet node started")


def main(args):
    rospy.init_node('detectnet_switcher')
    switch_detectnet_sub = rospy.Subscriber(switch_detectnet_sub, switch_detectnet_callback, queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down DetectNet switcher module")

if __name__ == "__main__":
    main(sys.argv)

