#! /usr/bin/env python

from std_msgs.msg import String
import sys
import rospy
import roslaunch

switch_detectnet_topic = "/switch_detectnet"
kinova_detectnet_launch_name = "detectnet.ros1.launch"
spot_detectnet_launch_name = "teddy_detectnet_v2.ros1.launch"

class DetectnetSwitcher():
    def __init__(self):
        self.node_to_use = None
        self.prev_node = None
        rospy.init_node('detectnet_switcher')
        self.switch_detectnet_sub = rospy.Subscriber(switch_detectnet_topic, String, self.switch_detectnet_callback, queue_size=1)

    def switch_detectnet_callback(self, switch):
        self.prev_node = self.node_to_use
        self.node_to_use = switch.data
        assert self.node_to_use == 'kinova' or self.node_to_use == 'spot'
        print(self.node_to_use)
        print(self.prev_node)

    def switch(self):
        if not self.node_to_use or self.node_to_use == self.prev_node:
            return
        self.prev_node = self.node_to_use
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(
            uuid, ["/home/arfi/catkin_ws/src/ros_deep_learning/launch/" + \
                (kinova_detectnet_launch_name if self.node_to_use == 'kinova' else spot_detectnet_launch_name)])
        launch.start()
        rospy.loginfo("detectnet node started")


def main(args):
    switcher = DetectnetSwitcher()
    while not rospy.is_shutdown():
        switcher.switch()

if __name__ == "__main__":
    main(sys.argv)

