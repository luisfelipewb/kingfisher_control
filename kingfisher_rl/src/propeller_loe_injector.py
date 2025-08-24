#!/usr/bin/python3

import rospy
from heron_msgs.msg import Drive

class LoeInjector:

    def __init__(self):

        rospy.init_node('loe_injector')


        self.loe_left = rospy.get_param("~loe_left", 1.0)  # Loss of effectiveness
        self.loe_right = rospy.get_param("~loe_right", 1.0)  # Loss of effectiveness

        self.output_msg = Drive()

        self.cmd_drive_sub_ = rospy.Subscriber("~cmd_drive_input", Drive, self.cmd_drive_cb, queue_size=1)
        self.cmd_drive_pub_ = rospy.Publisher("~cmd_drive_output", Drive, queue_size=1)


    def cmd_drive_cb(self, msg):
        # Apply simulated actuator loss-of-effectiveness (LOE)
        self.output_msg.left = msg.left * self.loe_left
        self.output_msg.right = msg.right * self.loe_right

        self.cmd_drive_pub_.publish(self.output_msg)


if __name__ == '__main__':

    try:
        rl_agent = LoeInjector()
        rospy.spin()
    finally:
        rospy.loginfo("Shutting down LOE injector...")
