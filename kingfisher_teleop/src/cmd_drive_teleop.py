#!/usr/bin/python3

import rospy

from kingfisher_msgs.msg import Drive
from sensor_msgs.msg import Joy

from topic_tools.srv import MuxSelect

class CmdDriveTeleop:

    def __init__(self):

        rospy.init_node('cmd_drive_teleop')


        self.acc_button = rospy.get_param('~acc_button', 7)
        self.turn_button = rospy.get_param('~turn_button', 6)
        self.drive_button = rospy.get_param('~drive_button', 4)
        self.direct_drive_button = rospy.get_param('~direct_drive_button', 5)
        self.mux_button = rospy.get_param('~mux_button', 7)

        self.left_axis_v = rospy.get_param('~left_axis_v', 1)
        self.right_axis_h = rospy.get_param('~right_axis_h', 3)
        self.right_axis_v = rospy.get_param('~right_axis_v', 4)
        self.turbo_axis = rospy.get_param('~turbo_axis', 5)

        self.forward_min = rospy.get_param('~forward_min', 0.0)
        self.forward_max = rospy.get_param('~forward_max', 0.6)
        self.forward_max = rospy.get_param('~forward_max', 1.0)
        self.reverse_min = rospy.get_param('~reverse_min', 0.0)
        self.reverse_max = rospy.get_param('~reverse_max', -1.0)
        self.forward_max_turbo = rospy.get_param('~forward_max_turbo', 1.0)

        self.current_fw_max = self.forward_max
        self.start_prev = None

        self.cmd_drive = Drive()
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_drive_pub = rospy.Publisher('~cmd_drive', Drive, queue_size=1)

        self.drive_mux_service = rospy.ServiceProxy('/drive_mux/select', MuxSelect)

    def scale_max(self, value):

        scale = (-value + 1.0)/2        
        self.current_fw_max = self.forward_max + scale*(self.forward_max_turbo - self.forward_max)

    def map_range(self, value):

        return_value = 0.0
        if value > 0.01:
            return_value = self.forward_min + value * (self.current_fw_max - self.forward_min)
        elif value < -0.01:
            return_value = self.reverse_min + value * (self.reverse_min - self.reverse_max)
        return return_value

    def call_mux(self, start):
        if start == 1 and self.start_prev == 0:
            try:
                self.drive_mux_service('teleop/cmd_drive')
                rospy.loginfo("Select teleop input")
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)

        self.start_prev = start

    def joy_callback(self, joy_msg):

        self.call_mux(joy_msg.buttons[self.mux_button])

        left, right = 0.0, 0.0
        
        self.scale_max(joy_msg.axes[self.turbo_axis])
        # Intuitive joystic controls
        if joy_msg.buttons[self.drive_button] == 1:

            # Forward 
            if joy_msg.axes[self.acc_button] > 0.1:
                left = self.current_fw_max
                right = self.current_fw_max
                # throttle for fine heading control
                steer = joy_msg.axes[self.right_axis_h]
                if steer < -0.1: # turn right
                    right = right * (1 - abs(steer))
                elif steer > 0.1: # turn left
                    left = left * (1 - abs(steer))
            # Reverse
            elif joy_msg.axes[self.acc_button] < -0.1:
                left = self.reverse_max
                right = self.reverse_max
                # throttle for fine heading control
                steer = joy_msg.axes[self.right_axis_h]
                if steer < -0.1: # turn right
                    right = right * (1 - abs(steer))
                elif steer > 0.1: # turn left
                    left = left * (1 - abs(steer))
            # Turn Left
            elif joy_msg.axes[self.turn_button] > 0.1:
                left = self.reverse_max
                right = 0.5
            # Turn Right
            elif joy_msg.axes[self.turn_button] < -0.1:
                left = 0.5
                right = self.reverse_max
            else:
                acc = joy_msg.axes[self.left_axis_v]
                left = self.map_range(acc)
                right = self.map_range(acc)
                steer = joy_msg.axes[self.right_axis_h]
                if steer < -0.1: # turn right
                    right = right * (1 - abs(steer))
                elif steer > 0.1: # turn left
                    left = left * (1 - abs(steer))

        
        # Joystick direct control
        elif joy_msg.buttons[self.direct_drive_button] == 1:
            left = joy_msg.axes[self.left_axis_v]
            right = joy_msg.axes[self.right_axis_v]

            left = self.map_range(left)
            right = self.map_range(right)

        self.publish_cmd_drive(left, right)
    
    def publish_cmd_drive(self, left, right):
        self.cmd_drive.left = left
        self.cmd_drive.right = right
        self.cmd_drive_pub.publish(self.cmd_drive)


if __name__ == '__main__':

    cmd_drive_teleop = CmdDriveTeleop()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rate.sleep()
