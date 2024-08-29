#!/usr/bin/python3

import rospy

from heron_msgs.msg import Drive

class CmdDriveSmoother:

    def __init__(self):

        rospy.init_node('smoother')

        self.timeout = rospy.get_param('~timeout', 1.0)
        self.tau = rospy.get_param('~tau', 1.0)

        self.rate = rospy.get_param('~rate', 20)

        self.target_left = 0.0
        self.target_right = 0.0

        self.current_left = 0.0
        self.current_right = 0.0

        self.last_time = rospy.Time.now().to_sec() - 10.0
        
        self.cmd_drive = Drive()
        self.joy_sub = rospy.Subscriber('~input', Drive, self.drive_callback)
        self.cmd_drive_pub = rospy.Publisher('~output', Drive, queue_size=1)


    def drive_callback(self, drive_msg):

        self.target_right = drive_msg.right
        self.target_left = drive_msg.left
        self.last_time = rospy.Time.now().to_sec()

    def update(self):

        current_time = rospy.Time.now().to_sec()
        time_step = current_time - self.last_time

        delta_right = self.target_right - self.current_right
        delta_left = self.target_left - self.current_left

        self.current_right += (time_step / self.tau) * delta_right
        self.current_left += (time_step / self.tau) * delta_left

        if (current_time - self.last_time) > self.timeout:
            self.target_right = 0.0
            self.target_left = 0.0

        self.publish_cmd_drive()


    def publish_cmd_drive(self):

        self.cmd_drive.left = self.current_left
        self.cmd_drive.right = self.current_right
        self.cmd_drive_pub.publish(self.cmd_drive)

if __name__ == '__main__':

    smoother = CmdDriveSmoother()
    rate = rospy.Rate(smoother.rate)

    while not rospy.is_shutdown():
        smoother.update()
        smoother.publish_cmd_drive()
        rate.sleep()
