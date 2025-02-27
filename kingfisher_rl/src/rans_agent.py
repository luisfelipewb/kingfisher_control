#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from heron_msgs.msg import Drive
import tf2_geometry_msgs
import tf2_ros

import threading

from visualization_msgs.msg import Marker


class RLAgent:

    def __init__(self):
        
        rospy.init_node('rl_agent')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_robot = None
        self.goal_world = None
        self.odom = None
        self.world_frame = None

        # TODO: review topics that are hardcoded
        rospy.Subscriber("~goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/action", Float32MultiArray, self.action_cb, queue_size=1)
        self.cmd_drive_pub_ = rospy.Publisher("~cmd_drive", Drive, queue_size=1)
        self.rl_status_pub_ = rospy.Publisher("~rl_status", Bool, queue_size=1)
        self.marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size = 1)
        self.goal_republisher = rospy.Publisher("/goal", PointStamped, queue_size=1)


        self.odom_lock = threading.Lock()
        self.goal_lock = threading.Lock()

        self.marker = Marker()
        self.marker.type = 2
        self.marker.id = 0
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.scale.z = 0.3
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.8
        self.marker.header.frame_id = "base_link"

    def action_cb(self, msg):
        # Convert the Float32MultiArray to cmd_drive and publish it
        if len(msg.data) == 2:
            self.publish_cmds(msg.data[0], msg.data[1])
        else:
            rospy.logwarn("Invalid action received. Expected 2 values")

    def odom_cb(self, msg):
        """
        Callback function for odometry messages.

        This function is called whenever a new odometry message is received. It assumes that the odometry message is in the `base_link` frame. The function updates the internal odometry state and the world frame ID.

        Args:
            msg (nav_msgs.msg.Odometry): The odometry message containing the current state of the robot.

        Attributes:
            odom (nav_msgs.msg.Odometry): The latest odometry message received.
            world_frame (str): The frame ID of the world coordinate frame.
        """
        with self.odom_lock:
            self.odom = msg
        self.world_frame = msg.header.frame_id

    def goal_cb(self, msg):
        """
        Callback function for handling goal messages.

        This function is triggered when a new goal message is received. It transforms
        the goal from its original frame to the world frame (the frame_id of the odometry message)
        and republishes it as a PointStamped message. Should onlyb be called after the an odometry
        message has been received.

        Args:
            msg (PoseStamped): The incoming goal message in the form of a PoseStamped.

        Raises:
            tf2_ros.LookupException: If the transform lookup fails.
            tf2_ros.ConnectivityException: If there is a connectivity issue during the transform lookup.
            tf2_ros.ExtrapolationException: If the transform lookup fails due to extrapolation issues.

        Note:
            The function uses a lock (`self.goal_lock`) to ensure thread safety while accessing
            and modifying the goal. The goal's timestamp is updated to the current time to account
            for potential time synchronization issues during field tests.
        """
        # Republish the goal as a PointStamped message in the world frame.
        # The world frame is the frame_id of the odometry message.
        with self.goal_lock:
            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, rospy.Time())
                goal_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn(f"Failed to find transform goal from [{self.goal_world.header.frame_id}] to [base_link] frame")
                return

            self.goal_world = goal_transformed
            # During field tests, time might not be synchronized with the computer running rviz.
            self.goal_world.header.stamp = rospy.Time.now()

        # Republish the pose goal in the PointStamped format
        goal_point = PointStamped()
        goal_point.header = self.goal_world.header
        goal_point.point = self.goal_world.pose.position
        self.goal_republisher.publish(goal_point)

        return


    def publish_cmds(self, thruster_left, thruster_right):
        """
        Publish commands to the thrusters.
        This method creates a Drive message and sets the left and right thruster
        values. It then publishes the message to the cmd_drive_pub_ topic.
        Parameters:
        thruster_left (float): The command value for the left thruster.
        thruster_right (float): The command value for the right thruster.
        Returns:
        None
        """

        msg = Drive()

        msg.left = 0.0
        msg.right = 0.0
        if isinstance(thruster_left, float):     
            msg.left = thruster_left
        if isinstance(thruster_right, float):     
            msg.right = thruster_right

        self.cmd_drive_pub_.publish(msg)


    def check_pre_conditions(self):
        """ Need odom to run"""

        while self.odom is None and not rospy.is_shutdown():
            rospy.logwarn_throttle(2, "Waiting for odometry...")
            rospy.sleep(0.1)

    def shutdown(self):
        rospy.loginfo("Shutting down node...")
        self.cmd_drive_pub_.unregister()
        self.rl_status_pub_.unregister()
        self.marker_pub.unregister()
        self.goal_republisher.unregister()

    def control_loop(self):
        rospy.loginfo_throttle(10,"RANS control loop running...")

if __name__ == '__main__':

    rl_agent = RLAgent()
    rospy.on_shutdown(rl_agent.shutdown)

    rl_agent.check_pre_conditions()

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            rl_agent.control_loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("... node killed")

