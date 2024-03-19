#!/usr/bin/python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Vector3Stamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from heron_msgs.msg import Drive
import tf2_geometry_msgs
import tf2_ros
import rospkg

import torch
from gym import spaces
from rl_games.algos_torch.players import BasicPpoPlayerContinuous, BasicPpoPlayerDiscrete
import numpy as np
import math
import yaml
import threading

from visualization_msgs.msg import Marker

# print path of the rl_games python library
import rl_games
print(rl_games.__file__)

class GoTo:

    def __init__(self):
        
        rospy.init_node('goto_rl')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.dist_threshold = 0.3

        config_folder = rospkg.RosPack().get_path('kingfisher_rl') + "/config/"
    
        rl_config = rospy.get_param("~config", "test.yaml")
        rl_policy = rospy.get_param("~policy", "test.pth")
        
        self.rl_config_path = config_folder+rl_config
        self.rl_policy_path=rl_policy
        self.device = 'cuda'

        num_obs = 11
        max_actions = 2

        with open(self.rl_config_path, 'r') as stream:
            self.cfg = yaml.safe_load(stream)
        
        observation_space = spaces.Dict({"state":spaces.Box(np.ones(num_obs) * -np.Inf, np.ones(num_obs) * np.Inf)})
        
        action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        
        self.player = BasicPpoPlayerContinuous(self.cfg, observation_space, action_space, clip_actions=True, deterministic=True)
        self.player.restore(self.rl_policy_path)

        self.goal_robot = None
        self.goal_world = None
        self.odom = None
        self.odom_timeout = rospy.Duration(1.0)
        self.goal_timeout = rospy.Duration(60.0)

        self.lin_vel = [0.0, 0.0]
        self.ang_vel = [0.0]
        self.heading = [0.0, 0.0]
        self.robot_position = [0.0, 0.0]
        self.previous_actions = [0.0, 0.0]

        #buffers for observations
        self._obs_buffer = torch.zeros((1, num_obs), device=self.device, dtype=torch.float32)
        self._target_positions =torch.zeros((1,2), device=self.device, dtype=torch.float32) #because the policy was trained to spawn randomly and go to (0.0)

        rospy.Subscriber("~goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        self.cmd_drive_pub_ = rospy.Publisher("~cmd_drive", Drive, queue_size=1)
        self.rl_status_pub_ = rospy.Publisher("~rl_status", Bool, queue_size=1)
        self.marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size = 1)

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

    def transform_twist(self, twist, transform):
        """
        Transforms a geometry_msgs/Twist message from one frame to another.

        :param twist: The Twist message to transform.
        :param transform: The transform to apply.
        :return: The transformed Twist message.
        """
        # Create a Vector3Stamped message for the linear and angular components of the twist message
        twist_linear = Vector3Stamped()
        twist_linear.vector = twist.linear
        twist_angular = Vector3Stamped()
        twist_angular.vector = twist.angular

        # Transform the linear and angular components of the twist message
        twist_linear_transformed = tf2_geometry_msgs.do_transform_vector3(twist_linear, transform)
        twist_angular_transformed = tf2_geometry_msgs.do_transform_vector3(twist_angular, transform)

        # Create a new Twist message and assign the transformed components to it
        twist_transformed = Twist()
        twist_transformed.linear = twist_linear_transformed.vector
        twist_transformed.angular = twist_angular_transformed.vector

        return twist_transformed

    def odom_cb(self, msg):
        """
        Callback function for handling new odometry messages. The robot's current position and heading are stored in
        class variables after transforming to the base_link frame.

        :param msg: The incoming odometry message.
        """
        child_frame_id = msg.child_frame_id
        frame_id = msg.header.frame_id
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        twist_stamped = TwistStamped()
        twist_stamped.header = msg.header
        twist_stamped.twist = msg.twist.twist
        # print(f"twist: \n{msg.twist.twist.linear}")

        try:
            # We want want the pose in the base_link frame. For SBG the odom is not in base_link
            transform = self.tf_buffer.lookup_transform('base_link', child_frame_id, rospy.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            # We want the twist in the base_link frame and odometry published in the world frame
            transform = self.tf_buffer.lookup_transform('base_link', frame_id, rospy.Time())
            twist_transformed = self.transform_twist(twist_stamped.twist, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to find transform from [{incoming_frame}] to [base_link] frame")
            return

        # print(f"\nOriginal: {msg.pose.pose}")
        with self.odom_lock:
            self.odom = msg
            self.odom = Odometry()
            self.odom.header = msg.header
            self.odom.child_frame_id = "base_link"
            self.odom.pose.pose = pose_transformed.pose
            self.odom.twist.twist = twist_transformed

    def goal_cb(self, msg):
        # store the goal in the world frame
        with self.goal_lock:
            self.goal_world = msg

        return

    def get_observations(self):

        # convert to robot position from goal perspective with ofset
        with self.odom_lock:
            robot_pos = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
            robot_vel = [self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y]
            robot_ang_vel = [self.odom.twist.twist.angular.z]
        goal_pos = [self.goal_robot.pose.position.x, self.goal_robot.pose.position.y]
        goal_bearing = math.atan2(goal_pos[1], goal_pos[0])
        goal_distance = np.linalg.norm(np.array(goal_pos))
        goal_cos_sin = [math.cos(goal_bearing), math.sin(goal_bearing)]
        previous_actions = self.previous_actions

        # rospy.loginfo_throttle(1, f"OBS Position  \t{goal_pos[0]:.2f}, {goal_pos[1]:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Velocity  \t{robot_vel[0]:.2f}, {robot_vel[1]:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Distance: \t{goal_distance:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Heading:  \t{goal_bearing*180/math.pi:.2f}")

        self._obs_buffer[:, 0:2] = torch.tensor(robot_vel, device=self.device)
        self._obs_buffer[:, 2] = torch.tensor(robot_ang_vel, device=self.device)
        self._obs_buffer[:, 3:5] = torch.tensor(goal_cos_sin, device=self.device)
        self._obs_buffer[:, 5] = torch.tensor(goal_distance, device=self.device)
        self._obs_buffer[:, 6:] = torch.tensor([0,0,0,0,0], device=self.device)
        # self._obs_buffer[:, 6:8] = torch.tensor(previous_actions, device=self.device)
        # self._obs_buffer[:, 8:] = torch.tensor([0,0,0], device=self.device)

        obs = dict({"state":self._obs_buffer})

        return obs
                
    def get_action(self, obs):

        action = self.player.get_action(obs, is_deterministic=True)

        tl = action[0].item()
        tr = action[1].item()

        return tl, tr

    def publish_cmds(self, thruster_left, thruster_right):
        msg = Drive()

        msg.left = 0.0
        msg.right = 0.0

        if isinstance(thruster_left, float):     
            msg.left = thruster_left
        if isinstance(thruster_right, float):     
            msg.right = thruster_right

        # print("cmd_drive sent to robot: l %.2f, r %.2f" %(msg.left, msg.right))
        self.cmd_drive_pub_.publish(msg)


    def check_pre_conditions(self):
        # Don't move if goal is not available
        if not self.goal_world:
            rospy.loginfo_throttle(5, "Waiting for goal...")
            return False
        
        # Don't move if odom is not available
        if self.odom is None:
            rospy.logwarn_throttle(1,"Waiting for odometry...")
            return False
        
        # Don't move if the odom is too old
        if rospy.Time.now() - self.odom.header.stamp > self.odom_timeout:
            rospy.logwarn_throttle(1, "Odometry is too old...")
            return False
        
        # TODO: Check if autonomous mode is enabled
        
        # Check if goal has been reached
        if self.goal_reached():
            rospy.loginfo("Goal reached")
            with self.goal_lock:
                self.goal_world = None
            return False

        if self.give_up():
            rospy.loginfo("Goal not reached. Giving up...")
            with self.goal_lock:
                self.goal_world = None
            return False

        return True

    def give_up(self):
        # Give up if the goal is too old
        if rospy.Time.now() - self.goal_world.header.stamp > self.goal_timeout:
            rospy.logwarn_throttle(1, "Goal is too old...")
            return True
        return False

    def goal_reached(self):
            """
            Check if the goal has been reached.

            Returns:
                bool: True if the goal has been reached, False otherwise.
            """
            try:
                with self.goal_lock:
                    transform = self.tf_buffer.lookup_transform('base_link', self.goal_world.header.frame_id, rospy.Time())
                    msg_transformed = tf2_geometry_msgs.do_transform_pose(self.goal_world, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn(f"Failed to find transform goal from [{self.goal_world.header.frame_id}] to [base_link] frame")
                return

            self.goal_robot = msg_transformed
            # Publish marker to debug goal position
            self.marker.header.stamp = rospy.Time.now()
            self.marker.pose = self.goal_robot.pose
            self.marker_pub.publish(self.marker)

            # Check if goal has been reached (already in the robot frame)
            dist = np.linalg.norm(np.array([self.goal_robot.pose.position.x, self.goal_robot.pose.position.y]))
            rospy.loginfo_throttle(1, "Distance to goal: %.2f" % dist)
            if dist < self.dist_threshold:
                return True
                
            return False

    def control_loop(self):

        if self.check_pre_conditions() == False:
            self.publish_cmds(0.0, 0.0)
            self.rl_status_pub_.publish(Bool(False))
            return
        
        rospy.loginfo_throttle(5,"RL is moving the robot")

        # Update goal position in robot frame
        obs = self.get_observations()

        # Get actions from the policy
        tl, tr = self.get_action(obs)
        self.previous_actions = [tl, tr]

        # Publish the actions
        self.publish_cmds(tl, tr)
        self.rl_status_pub_.publish(Bool(True))


if __name__ == '__main__':

    goto_rl = GoTo()
    rate = rospy.Rate(10) # 10 Hz control frequency. TODO: use rosparam

    while not rospy.is_shutdown():
        goto_rl.control_loop()
        rate.sleep()


