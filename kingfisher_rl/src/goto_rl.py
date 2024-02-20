#!/usr/bin/python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
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
        
        self.dist_threshold = 1.0

        config_folder = rospkg.RosPack().get_path('kingfisher_rl') + "/config/"
    
        rl_config = rospy.get_param("~config", "test.yaml")
        rl_policy = rospy.get_param("~policy", "test.pth")
        
        self.rl_config_path = config_folder+rl_config
        self.rl_policy_path=config_folder+rl_policy
        self.device = 'cuda'

        num_obs = 10 
        max_actions = 2

        with open(self.rl_config_path, 'r') as stream:
            self.cfg = yaml.safe_load(stream)
        
        observation_space = spaces.Dict({"state":spaces.Box(np.ones(num_obs) * -np.Inf, np.ones(num_obs) * np.Inf),
                                              "transforms":spaces.Box(low=-1, high=1, shape=(max_actions, 5)),
                                              "masks":spaces.Box(low=0, high=1, shape=(max_actions,))})
        
        action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        
        self.player = BasicPpoPlayerContinuous(self.cfg, observation_space, action_space, clip_actions=True, deterministic=True)
        self.player.restore(self.rl_policy_path)

        self.goal_robot = None
        self.goal_world = None
        self.odom = None
        self.age_thresh = rospy.Duration(1.0)

        self.lin_vel = [0.0, 0.0]
        self.ang_vel = [0.0]
        self.heading = [0.0, 0.0]
        self.robot_position = [0.0, 0.0]

        #buffers for observations
        self._obs_buffer = torch.zeros((1, num_obs), device=self.device, dtype=torch.float32)
        self._task_label = torch.ones((1), device=self.device, dtype=torch.float32)
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
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5
        self.marker.header.frame_id = "base_link"

    def odom_cb(self, msg):
        """
        Callback function for handling new odometry messages. The robot's current position and heading are stored in
        class variables after transforming to the base_link frame.

        :param msg: The incoming odometry message.
        """
        incoming_frame = msg.child_frame_id
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        twist_stamped = TwistStamped()
        twist_stamped.header = msg.header
        twist_stamped.twist = msg.twist.twist

        try:
            transform = self.tf_buffer.lookup_transform('base_link', incoming_frame, rospy.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            # TODO: Transform twist as well Necessary in case of the SBG IMU
            # twist_transformed = tf2_geometry_msgs.do_transform_twist(twist_stamped, transform)
            twist_transformed = twist_stamped
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
            self.odom.twist.twist = twist_transformed.twist
        # print(f"\nOdometry: {self.odom}")

        # self.heading = [math.cos(msg.pose.pose.orientation.z), math.sin(msg.pose.pose.orientation.z)]
        # self.lin_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        # self.ang_vel = [msg.twist.twist.angular.z]
        # self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def goal_cb(self, msg):
        # store the goal in the world frame
        with self.goal_lock:
            self.goal_world = msg

        return

    def get_observations(self):

        # convert to robot position from goal perspective with ofset
        goal_pos = [self.goal_robot.pose.position.x, self.goal_robot.pose.position.y]
        with self.odom_lock:
            robot_pos = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        # print("goal_pos: ",goal_pos)
        # print("robot_pos: ",robot_pos)
        # print("heading", self.heading)
        # print("ROBOT Position from Frame goal %.2f, %.2f" %(robot_pos[0],robot_pos[1]))
        rospy.loginfo_throttle(1, "OBS Position %.2f, %.2f" %(goal_pos[0],goal_pos[1]))
        goal_distance = np.linalg.norm(np.array(goal_pos))
        rospy.loginfo_throttle(1, "OBS Distance: %.2f" % goal_distance)
        goal_bearing = math.atan2(goal_pos[1], goal_pos[0])
        rospy.loginfo_throttle(1, f"OBS Heading: {goal_bearing}")


        current_state = {"position":robot_pos, "orientation": self.heading, "linear_velocity": self.lin_vel, "angular_velocity":self.ang_vel}
        
        self._obs_buffer[:, 0:2] = torch.tensor(current_state["orientation"], device=self.device)
        self._obs_buffer[:, 2:4] = torch.tensor(current_state["linear_velocity"], device=self.device)
        self._obs_buffer[:, 4] = torch.tensor(current_state["angular_velocity"], device=self.device)
        self._obs_buffer[:, 5] = self._task_label
        self._obs_buffer[:, 6:8] = self._target_positions - torch.tensor(current_state["position"], device=self.device)
        self._obs_buffer[:, 8:] = torch.tensor([0,0], device=self.device)


        obs = dict({"state":self._obs_buffer,
                    "transforms":torch.zeros((1, 2, 5), device=self.device, dtype=torch.float32),
                    "masks":torch.zeros((1, 2), device=self.device, dtype=torch.long)})

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
        if rospy.Time.now() - self.odom.header.stamp > self.age_thresh:
            rospy.logwarn_throttle(1, "Odometry is too old...")
            return False
        
        # TODO: Check if autonomous mode is enabled
        
        # Check if goal has been reached
        if self.goal_reached():
            rospy.loginfo("Goal reached")
            with self.goal_lock:
                self.goal_world = None
            return False

        return True

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

        # Update goal_robot position
        obs = self.get_observations()

        # Get actions from the policy
        tl, tr = self.get_action(obs)

        # Publish the actions
        self.publish_cmds(tl, tr)
        self.rl_status_pub_.publish(Bool(True))


if __name__ == '__main__':

    goto_rl = GoTo()
    rate = rospy.Rate(10) # 10 Hz control frequency. TODO: use rosparam

    while not rospy.is_shutdown():
        goto_rl.control_loop()
        rate.sleep()


