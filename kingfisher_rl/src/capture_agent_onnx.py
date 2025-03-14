#!/usr/bin/python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Vector3Stamped
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from heron_msgs.msg import Drive
import tf2_geometry_msgs
import tf2_ros
import rospkg

import torch
import onnxruntime as ort
import numpy as np
import math
import threading

from visualization_msgs.msg import Marker

# print path of the rl_games python library
import rl_games
import time
print(rl_games.__file__)

class RLAgent:

    def __init__(self):
        
        rospy.init_node('rl_agent')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # rl_config = rospy.get_param("~config_file", "test.yaml")
        # rl_policy = rospy.get_param("~policy_file", "test.pth")
        model_path = rospy.get_param("~onnx_path", "test")
        # Get the file name of the model path  (without the path and extension)
        self.model_name = (f"{model_path}").replace(".", "_")

        self.dist_threshold = rospy.get_param("~dist_threshold", 0.3)
        self.control_freq = rospy.get_param("~control_freq", 20.0)

        model_folder = rospkg.RosPack().get_path('kingfisher_rl') + "/models/"
        print(f"Config folder: {model_folder}")
        # self.rl_config_path = config_folder+rl_config
        # self.rl_policy_path = config_folder+rl_policy
        self.rl_onnx_path = model_folder+model_path
        self.device = 'cpu'

        num_obs = 8
        obs_buffer_len = rospy.get_param("~obs_buffer_len", 1)
        size_buffer = obs_buffer_len * num_obs

        # with open(self.rl_config_path, 'r') as stream:
        #     self.cfg = yaml.safe_load(stream)
        
        # observation_space = spaces.Dict({"state":spaces.Box(np.ones(size_buffer) * -np.Inf, np.ones(size_buffer) * np.Inf)})
        # action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float64)
        
        # self.player = BasicPpoPlayerContinuous(self.cfg, observation_space, action_space, clip_actions=True, deterministic=True, device=self.device)
        # self.player.restore(self.rl_policy_path)

        print(f"Loading model from {model_path}")
        # self.onnx_model = onnx.load(self.rl_onnx_path)
        self.ort_model = ort.InferenceSession(self.rl_onnx_path, providers=['CPUExecutionProvider'])

        self.goal_robot = None
        self.goal_world = None
        self.odom = None
        self.world_frame = None
        self.odom_timeout = rospy.Duration(1.0)
        goal_timeout_value = rospy.get_param("~goal_timeout", 5.0)
        self.goal_timeout = rospy.Duration(goal_timeout_value)

        self.lin_vel = [0.0, 0.0]
        self.ang_vel = [0.0]
        self.heading = [0.0, 0.0]
        self.robot_position = [0.0, 0.0]
        self.previous_actions = [0.0, 0.0]

        #buffers for observations
        self._obs_buffer = torch.zeros((1, obs_buffer_len, num_obs), dtype=torch.float32)

        # TODO: review topics that are hardcoded
        rospy.Subscriber("~goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1)
        self.cmd_drive_pub_ = rospy.Publisher("~cmd_drive", Drive, queue_size=1)
        self.rl_status_pub_ = rospy.Publisher("~rl_status", Bool, queue_size=1)
        self.marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size = 1)
        self.rl_model_name = rospy.Publisher('~rl_model_name', String, queue_size=1)


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

    def odom_cb(self, msg):
        """
        Assumption: The odometry message is in the base_link frame.
        """
        with self.odom_lock:
            self.odom = msg
        self.world_frame = msg.header.frame_id

    def goal_cb(self, msg):
        # store the goal in the world frame
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

        return

    def get_observations(self):

        # convert to robot position from goal perspective with ofset
        with self.odom_lock:
            robot_pos = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
            robot_vel = [self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y]
            robot_ang_vel = [self.odom.twist.twist.angular.z]
        goal_pos = [self.goal_robot.pose.position.x, self.goal_robot.pose.position.y]
        goal_bearing = math.atan2(goal_pos[1], goal_pos[0])
        goal_distance = min(np.linalg.norm(np.array(goal_pos)), 9.0) # Limit the distance to 9m
        goal_cos_sin = [math.cos(goal_bearing), math.sin(goal_bearing)]

        # rospy.loginfo_throttle(1, f"OBS Position  \t{goal_pos[0]:.2f}, {goal_pos[1]:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Velocity  \t{robot_vel[0]:.2f}, {robot_vel[1]:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Distance: \t{goal_distance:.2f}")
        # rospy.loginfo_throttle(1, f"OBS Heading:  \t{goal_bearing*180/math.pi:.2f}")

        # Shift the buffer
        self._obs_buffer[0,:-1, :] = self._obs_buffer[0,1:, :].clone()

        # Define the values for each field
        # TODO: Removeh ardcoded values for the observations for testing
        actions = np.array([[0.0, 0.0]], dtype=np.float32)  # 2
        root_lin_vel_b = np.array([robot_vel], dtype=np.float32)  # 2
        root_ang_vel_b = np.array([robot_ang_vel], dtype=np.float32)  # 1
        goal_cos_sin = np.array([goal_cos_sin], dtype=np.float32)  # 2
        distance = np.array([[goal_distance]], dtype=np.float32)  # 1
        observations = np.concatenate(
            [
                actions,  # 2
                root_lin_vel_b,  # 2
                root_ang_vel_b,  # 1
                goal_cos_sin,  # 1
                distance,  # 1
            ],
            axis=1,
        )
        obs = dict({"obs": observations})


        return obs
                
    def get_action(self, obs):
     
        output = self.ort_model.run(None, obs)
        action = np.clip(output[0].squeeze(0), -1, 1)

        tl = float(action[0])
        tr = float(action[1])

        return tl, tr

    def publish_cmds(self, thruster_left, thruster_right):
        msg = Drive()

        msg.left = 0.0
        msg.right = 0.0
        if isinstance(thruster_left, float):     
            msg.left = thruster_left
        if isinstance(thruster_right, float):     
            msg.right = thruster_right

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
        self.rl_model_name.publish(String(self.model_name))


if __name__ == '__main__':

    rl_agent = RLAgent()
    rate = rospy.Rate(rl_agent.control_freq)

    while not rospy.is_shutdown():
        rl_agent.control_loop()
        rate.sleep()


