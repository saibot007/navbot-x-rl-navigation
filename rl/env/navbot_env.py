import math
import time
import tempfile
import subprocess

import gymnasium as gym
import numpy as np
import rclpy
from gymnasium import spaces
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class NavBotEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = Node("navbot_rl_env")

        # World / robot settings
        self.world_name = "empty_world"
        self.robot_name = "navbot_x"
        self.xacro_path = "/home/atharva-sharma/NewStorage/projects/navbot-x/ros2_ws/src/navbot_x_description/urdf/navbot_x.urdf.xacro"

        # Spawn / reset settings
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_z = 0.2
        self.spawn_yaw = 0.0

        # Goal settings
        self.goal_x = 2.5
        self.goal_y = 0.0
        self.goal_tolerance = 0.30

        # Episode settings
        self.max_steps = 300
        self.current_step = 0

        # LiDAR / collision settings
        self.num_lidar_bins = 12
        self.scan_clip_max = 5.0
        self.collision_distance = 0.22

        # Action settings
        self.max_linear = 0.25
        self.max_angular = 1.5

        self.action_space = spaces.Box(
            low=np.array([0.0, -self.max_angular], dtype=np.float32),
            high=np.array([self.max_linear, self.max_angular], dtype=np.float32),
            dtype=np.float32,
        )

        obs_low = np.array(
            [0.0] * self.num_lidar_bins + [0.0, -math.pi, -5.0, -5.0],
            dtype=np.float32,
        )
        obs_high = np.array(
            [self.scan_clip_max] * self.num_lidar_bins + [100.0, math.pi, 5.0, 5.0],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32,
        )

        # ROS interfaces
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.node.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Cached messages
        self.scan_msg = None
        self.odom_msg = None

        # Reward shaping memory
        self.prev_distance_to_goal = None

        self.node.get_logger().info("NavBotEnv initialized.")

    def scan_callback(self, msg):
        self.scan_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def _spin_once(self, timeout=0.1):
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def _clear_cached_msgs(self):
        self.scan_msg = None
        self.odom_msg = None

    def _publish_zero_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self.cmd_pub.publish(msg)
            self._spin_once(0.05)

    def _wait_for_fresh_data(self, timeout=6.0):
        start = time.time()
        while time.time() - start < timeout:
            self._spin_once(0.1)
            if self.scan_msg is not None and self.odom_msg is not None:
                return True
        return False

    def _delete_robot(self):
        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", f'name: "{self.robot_name}", type: MODEL'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        #print("\n[DELETE ROBOT CMD]")
        #print("CMD:", " ".join(cmd))
        #print("STDOUT:", result.stdout)
        #print("STDERR:", result.stderr)

        return result.returncode == 0

    def _generate_temp_urdf(self):
        result = subprocess.run(
            ["xacro", self.xacro_path],
            capture_output=True,
            text=True
        )

        print("\n[XACRO TO URDF]")
        print("STDOUT length:", len(result.stdout))
        print("STDERR:", result.stderr)

        if result.returncode != 0:
            raise RuntimeError(f"xacro failed for {self.xacro_path}")

        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        tmp.write(result.stdout)
        tmp.flush()
        tmp.close()
        return tmp.name

    def _spawn_robot(self):
        urdf_path = self._generate_temp_urdf()

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", self.world_name,
            "-name", self.robot_name,
            "-file", urdf_path,
            "-x", str(self.spawn_x),
            "-y", str(self.spawn_y),
            "-z", str(self.spawn_z),
            "-Y", str(self.spawn_yaw),
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        #print("\n[SPAWN ROBOT CMD]")
        #print("CMD:", " ".join(cmd))
        #print("STDOUT:", result.stdout)
        #print("STDERR:", result.stderr)

        return result.returncode == 0

    def _yaw_from_quaternion(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _get_robot_pose(self):
        if self.odom_msg is None:
            return None
        pos = self.odom_msg.pose.pose.position
        ori = self.odom_msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(ori)
        return pos.x, pos.y, yaw

    def _get_robot_twist(self):
        if self.odom_msg is None:
            return 0.0, 0.0
        twist = self.odom_msg.twist.twist
        return twist.linear.x, twist.angular.z

    def _get_distance_to_goal(self):
        pose = self._get_robot_pose()
        if pose is None:
            return 999.0
        x, y, _ = pose
        return math.hypot(self.goal_x - x, self.goal_y - y)

    def _get_angle_to_goal(self):
        pose = self._get_robot_pose()
        if pose is None:
            return 0.0
        x, y, yaw = pose
        goal_heading = math.atan2(self.goal_y - y, self.goal_x - x)
        return self._normalize_angle(goal_heading - yaw)

    def _process_scan(self):
        if self.scan_msg is None:
            return np.full(self.num_lidar_bins, self.scan_clip_max, dtype=np.float32)

        ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), self.scan_clip_max, ranges)
        ranges = np.where(np.isnan(ranges), self.scan_clip_max, ranges)
        ranges = np.clip(ranges, 0.0, self.scan_clip_max)

        if len(ranges) < self.num_lidar_bins:
            padded = np.full(self.num_lidar_bins, self.scan_clip_max, dtype=np.float32)
            padded[:len(ranges)] = ranges
            return padded

        chunks = np.array_split(ranges, self.num_lidar_bins)
        return np.array([float(np.min(chunk)) for chunk in chunks], dtype=np.float32)

    def _get_observation(self):
        lidar = self._process_scan()
        dist = self._get_distance_to_goal()
        angle = self._get_angle_to_goal()
        lin_vel, ang_vel = self._get_robot_twist()

        obs = np.concatenate(
            [
                lidar,
                np.array([dist, angle, lin_vel, ang_vel], dtype=np.float32),
            ]
        ).astype(np.float32)

        return obs

    def _check_collision(self):
        lidar = self._process_scan()
        return float(np.min(lidar)) < self.collision_distance

    def _check_goal_reached(self):
        return self._get_distance_to_goal() < self.goal_tolerance

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0

        self._publish_zero_cmd()
        time.sleep(0.2)

        self._clear_cached_msgs()

        self._delete_robot()
        time.sleep(0.8)

        ok = self._spawn_robot()
        if not ok:
            raise RuntimeError("Failed to respawn robot during reset")

        self._publish_zero_cmd()
        time.sleep(1.0)

        if not self._wait_for_fresh_data(timeout=6.0):
            raise RuntimeError("Robot respawned but no fresh /scan or /odom arrived")

        obs = self._get_observation()
        self.prev_distance_to_goal = self._get_distance_to_goal()

        info = {}
        return obs, info

    def step(self, action):
        self.current_step += 1

        linear_x = float(np.clip(action[0], 0.0, self.max_linear))
        angular_z = float(np.clip(action[1], -self.max_angular, self.max_angular))

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        for _ in range(3):
            self._spin_once(0.1)

        obs = self._get_observation()

        collision = self._check_collision()
        goal_reached = self._check_goal_reached()

        terminated = bool(collision or goal_reached)
        truncated = bool(self.current_step >= self.max_steps)

        current_dist = self._get_distance_to_goal()
        progress = 0.0
        if self.prev_distance_to_goal is not None:
            progress = self.prev_distance_to_goal - current_dist
        self.prev_distance_to_goal = current_dist

        min_scan = float(np.min(self._process_scan()))

        reward = 0.0
        reward += 10.0 * progress
        reward -= 0.01
        reward -= 0.05 * abs(angular_z)

        if min_scan < 0.40:
            reward -= 0.10 * (0.40 - min_scan)

        if goal_reached:
            reward += 50.0

        if collision:
            reward -= 50.0

        info = {
            "distance_to_goal": current_dist,
            "goal_reached": goal_reached,
            "collision": collision,
            "min_scan": min_scan,
        }

        return obs, reward, terminated, truncated, info

    def render(self):
        pass

    def close(self):
        try:
            self._publish_zero_cmd()
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass