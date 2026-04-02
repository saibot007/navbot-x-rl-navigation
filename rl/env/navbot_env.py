import math
import time
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

        # Spawn / reset settings
        self.spawn_x = -1.0
        self.spawn_y = 0.0
        self.spawn_z = 0.02
        self.spawn_yaw = 0.0

        # Goal settings
        self.goal_x = 1.0
        self.goal_y = 0.0
        self.goal_tolerance = 0.15

        # Episode settings
        self.max_steps = 250
        self.current_step = 0
        self.episode_reward = 0.0
        

        # LiDAR / collision settings
        self.num_lidar_bins = 12
        self.scan_clip_max = 5.0
        self.collision_distance = 0.22
        self.safe_distance = 0.50

        # Action settings
        self.max_linear = 0.25
        self.max_angular = 1.5

        # Normalized action space for PPO
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        obs_low = np.array(
            [0.0] * self.num_lidar_bins + [0.0, -math.pi, -self.max_linear, -self.max_angular],
            dtype=np.float32,
        )
        obs_high = np.array(
            [self.scan_clip_max] * self.num_lidar_bins + [100.0, math.pi, self.max_linear, self.max_angular],
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

        # Reward / action memory
        self.prev_distance_to_goal = None
        self.prev_angle_to_goal = None
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.curriculum_stage = 0
        self.episode_count = 0
        self.success_window = []
        self.success_window_size = 50

        # Odom reset reference
        self.reset_odom_x = None
        self.reset_odom_y = None
        self.reset_odom_yaw = None

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

    def _wait_for_fresh_data(self, timeout=3.0):
        start = time.time()
        while time.time() - start < timeout:
            self._spin_once(0.1)
            if self.scan_msg is not None and self.odom_msg is not None:
                return True
        return False

    def _yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

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

    def _reset_pose(self):
        qx, qy, qz, qw = self._yaw_to_quaternion(self.spawn_yaw)

        cmd = [
            "gz", "service",
            "-s", f"/world/{self.world_name}/set_pose/blocking",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req",
            (
                f'name: "{self.robot_name}" '
                f'position {{ x: {self.spawn_x} y: {self.spawn_y} z: {self.spawn_z} }} '
                f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}'
            )
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5
            )

            print("[SET_POSE CMD]", " ".join(cmd))
            print("[SET_POSE STDOUT]", result.stdout)
            print("[SET_POSE STDERR]", result.stderr)

            stdout = result.stdout.lower()
            return result.returncode == 0 and ("data: true" in stdout or stdout.strip() == "")
        except subprocess.TimeoutExpired:
            print("[SET_POSE ERROR] timeout")
            return False

    def _get_robot_pose(self):
        if self.odom_msg is None:
            return None

        pos = self.odom_msg.pose.pose.position
        ori = self.odom_msg.pose.pose.orientation
        odom_yaw = self._yaw_from_quaternion(ori)

        # Before reset reference captured, return raw odom
        if self.reset_odom_x is None or self.reset_odom_y is None or self.reset_odom_yaw is None:
            return pos.x, pos.y, odom_yaw

        dx = pos.x - self.reset_odom_x
        dy = pos.y - self.reset_odom_y
        dyaw = self._normalize_angle(odom_yaw - self.reset_odom_yaw)

        x = self.spawn_x + dx
        y = self.spawn_y + dy
        yaw = self._normalize_angle(self.spawn_yaw + dyaw)

        return x, y, yaw

    def _get_robot_twist(self):
        if self.odom_msg is None:
            return 0.0, 0.0

        twist = self.odom_msg.twist.twist
        lin = float(twist.linear.x)
        ang = float(twist.angular.z)

        if abs(lin) < 1e-6:
            lin = 0.0
        if abs(ang) < 1e-6:
            ang = 0.0

        lin = float(np.clip(lin, -self.max_linear, self.max_linear))
        ang = float(np.clip(ang, -self.max_angular, self.max_angular))

        return lin, ang

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
    
    def _update_curriculum(self):
        return

        success_rate = sum(self.success_window) / len(self.success_window)

        if self.curriculum_stage == 0 and success_rate >= 0.60:
            self.curriculum_stage = 1
            print("[CURRICULUM] Advanced to stage 1")

        elif self.curriculum_stage == 1 and success_rate >= 0.60:
            self.curriculum_stage = 2
            print("[CURRICULUM] Advanced to stage 2")


    def _sample_task(self):
        if self.curriculum_stage == 0:
            self.spawn_x = -0.55
            self.spawn_y = 0.0
            self.spawn_yaw = 0.0

            self.goal_x = 0.55
            self.goal_y = 0.0

            self.goal_tolerance = 0.28
            self.max_steps = 260

        elif self.curriculum_stage == 1:
            self.spawn_x = float(np.random.uniform(-0.75, -0.55))
            self.spawn_y = float(np.random.uniform(-0.15, 0.15))
            self.spawn_yaw = float(np.random.uniform(-0.35, 0.35))

            self.goal_x = float(np.random.uniform(0.45, 0.70))
            self.goal_y = float(np.random.uniform(-0.15, 0.15))

            self.goal_tolerance = 0.18
            self.max_steps = 220

        else:
            self.spawn_x = float(np.random.uniform(-1.00, -0.85))
            self.spawn_y = float(np.random.uniform(-0.20, 0.20))
            self.spawn_yaw = float(np.random.uniform(-0.50, 0.50))

            self.goal_x = float(np.random.uniform(0.85, 1.00))
            self.goal_y = float(np.random.uniform(-0.20, 0.20))

            self.goal_tolerance = 0.15
            self.max_steps = 250

    def _check_goal_reached(self):
        return self._get_distance_to_goal() < self.goal_tolerance

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0
        self.episode_reward = 0.0
        self._update_curriculum()
        self._sample_task()
        self.episode_count += 1

        self._publish_zero_cmd()
        time.sleep(0.2)
        self._publish_zero_cmd()
        time.sleep(0.2)

        ok = self._reset_pose()
        if not ok:
            raise RuntimeError("Pose reset failed")

        time.sleep(0.5)

        self._publish_zero_cmd()
        time.sleep(0.3)
        self._publish_zero_cmd()
        time.sleep(0.2)

        self._clear_cached_msgs()

        if not self._wait_for_fresh_data(timeout=3.0):
            raise RuntimeError("Pose reset worked but no fresh /scan or /odom arrived")

        # Capture odom reference immediately after reset
        raw_pos = self.odom_msg.pose.pose.position
        raw_ori = self.odom_msg.pose.pose.orientation
        self.reset_odom_x = raw_pos.x
        self.reset_odom_y = raw_pos.y
        self.reset_odom_yaw = self._yaw_from_quaternion(raw_ori)

        obs = self._get_observation()
        pose = self._get_robot_pose()
        print(f"[RESET ACTUAL POSE] {pose}")

        self.prev_distance_to_goal = self._get_distance_to_goal()
        self.prev_angle_to_goal = abs(self._get_angle_to_goal())
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0

        info = {}
        return obs, info

    def step(self, action):
        self.current_step += 1

        raw_forward = float(np.clip(action[0], 0.0, 1.0))
        raw_turn = float(np.clip(action[1], -1.0, 1.0))

        raw_linear_x = self.max_linear * raw_forward
        raw_angular_z = self.max_angular * raw_turn

        alpha_lin = 0.3
        alpha_ang = 0.4

        linear_x = alpha_lin * self.prev_linear_x + (1.0 - alpha_lin) * raw_linear_x
        angular_z = alpha_ang * self.prev_angular_z + (1.0 - alpha_ang) * raw_angular_z

        

        angular_change = abs(angular_z - self.prev_angular_z)
        linear_change = abs(linear_x - self.prev_linear_x)

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        for _ in range(6):
            self.cmd_pub.publish(cmd)
            self._spin_once(0.05)

        self.prev_linear_x = linear_x
        self.prev_angular_z = angular_z

        obs = self._get_observation()

        collision = self._check_collision()
        goal_reached = self._check_goal_reached()

        terminated = bool(collision or goal_reached)
        truncated = bool(self.current_step >= self.max_steps)

        current_dist = self._get_distance_to_goal()
        current_angle = abs(self._get_angle_to_goal())
        min_scan = float(np.min(self._process_scan()))

        progress = 0.0
        if self.prev_distance_to_goal is not None:
            progress = self.prev_distance_to_goal - current_dist

        angle_improvement = 0.0
        if self.prev_angle_to_goal is not None:
            angle_improvement = self.prev_angle_to_goal - current_angle

        self.prev_distance_to_goal = current_dist
        self.prev_angle_to_goal = current_angle

        reward = 0.0

        reward += 25.0 * progress
        reward += 2.0 * angle_improvement

        aligned_forward = max(0.0, math.cos(current_angle))
        reward += 0.12 * linear_x * aligned_forward

        reward -= 0.01
        reward -= 0.02 * abs(angular_z)
        reward -= 0.04 * angular_change
        reward -= 0.01 * linear_change
       

        if min_scan < self.safe_distance:
            reward -= 0.25 * (self.safe_distance - min_scan)

        if linear_x < 0.03 and current_dist > self.goal_tolerance:
            reward -= 0.20

        if goal_reached:
            reward += 120.0

        if collision:
            reward -= 120.0

        self.episode_reward += reward

        odom_lin, odom_ang = self._get_robot_twist()

        info = {
           "distance_to_goal": current_dist,
           "goal_reached": goal_reached,
           "collision": collision,
           "min_scan": min_scan,
           "episode_step": self.current_step,
           "cmd_linear_x": float(linear_x),
           "cmd_angular_z": float(angular_z),
           "odom_linear_x": float(odom_lin),
           "odom_angular_z": float(odom_ang),
           "angle_to_goal": float(current_angle),
        }

        if terminated or truncated:
            info["episode"] = {
                "r": float(self.episode_reward),
                "l": int(self.current_step),
                "success": int(goal_reached),
                "collision": int(collision),
            }

            self.success_window.append(int(goal_reached))
            if len(self.success_window) > self.success_window_size:
                self.success_window.pop(0)

            print(f"[EP END] step={self.current_step}, collision={collision}, goal={goal_reached}, truncated={truncated}")
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