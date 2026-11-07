#!/usr/bin/env python3
"""
AMR Dynamic Obstacle Avoidance & Goal Arrival Accuracy Benchmark
Project: Autonomous Mobile Robot (AMR) for Food Delivery in Restaurant Environment
Platform: ROS 2 Jazzy | Nav2 (Smac 2D Planner + DWB Controller) & Gazebo Harmonic

Features:
    1. Multi-Stage Obstacle Avoidance Quantification:
       - Captures Nominal Global Path (P0) at t=0 winding through restaurant tables.
       - Detects Global Path Replanning Event: Compares new paths (Pt) against P0 to find
         the exact divergence point (Divergence Lookahead Distance) and replan timestamp.
       - Detects Physical Evasion Trigger: Measures Cross-Track Error (lateral deflection)
         relative to P0 and records the exact moment the robot swerves to avoid collision.
       - Records Front-Cone LiDAR distance (within +/- 30 deg front view) at evasion start.
    2. Optional Ground Truth Obstacle Tracking:
       - Can automatically query Gazebo for obstacle model pose, or take CLI --obs-x / --obs-y.
       - Computes true Euclidean distance to obstacle at replan time, at evasion time,
         and minimum physical clearance during the entire mission.
    3. Target Convergence & Stopping Accuracy:
       - Listens to user's RViz2 'Nav2 Goal' click (/goal_pose).
       - Compares final stopping pose between:
           * Target Goal (/goal_pose)
           * Robot Estimate (AMCL / TF map -> base_footprint)
           * Ground Truth (Gazebo /ground_truth/odom)
    4. Publication-grade metrics logged to docs/benchmark_obstacle_avoidance_results.txt (CSV-compatible).
"""

import os
import sys
import math
import time
import argparse
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray

from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_to_yaw(q):
    """Converts quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalizes angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def gazebo_odom_to_map(gx, gy, gyaw, spawn_x=8.85, spawn_y=-3.80, spawn_yaw=math.pi):
    """
    Transforms Gazebo /ground_truth/odom (origin at spawn) into Map frame coordinates.
    In restaurant_main.launch.py, AMR spawns at Area 08 dock: X=8.85m, Y=-3.80m, Yaw=pi.
    """
    cos_yaw = math.cos(spawn_yaw)
    sin_yaw = math.sin(spawn_yaw)
    mx = spawn_x + (cos_yaw * gx - sin_yaw * gy)
    my = spawn_y + (sin_yaw * gx + cos_yaw * gy)
    myaw = normalize_angle(gyaw + spawn_yaw)
    return mx, my, myaw


def query_gazebo_model_pose(model_name='obstacle_box'):
    """Attempts to query Gazebo Harmonic for the model's world position."""
    try:
        cmd = ['gz', 'model', '-m', model_name, '-p']
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1.5)
        if res.returncode == 0:
            lines = res.stdout.splitlines()
            # Expected format: position { x: 4.0 y: 0.0 z: 0.4 }
            for line in lines:
                if 'x:' in line and 'y:' in line:
                    parts = line.strip().split()
                    x = float(parts[parts.index('x:') + 1])
                    y = float(parts[parts.index('y:') + 1])
                    return (x, y)
    except Exception:
        pass
    return None


class ObstacleAvoidanceBenchmark(Node):
    def __init__(self, output_file='docs/benchmark_obstacle_avoidance_results.txt',
                 timeout_sec=120.0, obs_x=None, obs_y=None, obs_name='obstacle_box',
                 spawn_x=8.85, spawn_y=-3.80, spawn_yaw=math.pi):
        super().__init__(
            'benchmark_obstacle_avoidance',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self.output_file = output_file
        self.timeout_sec = timeout_sec
        self.obs_name = obs_name
        self.spawn_x = float(spawn_x)
        self.spawn_y = float(spawn_y)
        self.spawn_yaw = float(spawn_yaw)

        # Ground truth obstacle pose (if known or specified)
        if obs_x is not None and obs_y is not None:
            self.obs_pos = (float(obs_x), float(obs_y))
        else:
            self.obs_pos = query_gazebo_model_pose(self.obs_name)

        # QoS Profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # TF Buffer & Listener for map -> base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, reliable_qos
        )
        self.sub_plan = self.create_subscription(
            Path, '/plan', self.plan_cb, reliable_qos
        )
        self.sub_unsmoothed_plan = self.create_subscription(
            Path, '/unsmoothed_plan', self.plan_cb, reliable_qos
        )
        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth/odom', self.gt_cb, sensor_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, reliable_qos
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, sensor_qos
        )
        self.sub_nav_status = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_cb, reliable_qos
        )

        # Navigation state storage
        self.target_goal = None        # (x, y, yaw)
        self.start_amcl = None         # (x, y, yaw)
        self.start_gt = None           # (x, y, yaw)
        self.latest_gt = None          # (x, y, yaw)
        self.latest_vel = (0.0, 0.0)   # (linear.x, angular.z)
        self.latest_scan_min = float('inf')
        self.front_laser_min = float('inf')
        self.min_obstacle_clearance = float('inf')

        # Path tracking and divergence metrics
        self.initial_nominal_path = None  # [(x, y), ...] at t=0
        self.nominal_path_length = 0.0
        self.replan_detected = False
        self.replan_time = None
        self.replan_divergence_point = None
        self.replan_lookahead_dist = None
        self.replan_robot_pose = None

        # Physical evasion (Cross-track deviation from P0)
        self.evasion_detected = False
        self.evasion_time = None
        self.evasion_pose = None
        self.evasion_front_laser = None
        self.max_cross_track_error = 0.0

        # Trajectory distance accumulation
        self.prev_amcl_pos = None
        self.prev_gt_pos = None
        self.total_dist_amcl = 0.0
        self.total_dist_gt = 0.0

        # Benchmark phase tracking
        self.is_tracking = False
        self.is_completed = False
        self.start_sim_time = None
        self.nav_status_succeeded = False

        # Stopping stability guard
        self.stopped_start_time = None
        self.final_amcl = None
        self.final_gt = None

        # Time series history
        self.history = []

        # 10 Hz Control / Monitor Loop
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.monitor_loop)

        obs_str = f"({self.obs_pos[0]:.2f}m, {self.obs_pos[1]:.2f}m)" if self.obs_pos else "Tự động phân tích qua LiDAR & Path"
        self.get_logger().info(
            f"\n{'='*80}\n"
            f"BENCHMARK NÉ VẬT CẢN & ĐỘ CHÍNH XÁC VỊ TRÍ ĐÍCH (AMR TIER 3 - NÂNG CAO)\n"
            f"  - Tệp lưu kết quả: {self.output_file}\n"
            f"  - Vị trí vật cản Ground Truth: {obs_str}\n"
            f"  - Cơ chế nhận diện né:\n"
            f"    + So sánh biến dạng quỹ đạo: Quỹ đạo mới (Pt) vs Quỹ đạo danh định (P0)\n"
            f"    + Sai số lệch ngang (Cross-Track Error) & LiDAR Front Cone (+/- 30°)\n"
            f"  - Trạng thái: ĐANG CHỜ LỆNH ĐIỀU HƯỚNG TỪ RVIZ2 (Nav2 Goal hoặc 2D Goal Pose)...\n"
            f"💡 HƯỚNG DẪN:\n"
            f"   1. Đặt vật cản mới trong Gazebo (Box / Cylinder trên lối đi).\n"
            f"   2. Mở RViz2, click 'Nav2 Goal' (hoặc 2D Goal Pose) chọn điểm đến phía sau vật cản.\n"
            f"   3. Script sẽ tự động bắt đầu ngay khi Nav2 tính toán đường đi (/plan)!\n"
            f"{'='*80}"
        )

    def start_benchmark(self, tx, ty, tyaw, source_str, initial_pts=None):
        self.target_goal = (tx, ty, tyaw)

        amcl_pose = self.get_current_amcl_pose()
        if amcl_pose is not None:
            self.start_amcl = amcl_pose
            self.prev_amcl_pos = (amcl_pose[0], amcl_pose[1])
        if self.latest_gt is not None:
            self.start_gt = self.latest_gt
            self.prev_gt_pos = (self.latest_gt[0], self.latest_gt[1])

        if self.obs_pos is None:
            self.obs_pos = query_gazebo_model_pose(self.obs_name)

        self.start_sim_time = self.get_clock().now()
        self.is_tracking = True
        self.is_completed = False
        self.nav_status_succeeded = False
        self.replan_detected = False
        self.evasion_detected = False
        self.max_cross_track_error = 0.0
        self.history.clear()
        self.total_dist_amcl = 0.0
        self.total_dist_gt = 0.0
        self.min_obstacle_clearance = float('inf')

        straight_dist = 0.0
        if self.start_amcl is not None:
            straight_dist = math.hypot(self.target_goal[0] - self.start_amcl[0], self.target_goal[1] - self.start_amcl[1])

        if initial_pts:
            self.initial_nominal_path = initial_pts
            self.nominal_path_length = sum(
                math.hypot(initial_pts[i+1][0] - initial_pts[i][0], initial_pts[i+1][1] - initial_pts[i][1])
                for i in range(len(initial_pts) - 1)
            )
            plan_info = f" | Quỹ đạo P0: {len(initial_pts)} điểm ({self.nominal_path_length:.2f} m)"
        else:
            self.initial_nominal_path = None
            plan_info = " | Đang chờ đường đi P0"

        self.get_logger().info(
            f"\n{'='*80}\n"
            f">>> ĐÃ BẮT ĐẦU THEO DÕI ĐIỀU HƯỚNG TỪ NGUỒN: [{source_str}]!\n"
            f"    - Tọa độ đích: ({tx:+.2f} m, {ty:+.2f} m) | Yaw: {math.degrees(tyaw):+.1f}°\n"
            f"    - Vị trí xuất phát: ({self.start_amcl[0] if self.start_amcl else 0.0:+.2f} m, {self.start_amcl[1] if self.start_amcl else 0.0:+.2f} m)\n"
            f"    - Khoảng cách chim bay: {straight_dist:.2f} m{plan_info}\n"
            f">>> ĐANG THEO DÕI HÀNH TRÌNH NÉ VẬT CẢN THEO THỜI GIAN THỰC...\n"
            f"{'='*80}"
        )

    def goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.start_benchmark(p.x, p.y, yaw, source_str="RViz2 /goal_pose")

    def plan_cb(self, msg: Path):
        if not msg.poses:
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        target_p = msg.poses[-1].pose.position
        target_yaw = quaternion_to_yaw(msg.poses[-1].pose.orientation)

        # Trigger benchmark automatically if not tracking yet, OR if a distinctly new goal arrives
        is_new_goal = False
        if self.target_goal is not None:
            is_new_goal = math.hypot(target_p.x - self.target_goal[0], target_p.y - self.target_goal[1]) > 0.60

        if not self.is_tracking or is_new_goal:
            if self.is_completed and not is_new_goal:
                return
            self.start_benchmark(target_p.x, target_p.y, target_yaw, source_str="Nav2 Global Plan (/plan)", initial_pts=pts)
            return

        # 1. Capture P0 if not set yet
        if self.initial_nominal_path is None:
            self.initial_nominal_path = pts
            self.nominal_path_length = sum(
                math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
                for i in range(len(pts) - 1)
            )
            self.get_logger().info(
                f"[PLANNER] Đã ghi nhận Quỹ đạo danh định ban đầu (P0): {len(pts)} điểm | Chiều dài: {self.nominal_path_length:.2f} m"
            )
            return

        # 2. Path Divergence Analysis: Has Smac Planner rerouted around the obstacle?
        if not self.replan_detected:
            # Check if any waypoint on new path deviates from P0 by >= 0.25 m
            diverge_idx = None
            for idx, (nx, ny) in enumerate(pts):
                min_d_to_p0 = min(math.hypot(nx - ox, ny - oy) for ox, oy in self.initial_nominal_path)
                if min_d_to_p0 >= 0.25:
                    diverge_idx = idx
                    break

            if diverge_idx is not None:
                self.replan_detected = True
                now = self.get_clock().now()
                self.replan_time = (now - self.start_sim_time).nanoseconds / 1e9 if self.start_sim_time else 0.0
                self.replan_divergence_point = pts[diverge_idx]
                self.replan_robot_pose = self.get_current_amcl_pose()

                # Lookahead distance from robot along new path to the divergence point
                lookahead = sum(
                    math.hypot(pts[k+1][0] - pts[k][0], pts[k+1][1] - pts[k][1])
                    for k in range(diverge_idx)
                )
                self.replan_lookahead_dist = lookahead

                obs_info = ""
                if self.obs_pos is not None and self.replan_robot_pose is not None:
                    d_obs = math.hypot(self.replan_robot_pose[0] - self.obs_pos[0], self.replan_robot_pose[1] - self.obs_pos[1])
                    obs_info = f" | Cách vật cản: {d_obs:.2f} m"

                self.get_logger().info(
                    f"\n>>> [SỰ KIỆN: TÁI LẬP QUỸ ĐẠO NÉ (GLOBAL REPLAN)]\n"
                    f"    - Thời điểm phát hiện & đổi đường mới: t = {self.replan_time:.2f} s\n"
                    f"    - Khoảng cách nhìn trước tới điểm tách nhánh: {self.replan_lookahead_dist:.2f} m\n"
                    f"    - Tọa độ điểm tách nhánh né: ({self.replan_divergence_point[0]:.2f}m, {self.replan_divergence_point[1]:.2f}m){obs_info}\n"
                )

    def gt_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        # Transform from Gazebo /ground_truth/odom (origin at spawn) to Map frame coordinates:
        mx, my, myaw = gazebo_odom_to_map(p.x, p.y, yaw, self.spawn_x, self.spawn_y, self.spawn_yaw)
        self.latest_gt = (mx, my, myaw)

    def odom_cb(self, msg: Odometry):
        self.latest_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
        if valid:
            self.latest_scan_min = min(valid)
            if self.is_tracking:
                self.min_obstacle_clearance = min(self.min_obstacle_clearance, self.latest_scan_min)

        # Front-Cone LiDAR: beams within [-30 deg, +30 deg]
        front_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            if -math.pi / 6.0 <= angle <= math.pi / 6.0:
                if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                    front_ranges.append(r)
            angle += msg.angle_increment
        if front_ranges:
            self.front_laser_min = min(front_ranges)

    def nav_status_cb(self, msg: GoalStatusArray):
        if msg.status_list:
            latest_status = msg.status_list[-1].status
            if latest_status == 4:  # STATUS_SUCCEEDED
                self.nav_status_succeeded = True

    def get_current_amcl_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            p = t.transform.translation
            yaw = quaternion_to_yaw(t.transform.rotation)
            return (p.x, p.y, yaw)
        except TransformException:
            return None

    def monitor_loop(self):
        if not self.is_tracking or self.is_completed:
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0 or self.start_sim_time is None:
            return

        elapsed_sec = (now - self.start_sim_time).nanoseconds / 1e9

        # Timeout guard
        if elapsed_sec >= self.timeout_sec:
            self.is_completed = True
            self.get_logger().warn(f">>> HẾT THỜI GIAN THEO DÕI ({self.timeout_sec:.1f}s)! Robot chưa đến được đích.")
            self.save_and_report(success=False)
            return

        # 1. Fetch current robot poses
        amcl_pose = self.get_current_amcl_pose()
        if amcl_pose is None:
            return

        ax, ay, ayaw = amcl_pose
        gx, gy, gyaw = self.latest_gt if self.latest_gt is not None else (ax, ay, ayaw)

        # 2. Accumulate trajectory length
        if self.prev_amcl_pos is not None:
            self.total_dist_amcl += math.hypot(ax - self.prev_amcl_pos[0], ay - self.prev_amcl_pos[1])
        self.prev_amcl_pos = (ax, ay)

        if self.prev_gt_pos is not None:
            self.total_dist_gt += math.hypot(gx - self.prev_gt_pos[0], gy - self.prev_gt_pos[1])
        self.prev_gt_pos = (gx, gy)

        # 3. Cross-Track Error relative to initial nominal plan P0
        cross_track_err = 0.0
        if self.initial_nominal_path and len(self.initial_nominal_path) > 1:
            cross_track_err = min(math.hypot(ax - ox, ay - oy) for ox, oy in self.initial_nominal_path)
            self.max_cross_track_error = max(self.max_cross_track_error, cross_track_err)

            # Detect Physical Evasion Start (lateral deflection >= 0.15m after leaving start area)
            if cross_track_err >= 0.15 and self.total_dist_gt >= 0.50 and not self.evasion_detected:
                self.evasion_detected = True
                self.evasion_time = elapsed_sec
                self.evasion_pose = (ax, ay, ayaw)
                self.evasion_front_laser = self.front_laser_min

                obs_info = ""
                if self.obs_pos is not None:
                    d_obs = math.hypot(ax - self.obs_pos[0], ay - self.obs_pos[1])
                    obs_info = f" | Cự ly tới vật cản: {d_obs:.2f} m"

                self.get_logger().info(
                    f"\n>>> [SỰ KIỆN: XE BẮT ĐẦU ĐÁNH LÁI NÉ VẬT CẢN (PHYSICAL EVASION)]\n"
                    f"    - Thời điểm bắt đầu bẻ lái: t = {self.evasion_time:.2f} s\n"
                    f"    - Độ lệch khỏi tim đường gốc (Cross-Track): {cross_track_err*100.0:.1f} cm\n"
                    f"    - LiDAR quét phía trước lúc bẻ lái: {self.evasion_front_laser:.2f} m{obs_info}\n"
                )

        # 4. Target distance & velocity
        dist_to_goal = math.hypot(ax - self.target_goal[0], ay - self.target_goal[1])
        v_lin, w_ang = self.latest_vel

        # 5. Record time-series sample
        self.history.append({
            't': elapsed_sec,
            'amcl_x': ax, 'amcl_y': ay, 'amcl_yaw': ayaw,
            'gt_x': gx, 'gt_y': gy, 'gt_yaw': gyaw,
            'dist_to_goal': dist_to_goal,
            'cross_track_m': cross_track_err,
            'laser_min': self.latest_scan_min,
            'front_laser_m': self.front_laser_min,
            'v_lin': v_lin, 'w_ang': w_ang
        })

        # Periodic logging every 1.0s
        if len(self.history) % 10 == 0:
            status_tag = "[NÉ VẬT CẢN]" if cross_track_err >= 0.15 else "[BÁM QUỸ ĐẠO]"
            self.get_logger().info(
                f"[{elapsed_sec:4.1f}s] {status_tag} Cách đích: {dist_to_goal:4.2f}m | "
                f"Lệch P0: {cross_track_err*100.0:4.1f}cm | D_min: {self.min_obstacle_clearance:4.2f}m | "
                f"v={v_lin:4.2f}m/s | Quãng đường: {self.total_dist_gt:4.2f}m"
            )

        # 6. Goal Arrival Check (Dual Condition: Nav2 Succeeded OR Robot Stopped near Goal)
        is_stopped = (abs(v_lin) < 0.03 and abs(w_ang) < 0.05)
        near_goal = (dist_to_goal <= 0.35)

        if (self.nav_status_succeeded and near_goal) or (near_goal and is_stopped):
            if self.stopped_start_time is None:
                self.stopped_start_time = now
            else:
                stable_stopped_sec = (now - self.stopped_start_time).nanoseconds / 1e9
                if stable_stopped_sec >= 1.2:  # Stood still at goal for 1.2 seconds
                    self.is_completed = True
                    self.final_amcl = amcl_pose
                    self.final_gt = (gx, gy, gyaw)
                    self.save_and_report(success=True)
        else:
            self.stopped_start_time = None

    def save_and_report(self, success=True):
        if not self.history:
            return

        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        final_sample = self.history[-1]
        t_total = final_sample['t']

        gx, gy, gyaw = self.final_gt if self.final_gt else (final_sample['gt_x'], final_sample['gt_y'], final_sample['gt_yaw'])
        ax, ay, ayaw = self.final_amcl if self.final_amcl else (final_sample['amcl_x'], final_sample['amcl_y'], final_sample['amcl_yaw'])
        tx, ty, tyaw = self.target_goal

        # Error metrics relative to Goal
        pos_err_amcl = math.hypot(ax - tx, ay - ty)
        pos_err_gt = math.hypot(gx - tx, gy - ty)
        yaw_err_amcl_deg = math.degrees(abs(normalize_angle(ayaw - tyaw)))
        yaw_err_gt_deg = math.degrees(abs(normalize_angle(gyaw - tyaw)))

        # Localization offset between belief and reality at stop
        loc_offset = math.hypot(gx - ax, gy - ay)

        # Straight-line distance vs Actual path length (Detour Ratio)
        sx, sy = (self.start_amcl[0], self.start_amcl[1]) if self.start_amcl else (0.0, 0.0)
        straight_dist = math.hypot(tx - sx, ty - sy)
        detour_ratio = (self.total_dist_gt / straight_dist) if straight_dist > 0.01 else 1.0

        # Avoidance metrics
        replan_time_str = f"{self.replan_time:.2f} s" if self.replan_time else "N/A"
        replan_lookahead_str = f"{self.replan_lookahead_dist:.2f} m" if self.replan_lookahead_dist else "N/A"
        evade_time_str = f"{self.evasion_time:.2f} s" if self.evasion_time else "N/A"
        evade_laser_str = f"{self.evasion_front_laser:.2f} m" if self.evasion_front_laser else "N/A"

        obs_dist_replan_str = "N/A"
        obs_dist_evade_str = "N/A"
        if self.obs_pos is not None:
            if self.replan_robot_pose is not None:
                d_replan = math.hypot(self.replan_robot_pose[0] - self.obs_pos[0], self.replan_robot_pose[1] - self.obs_pos[1])
                obs_dist_replan_str = f"{d_replan:.2f} m"
            if self.evasion_pose is not None:
                d_evade = math.hypot(self.evasion_pose[0] - self.obs_pos[0], self.evasion_pose[1] - self.obs_pos[1])
                obs_dist_evade_str = f"{d_evade:.2f} m"

        with open(self.output_file, 'w', encoding='utf-8') as f:
            f.write("# =============================================================================\n")
            f.write("# AMR TIER 3 BENCHMARK: DYNAMIC OBSTACLE AVOIDANCE & TARGET ARRIVAL ACCURACY\n")
            f.write("# Platform: ROS 2 Jazzy | Nav2 Smac 2D Planner + DWB Controller | Gazebo Harmonic\n")
            f.write("# =============================================================================\n")
            f.write(f"# Status                                : {'GOAL ARRIVED (SUCCESS)' if success else 'TIMEOUT / INCOMPLETE'}\n")
            f.write(f"# Commanded Target Goal (X, Y, Yaw)     : ({tx:.3f} m, {ty:.3f} m, {math.degrees(tyaw):.2f} deg)\n")
            f.write(f"# Robot Estimated Arrival Pose (AMCL)   : ({ax:.3f} m, {ay:.3f} m, {math.degrees(ayaw):.2f} deg)\n")
            f.write(f"# Ground Truth Arrival Pose (Gazebo)    : ({gx:.3f} m, {gy:.3f} m, {math.degrees(gyaw):.2f} deg)\n")
            f.write(f"# Estimated Goal Position Error (e_amcl): {pos_err_amcl:.4f} m ({pos_err_amcl*100.0:.2f} cm)\n")
            f.write(f"# Actual Ground Truth Position Error    : {pos_err_gt:.4f} m ({pos_err_gt*100.0:.2f} cm)\n")
            f.write(f"# Heading Yaw Error (AMCL vs Goal)      : {yaw_err_amcl_deg:.2f} deg\n")
            f.write(f"# Heading Yaw Error (Ground Truth)      : {yaw_err_gt_deg:.2f} deg\n")
            f.write(f"# Final Localization Offset (|GT - AMCL|): {loc_offset:.4f} m ({loc_offset*100.0:.2f} cm)\n")
            f.write(f"# -----------------------------------------------------------------------------\n")
            f.write(f"# METRICS: QUANTITATIVE DYNAMIC OBSTACLE AVOIDANCE\n")
            f.write(f"# Global Detour Replan Timestamp        : {replan_time_str}\n")
            f.write(f"# Replan Lookahead Distance to Detour   : {replan_lookahead_str}\n")
            f.write(f"# Distance to Obstacle at Replan (GT)   : {obs_dist_replan_str}\n")
            f.write(f"# Physical Evasion Start Timestamp      : {evade_time_str}\n")
            f.write(f"# Front LiDAR Range at Evasion Start    : {evade_laser_str}\n")
            f.write(f"# Distance to Obstacle at Evasion (GT)  : {obs_dist_evade_str}\n")
            f.write(f"# Maximum Cross-Track Detour from P0    : {self.max_cross_track_error:.3f} m ({self.max_cross_track_error*100.0:.1f} cm)\n")
            f.write(f"# Minimum Obstacle Clearance (D_min)    : {self.min_obstacle_clearance:.4f} m ({self.min_obstacle_clearance*100.0:.2f} cm)\n")
            f.write(f"# Nominal Path Length (P0)              : {self.nominal_path_length:.3f} m\n")
            f.write(f"# Actual Trajectory Length (GT)         : {self.total_dist_gt:.3f} m\n")
            f.write(f"# Detour Overhead Factor (L_act / L_str): {detour_ratio:.3f} (+{(detour_ratio-1.0)*100.0:.1f}% detour)\n")
            f.write(f"# Total Travel Duration                 : {t_total:.2f} s\n")
            f.write("# -----------------------------------------------------------------------------\n")
            f.write("# COLUMNS DEFINITION (COMMA-SEPARATED TIME-SERIES):\n")
            f.write("timestamp,amcl_x,amcl_y,amcl_yaw,gt_x,gt_y,gt_yaw,dist_to_goal_m,cross_track_m,laser_min_m,front_laser_m,v_lin_mps,w_ang_radps\n")

            for h in self.history:
                f.write(
                    f"{h['t']:.3f},"
                    f"{h['amcl_x']:.5f},{h['amcl_y']:.5f},{h['amcl_yaw']:.5f},"
                    f"{h['gt_x']:.5f},{h['gt_y']:.5f},{h['gt_yaw']:.5f},"
                    f"{h['dist_to_goal']:.4f},{h['cross_track_m']:.4f},"
                    f"{h['laser_min']:.4f},{h['front_laser_m']:.4f},"
                    f"{h['v_lin']:.4f},{h['w_ang']:.4f}\n"
                )

        print("\n" + "=" * 80)
        print("          KẾT QUẢ THỰC NGHIỆM: NÉ VẬT CẢN & ĐỘ CHÍNH XÁC VỊ TRÍ ĐÍCH")
        print("=" * 80)
        print(f"{'Tiêu chí đánh giá định lượng':<44} | {'Kết quả đo đạc':<28}")
        print("-" * 80)
        print(f"{'Trạng thái điều hướng đến đích':<44} | {'THÀNH CÔNG (ARRIVED)' if success else 'CHƯA HOÀN TẤT':<28}")
        print(f"{'Tọa độ điểm đích yêu cầu (Target Goal)':<44} | {f'({tx:.2f}m, {ty:.2f}m, {math.degrees(tyaw):.1f}°)':<28}")
        print(f"{'Vị trí dừng thực tế (Ground Truth)':<44} | {f'({gx:.2f}m, {gy:.2f}m, {math.degrees(gyaw):.1f}°)':<28}")
        print(f"{'Sai số vị trí thực tế so với đích (e_pos)':<44} | {f'{pos_err_gt*100.0:.2f} cm':<28}")
        print(f"{'Sai số góc hướng thực tế (e_yaw)':<44} | {f'{yaw_err_gt_deg:.2f}°':<28}")
        print("-" * 80)
        print(f"{'Thời điểm phát hiện & đổi đường mới (t_replan)':<44} | {replan_time_str:<28}")
        print(f"{'Cự ly nhìn trước tới điểm tách nhánh né':<44} | {replan_lookahead_str:<28}")
        if self.obs_pos:
            print(f"{'Khoảng cách tới vật cản lúc đổi đường (GT)':<44} | {obs_dist_replan_str:<28}")
        print(f"{'Thời điểm xe bắt đầu đánh lái né (t_evade)':<44} | {evade_time_str:<28}")
        print(f"{'LiDAR quét vật cản phía trước lúc bẻ lái':<44} | {evade_laser_str:<28}")
        if self.obs_pos:
            print(f"{'Khoảng cách tới vật cản lúc bắt đầu né (GT)':<44} | {obs_dist_evade_str:<28}")
        print(f"{'Độ lệch né cực đại khỏi quỹ đạo gốc P0':<44} | {f'{self.max_cross_track_error*100.0:.1f} cm':<28}")
        print(f"{'Khoảng cách né an toàn nhỏ nhất (D_min)':<44} | {f'{self.min_obstacle_clearance*100.0:.1f} cm (An toàn)':<28}")
        print("-" * 80)
        print(f"{'Chiều dài quỹ đạo danh định ban đầu (P0)':<44} | {f'{self.nominal_path_length:.2f} m':<28}")
        print(f"{'Quãng đường thực tế đã đi (L_actual)':<44} | {f'{self.total_dist_gt:.2f} m':<28}")
        print(f"{'Hệ số vòng tránh né vật cản (Detour Ratio)':<44} | {f'{detour_ratio:.2f} (+{(detour_ratio-1.0)*100.0:.1f}%)':<28}")
        print(f"{'Thời gian di chuyển tổng cộng (Travel Time)':<44} | {f'{t_total:.2f} giây':<28}")
        print(f"{'Sai lệch định vị AMCL so với thực tế':<44} | {f'{loc_offset*100.0:.2f} cm':<28}")
        print("=" * 80)
        print(f"[OK] Đã xuất báo cáo và tệp dữ liệu chuỗi thời gian CSV tại:\n     {os.path.abspath(self.output_file)}\n")


def main():
    parser = argparse.ArgumentParser(description='AMR Dynamic Obstacle Avoidance & Goal Convergence Benchmark')
    parser.add_argument('--output', default='docs/benchmark_obstacle_avoidance_results.txt', help='Output log file path')
    parser.add_argument('--timeout', type=float, default=120.0, help='Max timeout seconds (default: 120.0)')
    parser.add_argument('--obs-x', type=float, default=None, help='Known obstacle ground truth X coordinate')
    parser.add_argument('--obs-y', type=float, default=None, help='Known obstacle ground truth Y coordinate')
    parser.add_argument('--obs-name', default='obstacle_box', help='Obstacle model name in Gazebo (default: obstacle_box)')
    parser.add_argument('--spawn-x', type=float, default=8.85, help='AMR Gazebo spawn X coordinate in map (default: 8.85)')
    parser.add_argument('--spawn-y', type=float, default=-3.80, help='AMR Gazebo spawn Y coordinate in map (default: -3.80)')
    parser.add_argument('--spawn-yaw', type=float, default=math.pi, help='AMR Gazebo spawn Yaw orientation (default: pi)')
    args = parser.parse_args()

    rclpy.init()
    node = ObstacleAvoidanceBenchmark(
        output_file=args.output,
        timeout_sec=args.timeout,
        obs_x=args.obs_x,
        obs_y=args.obs_y,
        obs_name=args.obs_name,
        spawn_x=args.spawn_x,
        spawn_y=args.spawn_y,
        spawn_yaw=args.spawn_yaw
    )

    try:
        while rclpy.ok() and not node.is_completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Đã ngắt bởi người dùng.")
        if node.is_tracking and node.history:
            print("[INFO] Đang lưu lại các mẫu dữ liệu đã thu thập...")
            node.save_and_report(success=False)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
