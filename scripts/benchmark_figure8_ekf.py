#!/usr/bin/env python3
"""
Figure-8 Live EKF Benchmark & Stability Evaluator for Gazebo & ROS 2
Project: Autonomous Mobile Robot (AMR) for Food Delivery
Description:
    Controls the physical robot inside Gazebo Sim to perform a compact Figure-8 trajectory.
    Records live real-time streams from:
      1. Ground Truth / Reference Path (Mathematical Figure-8 closing at 0, 0, 0)
      2. Raw Wheel Odometry (/wheel/odom)
      3. EKF Filtered Odometry (/odometry/filtered)
    Computes real-world sensor fusion accuracy metrics and writes results to a
    comma-delimited .txt file (CSV format).
"""

import os
import sys
import math
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    """Converts geometry_msgs/Quaternion to yaw angle in radians."""
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


def transform_to_relative(x, y, yaw, x0, y0, yaw0):
    """Transforms global coordinates to relative frame aligned with start pose."""
    dx = x - x0
    dy = y - y0
    rel_x = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    rel_y = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    rel_yaw = normalize_angle(yaw - yaw0)
    return rel_x, rel_y, rel_yaw


class Figure8LiveBenchmark(Node):
    def __init__(self, radius=0.35, velocity=0.18, loops=1, output_file='docs/benchmark_figure8_results.txt'):
        super().__init__(
            'figure8_live_benchmark',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )

        # Compact parameters designed for restaurant corridor / dining aisles:
        # Radius R = 0.35m -> Circle diameter D = 0.70m -> Total lateral span = 1.40m
        self.radius = radius
        self.v = velocity
        self.loops = loops
        self.output_file = output_file

        self.w_turn = self.v / self.radius
        self.t_circle = (2.0 * math.pi) / self.w_turn
        self.t_fig8 = 2.0 * self.t_circle
        self.total_duration = self.t_fig8 * self.loops

        # QoS for sensors & odometry
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for drive commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.sub_raw = self.create_subscription(Odometry, '/wheel/odom', self.raw_odom_cb, qos)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_odom_cb, reliable_qos)
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/odom', self.gt_odom_cb, qos)

        # Current live state buffers
        self.latest_raw = None
        self.latest_ekf = None
        self.latest_gt = None

        # Origin reference poses (t=0)
        self.start_gt = None
        self.start_raw = None
        self.start_ekf = None

        # Ideal mathematical figure-8 integration state
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0

        # Data collection buffers
        self.history = []  # List of dicts

        # Control loop timer (50 Hz = 0.02s)
        self.dt = 0.02
        self.start_time = None
        self.is_running = False
        self.is_completed = False

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f"Figure8LiveBenchmark Node initialized (COMPACT RESTAURANT MODE)!\n"
            f"  - Bán kính vòng R: {self.radius:.2f} m (Đường kính mỗi cánh: {2*self.radius:.2f} m)\n"
            f"  - Tổng bề rộng quỹ đạo: {4*self.radius:.2f} m (An toàn, không chạm tường)\n"
            f"  - Vận tốc di chuyển v: {self.v:.2f} m/s | Vận tốc góc w: +/-{self.w_turn:.2f} rad/s\n"
            f"  - Thời lượng chạy 1 vòng: {self.total_duration:.1f} s ({self.loops} vòng)\n"
            f"  - Đang chờ dữ liệu từ các topic /wheel/odom và /odometry/filtered..."
        )

    def raw_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_raw = (p.x, p.y, yaw)

    def ekf_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_ekf = (p.x, p.y, yaw)

    def gt_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_gt = (p.x, p.y, yaw)

    def control_loop(self):
        if self.is_completed:
            return

        # Check topic availability
        if self.latest_raw is None or self.latest_ekf is None:
            return

        # Fallback for ground truth if ground_truth/odom bridge is still syncing
        if self.latest_gt is None:
            self.latest_gt = self.latest_ekf

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # Wait for simulation clock /clock to start publishing

        # Initialize start references
        if not self.is_running:
            self.start_raw = self.latest_raw
            self.start_ekf = self.latest_ekf
            self.start_gt = self.latest_gt
            self.start_time = now
            self.is_running = True
            self.get_logger().info(">>> Cả 3 nguồn Odometry đã sẵn sàng! BẮT ĐẦU CHẠY QUỸ ĐẠO SỐ 8 THU NHỎ TRONG GAZEBO...")
            return

        # Calculate elapsed simulation time in seconds (synchronized with Gazebo)
        elapsed_sec = (now - self.start_time).nanoseconds / 1e9

        if elapsed_sec >= self.total_duration:
            # Stop the robot immediately
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.is_completed = True
            self.get_logger().info(">>> HOÀN THÀNH QUỸ ĐẠO SỐ 8! Đã dừng robot an toàn.")
            self.save_and_report()
            return

        # Compute Figure-8 velocity profile
        phase_t = elapsed_sec % self.t_fig8
        if phase_t < self.t_circle:
            w_cmd = +self.w_turn  # Circle 1: Left turn (CCW)
        else:
            w_cmd = -self.w_turn  # Circle 2: Right turn (CW)

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

        # Update ideal kinematic reference path (exact closed loop)
        self.ref_yaw = normalize_angle(self.ref_yaw + w_cmd * self.dt)
        self.ref_x += self.v * math.cos(self.ref_yaw) * self.dt
        self.ref_y += self.v * math.sin(self.ref_yaw) * self.dt

        # Compute relative poses aligned with starting point (0, 0, 0)
        raw_rx, raw_ry, raw_ryaw = transform_to_relative(*self.latest_raw, *self.start_raw)
        ekf_rx, ekf_ry, ekf_ryaw = transform_to_relative(*self.latest_ekf, *self.start_ekf)

        # Error metrics relative to the Ideal Reference Target Trajectory
        err_pos_raw = math.hypot(raw_rx - self.ref_x, raw_ry - self.ref_y)
        err_yaw_raw_deg = abs(math.degrees(normalize_angle(raw_ryaw - self.ref_yaw)))

        err_pos_ekf = math.hypot(ekf_rx - self.ref_x, ekf_ry - self.ref_y)
        err_yaw_ekf_deg = abs(math.degrees(normalize_angle(ekf_ryaw - self.ref_yaw)))

        # Append record
        self.history.append({
            't': elapsed_sec,
            'gt_x': self.ref_x, 'gt_y': self.ref_y, 'gt_yaw': self.ref_yaw,
            'raw_x': raw_rx, 'raw_y': raw_ry, 'raw_yaw': raw_ryaw,
            'ekf_x': ekf_rx, 'ekf_y': ekf_ry, 'ekf_yaw': ekf_ryaw,
            'err_pos_raw': err_pos_raw, 'err_yaw_deg_raw': err_yaw_raw_deg,
            'err_pos_ekf': err_pos_ekf, 'err_yaw_deg_ekf': err_yaw_ekf_deg
        })

        # Periodic logging to terminal every 2 seconds
        if len(self.history) % 100 == 0:
            self.get_logger().info(
                f"Progress: [{elapsed_sec:.1f}/{self.total_duration:.1f}s] | "
                f"Err Pos: Raw={err_pos_raw:.3f}m vs EKF={err_pos_ekf:.3f}m | "
                f"Err Yaw: Raw={err_yaw_raw_deg:.1f}° vs EKF={err_yaw_ekf_deg:.1f}°"
            )

    def save_and_report(self):
        if not self.history:
            self.get_logger().warn("Không có dữ liệu thu thập được để lưu!")
            return

        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)

        err_pos_raw_all = [h['err_pos_raw'] for h in self.history]
        err_pos_ekf_all = [h['err_pos_ekf'] for h in self.history]
        err_yaw_raw_all = [h['err_yaw_deg_raw'] for h in self.history]
        err_yaw_ekf_all = [h['err_yaw_deg_ekf'] for h in self.history]

        final_raw_pos = err_pos_raw_all[-1]
        final_ekf_pos = err_pos_ekf_all[-1]
        max_raw_pos = max(err_pos_raw_all)
        max_ekf_pos = max(err_pos_ekf_all)
        rms_raw_pos = math.sqrt(sum(x**2 for x in err_pos_raw_all) / len(err_pos_raw_all))
        rms_ekf_pos = math.sqrt(sum(x**2 for x in err_pos_ekf_all) / len(err_pos_ekf_all))

        final_raw_yaw = err_yaw_raw_all[-1]
        final_ekf_yaw = err_yaw_ekf_all[-1]
        max_raw_yaw = max(err_yaw_raw_all)
        max_ekf_yaw = max(err_yaw_ekf_all)

        impr_pos = (1.0 - final_ekf_pos / final_raw_pos) * 100.0 if final_raw_pos > 0 else 0.0
        impr_yaw = (1.0 - final_ekf_yaw / final_raw_yaw) * 100.0 if final_raw_yaw > 0 else 0.0

        with open(self.output_file, 'w', encoding='utf-8') as f:
            f.write("# =============================================================================\n")
            f.write("# AMR GAZEBO LIVE BENCHMARK LOG: FIGURE-8 TRAJECTORY (EKF VS RAW ODOMETRY)\n")
            f.write("# Recorded from Live Gazebo Sim & ROS 2 Navigation Stack\n")
            f.write("# =============================================================================\n")
            f.write(f"# Trajectory: Compact Figure-8 (Radius R = {self.radius:.2f} m, Linear Velocity v = {self.v:.2f} m/s)\n")
            f.write(f"# Total Simulated Loops: {self.loops} ({len(self.history)} samples at 50 Hz)\n")
            f.write("# Topics: /ground_truth/odom, /wheel/odom, /odometry/filtered, /cmd_vel\n")
            f.write("# -----------------------------------------------------------------------------\n")
            f.write("# SUMMARY BENCHMARK METRICS:\n")
            f.write(f"# Final Loop Closure Position Error : Raw Odom = {final_raw_pos:.4f} m | EKF = {final_ekf_pos:.4f} m (Improvement: {impr_pos:.1f}%)\n")
            f.write(f"# Maximum Position Error (e_max)    : Raw Odom = {max_raw_pos:.4f} m | EKF = {max_ekf_pos:.4f} m\n")
            f.write(f"# Root Mean Square Error (RMSE pos) : Raw Odom = {rms_raw_pos:.4f} m | EKF = {rms_ekf_pos:.4f} m\n")
            f.write(f"# Final Heading Drift (Yaw Error)   : Raw Odom = {final_raw_yaw:.2f} deg | EKF = {final_ekf_yaw:.2f} deg (Improvement: {impr_yaw:.1f}%)\n")
            f.write(f"# Maximum Heading Drift             : Raw Odom = {max_raw_yaw:.2f} deg | EKF = {max_ekf_yaw:.2f} deg\n")
            f.write("# =============================================================================\n")
            f.write("# COLUMNS DEFINITION (COMMA-SEPARATED):\n")
            f.write("timestamp,gt_x,gt_y,gt_yaw,raw_x,raw_y,raw_yaw,ekf_x,ekf_y,ekf_yaw,err_pos_raw,err_yaw_deg_raw,err_pos_ekf,err_yaw_deg_ekf\n")

            for h in self.history:
                f.write(
                    f"{h['t']:.4f},"
                    f"{h['gt_x']:.6f},{h['gt_y']:.6f},{h['gt_yaw']:.6f},"
                    f"{h['raw_x']:.6f},{h['raw_y']:.6f},{h['raw_yaw']:.6f},"
                    f"{h['ekf_x']:.6f},{h['ekf_y']:.6f},{h['ekf_yaw']:.6f},"
                    f"{h['err_pos_raw']:.6f},{h['err_yaw_deg_raw']:.4f},"
                    f"{h['err_pos_ekf']:.6f},{h['err_yaw_deg_ekf']:.4f}\n"
                )

        print("\n" + "=" * 80)
        print("   KẾT QUẢ THỰC NGHIỆM TRÊN GAZEBO THẬT: ĐÁNH GIÁ ĐỘ ỔN ĐỊNH BỘ LỌC EKF")
        print("=" * 80)
        print(f"{'Tiêu chí đánh giá định lượng':<40} | {'Raw Wheel Odom':<16} | {'EKF Filtered':<14}")
        print("-" * 80)
        print(f"{'Sai số đóng vòng cuối (Loop Closure Gap)':<40} | {final_raw_pos:>13.4f} m | {final_ekf_pos:>11.4f} m")
        print(f"{'Sai số vị trí cực đại (Max Pos Error)':<40} | {max_raw_pos:>13.4f} m | {max_ekf_pos:>11.4f} m")
        print(f"{'Sai số bình phương trung bình (RMSE)':<40} | {rms_raw_pos:>13.4f} m | {rms_ekf_pos:>11.4f} m")
        print(f"{'Sai số góc hướng cuối (Final Yaw Drift)':<40} | {final_raw_yaw:>11.2f} deg | {final_ekf_yaw:>9.2f} deg")
        print(f"{'Sai số góc hướng cực đại (Max Yaw Drift)':<40} | {max_raw_yaw:>11.2f} deg | {max_ekf_yaw:>9.2f} deg")
        print("-" * 80)
        print(f"-> Mức độ cải thiện độ chính xác vị trí : {impr_pos:.1f}%")
        print(f"-> Mức độ triệt tiêu trôi dạt góc hướng : {impr_yaw:.1f}%")
        print("=" * 80)
        print(f"[OK] Đã ghi toàn bộ nhật ký ra tệp: {os.path.abspath(self.output_file)}\n")


def main():
    parser = argparse.ArgumentParser(description='Live Gazebo Figure-8 EKF Benchmark Runner')
    parser.add_argument('--radius', type=float, default=0.35,
                        help='Radius of each Figure-8 circle in meters (default: 0.35 m for restaurant corridor)')
    parser.add_argument('--velocity', type=float, default=0.18,
                        help='Linear velocity in m/s (default: 0.18 m/s safe indoor speed)')
    parser.add_argument('--loops', type=int, default=1,
                        help='Number of Figure-8 loops (default: 1)')
    parser.add_argument('--output', default='docs/benchmark_figure8_results.txt',
                        help='Output .txt file path')
    args = parser.parse_args()

    rclpy.init()
    node = Figure8LiveBenchmark(
        radius=args.radius,
        velocity=args.velocity,
        loops=args.loops,
        output_file=args.output
    )

    try:
        while rclpy.ok() and not node.is_completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Đã ngắt bởi người dùng. Đang gửi lệnh dừng xe...")
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
