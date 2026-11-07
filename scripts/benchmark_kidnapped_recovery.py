#!/usr/bin/env python3
"""
AMR Kidnapped Robot Problem Benchmark & Recovery Evaluator
Project: Autonomous Mobile Robot (AMR) for Food Delivery in Restaurant Environment
Platform: ROS 2 Jazzy | Nav2 AMCL & Gazebo Harmonic
Description:
    Evaluates the robot's ability to self-recover its global localization
    after being kidnapped (relocated to an unknown location without odometry feedback).
    
    Enhanced Strategy:
      - Step-and-Pause Active Relocalization: Robot rotates in short pulses (~24 deg) 
        and pauses briefly (0.8s) to allow LiDAR to take motionless, razor-sharp scans.
      - Particle Cloud Covariance & Pose Stabilization Tracking:
        Detects convergence when AMCL particle uncertainty drops below 0.32m
        and estimated pose remains stable.
      - Automatically stops the robot upon recovery and logs results to docs/benchmark_kidnapped_results.txt.
"""

import os
import sys
import math
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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


class KidnappedRecoveryBenchmark(Node):
    def __init__(self, spin_speed=0.35, timeout_sec=45.0, output_file='docs/benchmark_kidnapped_results.txt'):
        super().__init__(
            'kidnapped_recovery_benchmark',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self.spin_speed = spin_speed
        self.timeout_sec = timeout_sec
        self.output_file = output_file

        # QoS Profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher to spin robot in place
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, reliable_qos
        )
        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth/odom', self.gt_cb, best_effort_qos
        )

        # State storage
        self.latest_amcl = None  # (x, y, yaw, cov_trace)
        self.initial_amcl = None
        self.prev_amcl_pos = None
        self.latest_gt = None    # (x, y, yaw)
        self.history = []

        # Benchmark phase tracking
        self.start_sim_time = None
        self.is_completed = False
        self.total_rotated_rad = 0.0

        # Convergence tracking
        self.converged_start_time = None
        self.recovery_time = None
        self.min_rotation_angle_deg = None
        self.initial_kidnap_distance = None

        # Control timer (10 Hz = 0.1s)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"\n{'='*75}\n"
            f"KidnappedRecoveryBenchmark Node ĐÃ KHỞI TẠO THÀNH CÔNG!\n"
            f"  - Cơ chế quét: Step-and-Pause (Quay 1.2s -> Dừng 0.8s để LiDAR quét tĩnh)\n"
            f"  - Vận tốc quay xung: w = {self.spin_speed:.2f} rad/s (~{math.degrees(self.spin_speed):.1f}°/s)\n"
            f"  - Thời gian tối đa (Timeout): {self.timeout_sec:.1f} s\n"
            f"  - Tệp lưu kết quả: {self.output_file}\n"
            f">>> SCRIPT ĐANG PHÁT LỆNH QUAY QUÉT THÔNG MINH ĐỂ AMCL HỘI TỤ...\n"
            f"{'='*75}"
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        # Spatial position uncertainty radius: sqrt(sigma_x^2 + sigma_y^2)
        cov_trace = math.sqrt(max(0.0, cov[0] + cov[7]))
        self.latest_amcl = (p.x, p.y, yaw, cov_trace)
        if self.initial_amcl is None:
            self.initial_amcl = self.latest_amcl

    def gt_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_gt = (p.x, p.y, yaw)

    def control_loop(self):
        if self.is_completed:
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # Wait for simulation clock /clock

        # Initialize start time on first valid sim clock tick
        if self.start_sim_time is None:
            self.start_sim_time = now

        elapsed_sec = (now - self.start_sim_time).nanoseconds / 1e9

        # Timeout guard
        if elapsed_sec >= self.timeout_sec:
            self.stop_robot()
            self.is_completed = True
            self.get_logger().warn(f">>> HẾT THỜI GIAN ({self.timeout_sec}s)! AMCL chưa kịp hội tụ hoàn toàn.")
            self.save_and_report(success=False)
            return

        # ---------------------------------------------------------------------
        # 1. Step-and-Pause Active Relocalization Pattern (Chu kỳ 2.0s):
        #    - Trong 1.2s đầu: Quay một góc ~24 độ
        #    - Trong 0.8s sau: Dừng lại hoàn toàn để LiDAR quét tĩnh siêu nét
        # ---------------------------------------------------------------------
        cycle_t = elapsed_sec % 2.0
        cmd = Twist()
        cmd.linear.x = 0.0

        if cycle_t < 1.2:
            cmd.angular.z = self.spin_speed
            self.total_rotated_rad += self.spin_speed * self.dt
            status_text = "Đang quay xoay góc"
        else:
            cmd.angular.z = 0.0
            status_text = "Tạm dừng quét tĩnh"

        self.cmd_pub.publish(cmd)
        rotated_angle_deg = math.degrees(self.total_rotated_rad)

        # 2. Check if we have received AMCL data
        if self.latest_amcl is None:
            if int(elapsed_sec * 10) % 15 == 0:
                self.get_logger().info(
                    f"[{elapsed_sec:4.1f}s] {status_text}... Đã xoay {rotated_angle_deg:5.1f}°. Chờ AMCL xuất vị trí ước lượng đầu tiên..."
                )
            return

        ax, ay, ayaw, cov_pos = self.latest_amcl

        # Measure stability of estimated position (distance between consecutive samples)
        if self.prev_amcl_pos is not None:
            pos_jitter = math.hypot(ax - self.prev_amcl_pos[0], ay - self.prev_amcl_pos[1])
        else:
            pos_jitter = 0.0
        self.prev_amcl_pos = (ax, ay)

        # Initial kidnap distance from dock
        if self.initial_kidnap_distance is None and self.initial_amcl is not None:
            # How far has AMCL moved from its initial dock estimate?
            self.initial_kidnap_distance = math.hypot(ax - self.initial_amcl[0], ay - self.initial_amcl[1])

        self.history.append({
            't': elapsed_sec,
            'amcl_x': ax, 'amcl_y': ay, 'amcl_yaw': ayaw,
            'cov_pos': cov_pos,
            'pos_jitter': pos_jitter,
            'rotated_deg': rotated_angle_deg
        })

        # Periodic logging every 1 second
        if len(self.history) % 10 == 0:
            self.get_logger().info(
                f"[{elapsed_sec:4.1f}s] {status_text} (Đã quay {rotated_angle_deg:5.1f}° | {rotated_angle_deg/360.0:.2f} vòng) | "
                f"AMCL=({ax:5.2f}, {ay:5.2f}, yaw={math.degrees(ayaw):4.1f}°) | "
                f"Độ bất định (Cov): {cov_pos:4.3f}m | Độ dịch vị trí: {pos_jitter:4.3f}m"
            )

        # ---------------------------------------------------------------------
        # 3. Convergence Detection (Tiêu chí khoa học của AMCL):
        #    - Đám mây hạt co cụm chặt chẽ: cov_pos < 0.32 m
        #    - Tọa độ (x, y) đã ổn định, không còn nhảy cóc: pos_jitter < 0.08 m
        #    - Duy trì vững chắc trong ít nhất 0.8 giây
        # ---------------------------------------------------------------------
        is_converged = (cov_pos < 0.32 and pos_jitter < 0.08 and elapsed_sec > 3.0)

        if is_converged:
            if self.converged_start_time is None:
                self.converged_start_time = now
            else:
                stable_duration = (now - self.converged_start_time).nanoseconds / 1e9
                if stable_duration >= 0.8:  # Held stable for 0.8s
                    self.stop_robot()
                    self.is_completed = True
                    self.recovery_time = elapsed_sec
                    self.min_rotation_angle_deg = rotated_angle_deg
                    self.get_logger().info(
                        f"\n{'='*75}\n"
                        f">>> THÀNH CÔNG RỰC RỠ! AMCL ĐÃ TỰ PHỤC HỒI ĐỊNH VỊ SAU KHI BỊ BẮT CÓC!\n"
                        f"    - Thời gian phục hồi (T_recover): {self.recovery_time:.2f} giây\n"
                        f"    - Góc quay tối thiểu: {self.min_rotation_angle_deg:.1f}° ({self.min_rotation_angle_deg/360.0:.2f} vòng)\n"
                        f"    - Vị trí AMCL chốt được trên bản đồ: X = {ax:.2f} m, Y = {ay:.2f} m, Yaw = {math.degrees(ayaw):.1f}°\n"
                        f"    - Độ tin cậy đám mây hạt (Covariance): {cov_pos:.3f} m\n"
                        f"{'='*75}"
                    )
                    self.save_and_report(success=True)
        else:
            self.converged_start_time = None

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def save_and_report(self, success=True):
        if not self.history:
            return

        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        final_sample = self.history[-1]

        rec_time = self.recovery_time if self.recovery_time else self.history[-1]['t']
        rot_deg = self.min_rotation_angle_deg if self.min_rotation_angle_deg else self.history[-1]['rotated_deg']
        final_x = final_sample['amcl_x']
        final_y = final_sample['amcl_y']
        final_yaw = math.degrees(final_sample['amcl_yaw'])
        final_cov = final_sample['cov_pos']

        with open(self.output_file, 'w', encoding='utf-8') as f:
            f.write("# =============================================================================\n")
            f.write("# AMR KIDNAPPED ROBOT PROBLEM: AMCL LOCALIZATION RECOVERY LOG\n")
            f.write("# Platform: ROS 2 Jazzy | Nav2 AMCL (Adaptive Monte Carlo Localization)\n")
            f.write("# =============================================================================\n")
            f.write(f"# Status: {'RECOVERY SUCCESS (CONVERGED)' if success else 'RECOVERY TIMEOUT'}\n")
            f.write(f"# Recovery Time (T_recover)            : {rec_time:.2f} s\n")
            f.write(f"# Minimum Rotation Angle (Delta Theta) : {rot_deg:.1f} deg ({rot_deg/360.0:.2f} rotations)\n")
            f.write(f"# Recovered Pose on Map (X, Y, Yaw)    : ({final_x:.2f} m, {final_y:.2f} m, {final_yaw:.1f} deg)\n")
            f.write(f"# Particle Cloud Covariance Radius     : {final_cov:.4f} m\n")
            f.write("# -----------------------------------------------------------------------------\n")
            f.write("# COLUMNS DEFINITION (COMMA-SEPARATED):\n")
            f.write("timestamp,amcl_x,amcl_y,amcl_yaw,cov_pos_m,pos_jitter_m,rotated_deg\n")

            for h in self.history:
                f.write(
                    f"{h['t']:.4f},"
                    f"{h['amcl_x']:.6f},{h['amcl_y']:.6f},{h['amcl_yaw']:.6f},"
                    f"{h['cov_pos']:.4f},{h['pos_jitter']:.4f},{h['rotated_deg']:.2f}\n"
                )

        print("\n" + "=" * 75)
        print("          KẾT QUẢ THỰC NGHIỆM: BÀI TOÁN BẮT CÓC ROBOT (AMCL)")
        print("=" * 75)
        print(f"{'Tiêu chí đánh giá định lượng':<42} | {'Kết quả đo đạc':<25}")
        print("-" * 75)
        print(f"{'Trạng thái phục hồi định vị':<42} | {'THÀNH CÔNG (CONVERGED)' if success else 'HẾT GIỜ (TIMEOUT)':<25}")
        print(f"{'Thời gian phục hồi định vị (T_recover)':<42} | {f'{rec_time:.2f} giây':<25}")
        print(f"{'Góc quay tối thiểu cần thiết':<42} | {f'{rot_deg:.1f}° ({rot_deg/360.0:.2f} vòng)':<25}")
        print(f"{'Tọa độ AMCL chốt được trên bản đồ':<42} | {f'({final_x:.2f}m, {final_y:.2f}m)':<25}")
        print(f"{'Góc hướng AMCL chốt được':<42} | {f'{final_yaw:.1f}°':<25}")
        print(f"{'Độ bất định đám mây hạt (Covariance)':<42} | {f'{final_cov*100.0:.1f} cm':<25}")
        print("=" * 75)
        print(f"[OK] Đã ghi toàn bộ dữ liệu ra tệp: {os.path.abspath(self.output_file)}\n")


def main():
    parser = argparse.ArgumentParser(description='AMCL Kidnapped Robot Problem Benchmark')
    parser.add_argument('--speed', type=float, default=0.35, help='In-place spin speed (rad/s, default: 0.35)')
    parser.add_argument('--timeout', type=float, default=45.0, help='Max timeout seconds (default: 45.0)')
    parser.add_argument('--output', default='docs/benchmark_kidnapped_results.txt', help='Output log file path')
    args = parser.parse_args()

    rclpy.init()
    node = KidnappedRecoveryBenchmark(
        spin_speed=args.speed,
        timeout_sec=args.timeout,
        output_file=args.output
    )

    try:
        while rclpy.ok() and not node.is_completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Đã ngắt bởi người dùng. Đang dừng xe...")
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
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
