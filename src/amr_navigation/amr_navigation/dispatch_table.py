#!/usr/bin/env python3
"""
Table Dispatcher & Autonomous Food Delivery Navigator for AMR
Project: Autonomous Mobile Robot (AMR) - Restaurant Environment (Pizza 4P's)
Platform: ROS 2 Jazzy | Nav2 (SmacPlanner2D + DWB Controller)

Features:
- Dispatches robot to predefined table standoff positions via CLI table ID (0..n).
- Implements Cluster/Group Fallback Recovery: If the target table is blocked,
  automatically attempts alternative tables within the same cluster.
- Displays real-time navigation metrics (distance remaining, ETA).
- Provides auto-standby countdown at table for food retrieval.
"""

import os
import sys
import time
import math
import argparse
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def load_tables_config(custom_path=None):
    """Load tables configuration YAML with multi-level path fallback."""
    search_paths = []
    if custom_path:
        search_paths.append(custom_path)

    # 1. Package share directory
    try:
        pkg_share = get_package_share_directory('amr_navigation')
        search_paths.append(os.path.join(pkg_share, 'config', 'restaurant_tables.yaml'))
    except PackageNotFoundError:
        pass

    # 2. Relative to this script in workspace source tree
    current_dir = os.path.dirname(os.path.abspath(__file__))
    search_paths.append(os.path.join(current_dir, '..', 'config', 'restaurant_tables.yaml'))
    search_paths.append(os.path.join(current_dir, '..', '..', 'config', 'restaurant_tables.yaml'))

    for p in search_paths:
        if os.path.exists(p):
            with open(p, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data, p

    raise FileNotFoundError(
        f"Could not find configuration file restaurant_tables.yaml. Searched at:\n" +
        "\n".join(search_paths)
    )


def print_tables_catalog(tables_dict):
    """Print a formatted ASCII table of all available tables and clusters."""
    print("\n" + "=" * 90)
    print(" DANH MỤC CÁC ĐIỂM ĐẾN & BÀN ĂN TRONG NHÀ HÀNG (RESTAURANT TABLE CATALOG)")
    print("=" * 90)
    print(f"{'ID':<4} | {'Mã Code':<14} | {'Tên Bàn':<32} | {'Tọa độ (X, Y, Yaw)':<24} | {'Dự phòng'}")
    print("-" * 90)
    for tid in sorted(tables_dict.keys(), key=lambda x: int(x)):
        t = tables_dict[tid]
        pose = t.get('pose', {})
        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        yaw = pose.get('yaw', 0.0)
        pose_str = f"({x:+.2f}, {y:+.2f}, {yaw:+.2f})"
        fallbacks = str(t.get('fallback_table_ids', []))
        print(f"{tid:<4} | {t.get('code', ''):<14} | {t.get('name', ''):<32} | {pose_str:<24} | {fallbacks}")
    print("=" * 90)
    print("💡 Hướng dẫn sử dụng:")
    print("   ros2 run amr_navigation dispatch_table <ID>       (Ví dụ: dispatch_table 12)")
    print("   ros2 run amr_navigation dispatch_table 0          (Quay về trạm Bếp / Home)")
    print("   ros2 run amr_navigation dispatch_table --list     (Hiển thị bảng này)\n")


def create_pose_stamped(navigator, x, y, yaw, frame_id='map'):
    """Create a PoseStamped message with heading orientation quaternion."""
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0

    # Planar 2D Yaw to Quaternion (qz = sin(yaw/2), qw = cos(yaw/2))
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    goal_pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return goal_pose


def navigate_to_single_pose(navigator, target_id, target_info, is_fallback=False, primary_id=None):
    """
    Attempt navigation to a single target pose.
    Returns:
        TaskResult (SUCCEEDED, FAILED, CANCELED)
    """
    pose_dict = target_info['pose']
    x = pose_dict['x']
    y = pose_dict['y']
    yaw = pose_dict['yaw']
    name = target_info.get('name', f'Bàn {target_id}')
    group_name = target_info.get('group_name', 'Cụm')

    if not is_fallback:
        print(f"\n🚀 [ĐIỀU PHỐI CHÍNH] Xuất phát tới Bàn {target_id}: '{name}'")
        print(f"   📍 Vị trí đỗ: X = {x:+.2f}m, Y = {y:+.2f}m, Yaw = {math.degrees(yaw):+.1f}° | Thuộc: {group_name}")
    else:
        print(f"\n🔄 [KÍCH HOẠT DỰ PHÒNG] Bàn {primary_id} bị cản trở! Thử vị trí thay thế cùng cụm -> Bàn {target_id}: '{name}'")
        print(f"   📍 Vị trí đỗ dự phòng: X = {x:+.2f}m, Y = {y:+.2f}m, Yaw = {math.degrees(yaw):+.1f}°")

    goal_pose = create_pose_stamped(navigator, x, y, yaw)
    navigator.goToPose(goal_pose)

    last_feedback_time = 0.0
    feedback_interval = 2.0  # seconds

    while not navigator.isTaskComplete():
        current_time = time.time()
        feedback = navigator.getFeedback()
        if feedback and (current_time - last_feedback_time >= feedback_interval):
            dist = feedback.distance_remaining
            elapsed = feedback.navigation_time.sec if hasattr(feedback, 'navigation_time') else 0
            print(f"   ⏳ Đang di chuyển... Còn cách đích: {dist:.2f}m | Thời gian hành trình: {elapsed}s")
            last_feedback_time = current_time
        time.sleep(0.2)

    return navigator.getResult()


def main(args=None):
    parser = argparse.ArgumentParser(
        description="AMR Food Delivery Table Dispatcher with Cluster Fallback"
    )
    parser.add_argument(
        'table_id',
        type=int,
        nargs='?',
        default=None,
        help="ID bàn ăn cần điều phối robot tới (0..n)"
    )
    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help="Liệt kê danh mục tất cả các bàn ăn và điểm đến"
    )
    parser.add_argument(
        '--home',
        action='store_true',
        help="Lệnh nhanh đưa robot quay về Trạm Bếp (ID: 0)"
    )
    parser.add_argument(
        '-w', '--wait',
        type=float,
        default=None,
        help="Ghi đè thời gian chờ đỗ tại bàn (giây)"
    )
    parser.add_argument(
        '-c', '--config',
        type=str,
        default=None,
        help="Đường dẫn tùy chỉnh tới tệp restaurant_tables.yaml"
    )

    parsed_args = parser.parse_args(args)

    # 1. Load configuration
    try:
        config_data, config_filepath = load_tables_config(parsed_args.config)
        tables = config_data.get('tables', {})
        settings = config_data.get('settings', {})
    except Exception as e:
        print(f"❌ [LỖI CẤU HÌNH] {e}", file=sys.stderr)
        sys.exit(1)

    # 2. Handle --list flag
    if parsed_args.list:
        print_tables_catalog(tables)
        print(f"📁 Tệp cấu hình đang sử dụng: {config_filepath}\n")
        return 0

    # 3. Determine target table ID
    target_id = None
    if parsed_args.home:
        target_id = settings.get('home_table_id', 0)
    elif parsed_args.table_id is not None:
        target_id = parsed_args.table_id
    else:
        print("❌ [LỖI THAM SỐ] Bạn chưa cung cấp ID bàn cần đến!")
        print_tables_catalog(tables)
        sys.exit(1)

    if target_id not in tables:
        print(f"❌ [LỖI] ID Bàn '{target_id}' không tồn tại trong danh mục!")
        print(f"   Các ID hợp lệ hiện có: {sorted(list(tables.keys()))}")
        sys.exit(1)

    # 4. Initialize ROS 2 and Nav2 BasicNavigator
    print("=" * 80)
    print("🤖 AMR FOOD DELIVERY MISSION DISPATCHER")
    print(f"📁 Nạp cấu hình: {config_filepath}")
    print("=" * 80)

    rclpy.init()
    navigator = BasicNavigator()

    try:
        print("🔍 Đang kiểm tra trạng thái của Nav2 Stack (chờ navigator sẵn sàng)...")
        # Ensure BasicNavigator respects current localization and NEVER forces a (0,0) initial pose
        navigator.initial_pose_received = True
        navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')
        print(" Nav2 Stack đã sẵn sàng và đang hoạt động tốt!\n")

        primary_table = tables[target_id]
        fallback_ids = primary_table.get('fallback_table_ids', [])

        # Candidate list: Primary table first, followed by fallbacks
        candidates = [target_id] + [fid for fid in fallback_ids if fid in tables]

        mission_succeeded = False
        final_table_id = None

        for idx, cid in enumerate(candidates):
            candidate_info = tables[cid]
            is_fallback = (idx > 0)

            result = navigate_to_single_pose(
                navigator=navigator,
                target_id=cid,
                target_info=candidate_info,
                is_fallback=is_fallback,
                primary_id=target_id
            )

            if result == TaskResult.SUCCEEDED:
                mission_succeeded = True
                final_table_id = cid
                break
            elif result == TaskResult.CANCELED:
                print("\n⛔ [ĐÃ HỦY] Nhiệm vụ điều hướng đã bị hủy bởi người dùng.")
                return 1
            else:
                # FAILED
                print(f"❌ [THẤT BẠI] Robot không thể tiếp cận Bàn {cid} (Đường đi bị vật cản chặn kín).")
                if is_fallback:
                    print(f"   -> Tiếp tục tìm kiếm vị trí thay thế khác trong cụm...")

        # 5. Mission Outcome Handling
        if mission_succeeded:
            delivered_table = tables[final_table_id]
            tname = delivered_table.get('name', '')
            gname = delivered_table.get('group_name', '')

            print("\n" + "*" * 80)
            if final_table_id == target_id:
                print(f"🎉 [GIAO HÀNG THÀNH CÔNG] Robot đã cập đỗ chính xác tại Bàn {final_table_id}: '{tname}'!")
            else:
                print(f"🎉 [GIAO HÀNG DỰ PHÒNG THÀNH CÔNG] Đã cập đỗ an toàn tại Bàn {final_table_id}: '{tname}' ({gname})!")
                print(f"   📢 Thông báo: Bàn mục tiêu ban đầu ({target_id}) bị cản trở. Quý khách tại bàn {target_id}")
                print(f"      có thể nhận đồ ăn tại vị trí lân cận (~1.0m - 1.5m).")
            print("*" * 80)

            # Wait / Standby countdown at table
            wait_time = parsed_args.wait if parsed_args.wait is not None else delivered_table.get('wait_time_sec', 10.0)
            if wait_time > 0 and final_table_id != 0:
                print(f"\n🍽️ [TRẠNG THÁI PHỤC VỤ] Robot đang đỗ chờ quý khách nhận món ăn trong {wait_time:.0f} giây...")
                remaining_wait = int(wait_time)
                while remaining_wait > 0:
                    print(f"   ⏱️ Đếm ngược: {remaining_wait}s còn lại...", end='\r')
                    time.sleep(1.0)
                    remaining_wait -= 1
                print("\n Hoàn tất thời gian phục vụ tại bàn!\n")
            elif final_table_id == 0:
                print("🏠 [VỀ TRẠNG THÁI TRẠM GỐC] Robot đã đỗ an toàn tại Khu Bếp / Trạm Sạc.")

            return 0

        else:
            # All candidates in cluster failed
            primary_group_name = primary_table.get('group_name', 'Cụm')
            print("\n" + "!" * 80)
            print(f"❌ [BÁO ĐỘNG] Toàn bộ các bàn trong '{primary_group_name}' đều bị phong tỏa!")
            print(f"   Không có vị trí đỗ nào khả thi để tiếp cận cụm bàn này an toàn.")
            print("!" * 80)

            if settings.get('auto_return_home_on_all_failed', True) and target_id != 0:
                print("\n🔄 [CƠ CHẾ BẢO VỆ] Tự động điều phối robot quay về Trạm Bếp (Home Station) để giải phóng hành lang...")
                home_id = settings.get('home_table_id', 0)
                if home_id in tables:
                    home_info = tables[home_id]
                    navigate_to_single_pose(navigator, home_id, home_info, is_fallback=False)
            return 2

    except KeyboardInterrupt:
        print("\n\n⚠️ [NGẮT KHẨN CẤP] Nhận tín hiệu KeyboardInterrupt! Đang hủy lệnh điều hướng...")
        navigator.cancelTask()
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
