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
    print(" RESTAURANT DESTINATION AND TABLE CATALOG")
    print("=" * 90)
    print(f"{'ID':<4} | {'Code':<14} | {'Table Name':<32} | {'Pose (X, Y, Yaw)':<24} | {'Fallback'}")
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
    print("💡 Usage:")
    print("   ros2 run amr_navigation dispatch_table <ID>       (Example: dispatch_table 12)")
    print("   ros2 run amr_navigation dispatch_table 0          (Return to Kitchen / Home)")
    print("   ros2 run amr_navigation dispatch_table --list     (Show this table)\n")


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
    name = target_info.get('name', f'Table {target_id}')
    group_name = target_info.get('group_name', 'Cluster')

    if not is_fallback:
        print(f"\n🚀 [PRIMARY DISPATCH] Departing for Table {target_id}: '{name}'")
        print(f"   📍 Docking pose: X = {x:+.2f}m, Y = {y:+.2f}m, Yaw = {math.degrees(yaw):+.1f}° | Group: {group_name}")
    else:
        print(f"\n🔄 [FALLBACK ACTIVATED] Table {primary_id} is blocked! Trying an alternative in the same cluster -> Table {target_id}: '{name}'")
        print(f"   📍 Fallback docking pose: X = {x:+.2f}m, Y = {y:+.2f}m, Yaw = {math.degrees(yaw):+.1f}°")

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
            print(f"   ⏳ Moving... Distance remaining: {dist:.2f}m | Travel time: {elapsed}s")
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
        help="Destination table ID (0..n)"
    )
    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help="List all tables and destinations"
    )
    parser.add_argument(
        '--home',
        action='store_true',
        help="Return the robot to the Kitchen Station (ID: 0)"
    )
    parser.add_argument(
        '-w', '--wait',
        type=float,
        default=None,
        help="Override the waiting time at the table (seconds)"
    )
    parser.add_argument(
        '-c', '--config',
        type=str,
        default=None,
        help="Custom path to restaurant_tables.yaml"
    )

    parsed_args = parser.parse_args(args)

    # 1. Load configuration
    try:
        config_data, config_filepath = load_tables_config(parsed_args.config)
        tables = config_data.get('tables', {})
        settings = config_data.get('settings', {})
    except Exception as e:
        print(f"❌ [CONFIGURATION ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    # 2. Handle --list flag
    if parsed_args.list:
        print_tables_catalog(tables)
        print(f"📁 Configuration file in use: {config_filepath}\n")
        return 0

    # 3. Determine target table ID
    target_id = None
    if parsed_args.home:
        target_id = settings.get('home_table_id', 0)
    elif parsed_args.table_id is not None:
        target_id = parsed_args.table_id
    else:
        print("❌ [ARGUMENT ERROR] No destination table ID was provided!")
        print_tables_catalog(tables)
        sys.exit(1)

    if target_id not in tables:
        print(f"❌ [ERROR] Table ID '{target_id}' does not exist in the catalog!")
        print(f"   Available valid IDs: {sorted(list(tables.keys()))}")
        sys.exit(1)

    # 4. Initialize ROS 2 and Nav2 BasicNavigator
    print("=" * 80)
    print("🤖 AMR FOOD DELIVERY MISSION DISPATCHER")
    print(f"📁 Loading configuration: {config_filepath}")
    print("=" * 80)

    rclpy.init()
    navigator = BasicNavigator()

    try:
        print("🔍 Checking Nav2 stack status (waiting for navigator readiness)...")
        # Ensure BasicNavigator respects current localization and NEVER forces a (0,0) initial pose
        navigator.initial_pose_received = True
        navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')
        print(" Nav2 stack is ready and active!\n")

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
                print("\n⛔ [CANCELED] The navigation task was canceled by the user.")
                return 1
            else:
                # FAILED
                print(f"❌ [FAILED] Robot could not reach Table {cid} (The path is blocked by obstacles).")
                if is_fallback:
                    print(f"   -> Searching for another fallback position in the cluster...")

        # 5. Mission Outcome Handling
        if mission_succeeded:
            delivered_table = tables[final_table_id]
            tname = delivered_table.get('name', '')
            gname = delivered_table.get('group_name', '')

            print("\n" + "*" * 80)
            if final_table_id == target_id:
                print(f"🎉 [DELIVERY SUCCEEDED] Robot docked successfully at Table {final_table_id}: '{tname}'!")
            else:
                print(f"🎉 [FALLBACK DELIVERY SUCCEEDED] Docked safely at Table {final_table_id}: '{tname}' ({gname})!")
                print(f"   📢 Notice: The original destination table ({target_id}) is blocked. Guests at table {target_id}")
                print(f"      can collect their food at a nearby position (~1.0m - 1.5m).")
            print("*" * 80)

            # Wait / Standby countdown at table
            wait_time = parsed_args.wait if parsed_args.wait is not None else delivered_table.get('wait_time_sec', 10.0)
            if wait_time > 0 and final_table_id != 0:
                print(f"\n🍽️ [SERVICE STATUS] Robot is waiting for food collection for {wait_time:.0f} seconds...")
                remaining_wait = int(wait_time)
                while remaining_wait > 0:
                    print(f"   ⏱️ Countdown: {remaining_wait}s remaining...", end='\r')
                    time.sleep(1.0)
                    remaining_wait -= 1
                print("\n Table service waiting time completed!\n")
            elif final_table_id == 0:
                print("🏠 [HOME STATION STATUS] Robot is safely docked at the Kitchen / Charging Station.")

            return 0

        else:
            # All candidates in cluster failed
            primary_group_name = primary_table.get('group_name', 'Cluster')
            print("\n" + "!" * 80)
            print(f"❌ [ALERT] All tables in '{primary_group_name}' are blocked!")
            print(f"   No feasible docking position allows safe access to this table cluster.")
            print("!" * 80)

            if settings.get('auto_return_home_on_all_failed', True) and target_id != 0:
                print("\n🔄 [PROTECTIVE ACTION] Automatically returning the robot to the Kitchen (Home Station) to clear the aisle...")
                home_id = settings.get('home_table_id', 0)
                if home_id in tables:
                    home_info = tables[home_id]
                    navigate_to_single_pose(navigator, home_id, home_info, is_fallback=False)
            return 2

    except KeyboardInterrupt:
        print("\n\n⚠️ [EMERGENCY INTERRUPT] KeyboardInterrupt received! Canceling navigation...")
        navigator.cancelTask()
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
