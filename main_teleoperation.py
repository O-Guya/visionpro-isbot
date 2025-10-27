#!/usr/bin/env python3
"""
Main Teleoperation Script
Vision Pro teleoperation control for Kinova robot arm.

Usage:
    python main_teleoperation.py --vp_ip <Vision_Pro_IP>

Controls:
    - Left hand pinch: Enable/disable control
    - Left hand open: Emergency stop
    - Left hand fist: Fine control mode (slow and precise)
    - Right hand position: Robot end effector position
    - Right hand orientation: Robot end effector orientation
    - Right hand pinch: Gripper control (close/open)
"""

import argparse
import time
import numpy as np
from avp_stream import VisionProStreamer
from kinova_is_sim import KinovaSimulator, KinovaController, SimpleMotionMapper


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Vision Pro to Kinova teleoperation')
    parser.add_argument('--vp_ip', type=str, required=True,
                        help='Vision Pro IP address (e.g., 10.31.181.201)')
    parser.add_argument('--no_gui', action='store_true',
                        help='Run without GUI (faster)')
    parser.add_argument('--position_scale', type=float, default=2.5,
                        help='Position scaling factor (default: 2.5, higher=robot moves more)')
    parser.add_argument('--workspace_offset', type=float, nargs=3,
                        default=[0.5, 0.3, 0.5],
                        help='Workspace offset [x y z] (default: 0.5 0.3 0.5)')
    parser.add_argument('--kinova_urdf', type=str, default=None,
                        help='Path to Kinova URDF file (optional)')
    parser.add_argument('--control_freq', type=float, default=100,
                        help='Control loop frequency in Hz (default: 100)')
    parser.add_argument('--no_head_relative', action='store_true',
                        help='Disable head-relative control mode')
    args = parser.parse_args()

    print("="*60)
    print("Vision Pro to Kinova Teleoperation")
    print("="*60)
    print(f"Vision Pro IP: {args.vp_ip}")
    print(f"Position scale: {args.position_scale}")
    print(f"Workspace offset: {args.workspace_offset}")
    print(f"Control frequency: {args.control_freq} Hz")
    print(f"Head-relative mode: {'OFF' if args.no_head_relative else 'ON'}")
    print(f"GUI mode: {'OFF' if args.no_gui else 'ON'}")
    print("="*60)

    # Initialize Vision Pro streamer
    print("\nConnecting to Vision Pro...")
    try:
        vp_streamer = VisionProStreamer(ip=args.vp_ip, record=False)
        print("✓ Connected to Vision Pro")
    except Exception as e:
        print(f"✗ Failed to connect to Vision Pro: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure Vision Pro is on the same network")
        print("2. Check the IP address is correct")
        print("3. Ensure the Tracking Streamer app is running on Vision Pro")
        return

    # Wait for initial data
    print("\nWaiting for Vision Pro data...")
    max_wait = 5.0
    start_time = time.time()
    while True:
        data = vp_streamer.latest
        if data is not None and 'right_wrist' in data:
            print("✓ Receiving Vision Pro data")
            break
        if time.time() - start_time > max_wait:
            print("✗ Timeout waiting for Vision Pro data")
            return
        time.sleep(0.1)

    # Initialize simulator
    print("\nInitializing PyBullet simulator...")
    sim = KinovaSimulator(gui=not args.no_gui, kinova_urdf_path=args.kinova_urdf)
    print("✓ Simulator initialized")

    # Initialize controller
    print("\nInitializing Kinova controller...")
    robot_id = sim.get_robot_id()
    ee_link_index = sim.get_end_effector_link_index()
    controller = KinovaController(robot_id, ee_link_index, num_joints=7)
    print("✓ Controller initialized")

    # Initialize motion mapper
    print("\nInitializing motion mapper...")
    mapper = SimpleMotionMapper(
        position_scale=args.position_scale,
        workspace_offset=np.array(args.workspace_offset),
        use_head_relative=not args.no_head_relative
    )
    print("✓ Motion mapper initialized")
    if not args.no_head_relative:
        print("  Head-relative control mode enabled")

    print("\n" + "="*60)
    print("READY FOR TELEOPERATION")
    print("="*60)
    print("\nControls:")
    print("  • Left hand PINCH: Enable/disable control")
    print("  • Left hand OPEN: Emergency stop")
    print("  • Left hand FIST: Fine control mode")
    print("  • Right hand POSITION: Control robot position")
    print("  • Right hand ORIENTATION: Control robot orientation")
    print("  • Right hand PINCH: Control gripper")
    print("\nPress Ctrl+C to exit")
    print("="*60 + "\n")

    # Teleoperation loop
    loop_rate = args.control_freq  # Hz (configurable)
    dt = 1.0 / loop_rate
    emergency_stopped = False

    try:
        while True:
            loop_start = time.time()

            # Get Vision Pro data
            vp_data = vp_streamer.latest

            if vp_data is None:
                print("Warning: No Vision Pro data")
                time.sleep(dt)
                continue

            # Map to robot command
            robot_cmd = mapper.map_to_robot(vp_data)

            # Handle emergency stop
            if robot_cmd is not None and robot_cmd.get("emergency_stop", False):
                if not emergency_stopped:
                    controller.stop()
                    emergency_stopped = True
                    print("⚠️  EMERGENCY STOP ACTIVATED")
                time.sleep(dt)
                continue
            else:
                if emergency_stopped:
                    emergency_stopped = False
                    print("✓ Emergency stop released")

            # Execute robot command
            if robot_cmd is not None:
                success = controller.move_to_pose(
                    robot_cmd["position"],
                    robot_cmd["orientation"],
                    speed_scale=robot_cmd["speed_scale"]
                )

                if not success:
                    print("Warning: IK failed for target pose")

                # Control gripper
                controller.control_gripper(robot_cmd["gripper"])

                # Optional: Add debug visualization
                if not args.no_gui:
                    # Draw target position
                    sim.add_debug_axes(
                        robot_cmd["position"],
                        robot_cmd["orientation"],
                        length=0.1
                    )
                    # Update gripper visualization
                    sim.update_gripper_visualization(robot_cmd["gripper"])

            # Step simulation
            sim.step()

            # Maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        # Cleanup
        print("Closing simulator...")
        sim.close()
        print("Teleoperation ended.")


if __name__ == "__main__":
    main()
