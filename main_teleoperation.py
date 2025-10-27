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
import threading
from avp_stream import VisionProStreamer
from kinova_is_sim import KinovaSimulator, KinovaController, SimpleMotionMapper


def calibrate_workspace(vp_streamer, mapper):
    """
    6-point calibration: record hand workspace boundaries.
    """
    print("\n" + "="*60)
    print("WORKSPACE CALIBRATION (6 points)")
    print("="*60)

    calibration_points = [
        ("x_max", "Move hand to FRONT (maximum forward), press Enter"),
        ("x_min", "Move hand to BACK (maximum backward), press Enter"),
        ("y_max", "Move hand to RIGHT (maximum right), press Enter"),
        ("y_min", "Move hand to LEFT (maximum left), press Enter"),
        ("z_max", "Move hand to TOP (maximum up), press Enter"),
        ("z_min", "Move hand to BOTTOM (maximum down), press Enter"),
    ]

    for point_name, instruction in calibration_points:
        print(f"\n{instruction}")
        input()
        data = vp_streamer.latest
        hand_pos = data['right_wrist'][0, :3, 3]
        mapper.set_calibration_point(point_name, hand_pos)
        print(f"✓ Recorded {point_name}: {hand_pos}")

    mapper.finish_calibration()
    print("\n" + "="*60 + "\n")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Vision Pro to Kinova teleoperation with calibration')
    parser.add_argument('--vp_ip', type=str, required=True,
                        help='Vision Pro IP address (e.g., 10.31.181.201)')
    parser.add_argument('--no_gui', action='store_true',
                        help='Run without GUI')
    parser.add_argument('--kinova_urdf', type=str, default=None,
                        help='Path to Kinova URDF file (optional)')
    parser.add_argument('--control_freq', type=float, default=100,
                        help='Control loop frequency in Hz (default: 100)')
    args = parser.parse_args()

    print("="*60)
    print("Vision Pro to Kinova Teleoperation (Calibration Mode)")
    print("="*60)
    print(f"Vision Pro IP: {args.vp_ip}")
    print(f"Control frequency: {args.control_freq} Hz")
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
    mapper = SimpleMotionMapper()
    print("✓ Motion mapper initialized")

    # Run calibration
    calibrate_workspace(vp_streamer, mapper)

    print("\n" + "="*60)
    print("KEYBOARD CONTROLS")
    print("="*60)
    print("  [Space] - Enable/Disable control")
    print("  [F]     - Toggle fine control mode")
    print("  [Q/Esc] - Emergency stop & exit")
    print("  [C]     - Re-calibrate workspace")
    print("\n  Right hand: Control robot position/orientation")
    print("  Right pinch: Control gripper")
    print("="*60 + "\n")

    # Keyboard state
    keyboard_state = {'exit': False, 'recalibrate': False}

    def keyboard_listener():
        """Simple keyboard input handler in separate thread."""
        while not keyboard_state['exit']:
            try:
                key = input()  # Blocking, but in separate thread
                key = key.lower().strip()

                if key in ['q', 'esc']:
                    keyboard_state['exit'] = True
                    print("\n[Q] Emergency stop & exit")
                elif key == ' ' or key == 'space':
                    mapper.control_enabled = not mapper.control_enabled
                    print(f"\n[Space] Control {'ENABLED' if mapper.control_enabled else 'DISABLED'}")
                elif key == 'f':
                    mapper.fine_control_mode = not mapper.fine_control_mode
                    print(f"\n[F] Fine control mode {'ON' if mapper.fine_control_mode else 'OFF'}")
                elif key == 'c':
                    keyboard_state['recalibrate'] = True
                    print("\n[C] Re-calibration requested")
            except:
                pass

    # Start keyboard listener thread
    kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kb_thread.start()

    # Teleoperation loop
    loop_rate = args.control_freq
    dt = 1.0 / loop_rate

    try:
        while not keyboard_state['exit']:
            loop_start = time.time()

            # Handle re-calibration request
            if keyboard_state['recalibrate']:
                mapper.control_enabled = False
                calibrate_workspace(vp_streamer, mapper)
                keyboard_state['recalibrate'] = False
                continue

            # Get Vision Pro data
            vp_data = vp_streamer.latest
            if vp_data is None:
                time.sleep(dt)
                continue

            # Map to robot command
            robot_cmd = mapper.map_to_robot(vp_data)

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
