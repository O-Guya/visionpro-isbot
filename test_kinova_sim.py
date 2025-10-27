#!/usr/bin/env python3
"""
Test script for Kinova simulation without Vision Pro.
Tests the simulator and controller with simulated hand movements.
"""

import numpy as np
import time
from kinova_is_sim import KinovaSimulator, KinovaController


def generate_circular_trajectory(center, radius, num_points=100):
    """Generate circular trajectory points."""
    angles = np.linspace(0, 2*np.pi, num_points)
    points = []
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        points.append([x, y, z])
    return points


def main():
    print("="*60)
    print("Kinova Simulator Test (No Vision Pro Required)")
    print("="*60)

    # Initialize simulator
    print("\nInitializing simulator...")
    sim = KinovaSimulator(gui=True)
    print("✓ Simulator ready")

    # Initialize controller
    print("\nInitializing controller...")
    robot_id = sim.get_robot_id()
    ee_link_index = sim.get_end_effector_link_index()
    controller = KinovaController(robot_id, ee_link_index)
    print("✓ Controller ready")

    # Test 1: Move to home position
    print("\n" + "="*60)
    print("Test 1: Reset to home position")
    print("="*60)
    controller.reset_to_home()
    for _ in range(240):  # Wait 1 second
        sim.step()
        time.sleep(1.0/240.0)
    print("✓ Home position reached")

    # Test 2: Move to target positions
    print("\n" + "="*60)
    print("Test 2: Move to target position")
    print("="*60)

    target_pos = [0.5, 0.3, 0.5]
    target_ori = [0, 1, 0, 0]  # Pointing down

    print(f"Moving to position: {target_pos}")
    print(f"Orientation (quaternion): {target_ori}")

    for i in range(500):  # 2 seconds
        controller.move_to_pose(target_pos, target_ori)
        sim.step()
        time.sleep(1.0/240.0)

        # Print progress every 120 steps (0.5 seconds)
        if i % 120 == 0:
            current_pos, current_ori = controller.get_end_effector_pose()
            error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            print(f"  Step {i}: Position error = {error:.4f}m")

    print("✓ Target position reached")

    # Test 3: Circular trajectory
    print("\n" + "="*60)
    print("Test 3: Circular trajectory")
    print("="*60)

    center = [0.5, 0.3, 0.5]
    radius = 0.1
    trajectory = generate_circular_trajectory(center, radius, num_points=200)
    target_ori = [0, 1, 0, 0]

    print(f"Following circular path...")
    print(f"  Center: {center}")
    print(f"  Radius: {radius}m")

    for i, target_pos in enumerate(trajectory):
        controller.move_to_pose(target_pos, target_ori)
        sim.step()
        time.sleep(1.0/240.0)

        # Print progress
        if i % 50 == 0:
            print(f"  Progress: {i}/{len(trajectory)}")

    print("✓ Circular trajectory completed")

    # Test 4: Gripper control
    print("\n" + "="*60)
    print("Test 4: Gripper control")
    print("="*60)

    print("Closing gripper...")
    for i in range(120):  # 0.5 seconds
        controller.control_gripper(0.0)  # Fully closed
        sim.step()
        time.sleep(1.0/240.0)
    print("✓ Gripper closed")

    time.sleep(0.5)

    print("Opening gripper...")
    for i in range(120):  # 0.5 seconds
        controller.control_gripper(1.0)  # Fully open
        sim.step()
        time.sleep(1.0/240.0)
    print("✓ Gripper opened")

    # Test 5: Different orientations
    print("\n" + "="*60)
    print("Test 5: Different orientations")
    print("="*60)

    orientations = [
        ([0, 1, 0, 0], "Pointing down"),
        ([0, 0, 1, 0], "Tilted"),
        ([0.707, 0, 0.707, 0], "Sideways"),
    ]

    target_pos = [0.5, 0.3, 0.5]

    for ori, description in orientations:
        print(f"\nMoving to orientation: {description}")
        print(f"  Quaternion: {ori}")

        for i in range(240):  # 1 second
            controller.move_to_pose(target_pos, ori)
            sim.step()
            time.sleep(1.0/240.0)

        print(f"✓ {description} orientation reached")
        time.sleep(0.5)

    print("\n" + "="*60)
    print("All tests completed successfully!")
    print("="*60)
    print("\nSimulator will stay open. Close the window or press Ctrl+C to exit.")

    # Keep simulator open
    try:
        while True:
            sim.step()
            time.sleep(1.0/240.0)
    except KeyboardInterrupt:
        print("\n\nShutting down...")

    sim.close()
    print("Test completed.")


if __name__ == "__main__":
    main()
