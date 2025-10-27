"""
Simple Motion Mapper for Vision Pro to Kinova teleoperation.
Maps Vision Pro hand tracking data to Kinova robot commands.
"""

import numpy as np
from scipy.spatial.transform import Rotation


class SimpleMotionMapper:
    """Simple direct pose mapping from Vision Pro to Kinova robot."""

    def __init__(self, robot_workspace_min=None, robot_workspace_max=None):
        """
        Initialize the motion mapper with calibration support.

        Args:
            robot_workspace_min: Robot workspace minimum [x, y, z] (default: [0.2, -0.3, 0.2])
            robot_workspace_max: Robot workspace maximum [x, y, z] (default: [0.8, 0.3, 0.8])
        """
        self.control_enabled = False
        self.fine_control_mode = False

        # Calibration (hand workspace boundaries)
        self.hand_min = None  # Will be set during calibration
        self.hand_max = None
        self.calibrated = False

        # Robot workspace
        self.robot_min = robot_workspace_min if robot_workspace_min is not None else np.array([0.2, -0.3, 0.2])
        self.robot_max = robot_workspace_max if robot_workspace_max is not None else np.array([0.8, 0.3, 0.8])

    def set_calibration_point(self, point_name, hand_position):
        """
        Set a calibration boundary point.

        Args:
            point_name: 'x_max', 'x_min', 'y_max', 'y_min', 'z_max', 'z_min'
            hand_position: [x, y, z] hand position at boundary
        """
        if self.hand_min is None:
            self.hand_min = hand_position.copy()
        if self.hand_max is None:
            self.hand_max = hand_position.copy()

        if 'x_max' in point_name:
            self.hand_max[0] = hand_position[0]
        elif 'x_min' in point_name:
            self.hand_min[0] = hand_position[0]
        elif 'y_max' in point_name:
            self.hand_max[1] = hand_position[1]
        elif 'y_min' in point_name:
            self.hand_min[1] = hand_position[1]
        elif 'z_max' in point_name:
            self.hand_max[2] = hand_position[2]
        elif 'z_min' in point_name:
            self.hand_min[2] = hand_position[2]

    def finish_calibration(self):
        """Mark calibration as complete."""
        if self.hand_min is not None and self.hand_max is not None:
            self.calibrated = True
            print(f"✓ Calibration complete")
            print(f"  Hand workspace: {self.hand_min} to {self.hand_max}")
            print(f"  Robot workspace: {self.robot_min} to {self.robot_max}")
        else:
            print("✗ Calibration failed: boundaries not set")

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        Convert 3x3 rotation matrix to quaternion [x, y, z, w].

        Args:
            rotation_matrix: (3, 3) rotation matrix

        Returns:
            np.array: Quaternion [x, y, z, w]
        """
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()  # Returns [x, y, z, w]
        return quat

    def rotation_matrix_to_euler(self, rotation_matrix):
        """
        Convert 3x3 rotation matrix to Euler angles (roll, pitch, yaw).

        Args:
            rotation_matrix: (3, 3) rotation matrix

        Returns:
            np.array: Euler angles [roll, pitch, yaw] in radians
        """
        r = Rotation.from_matrix(rotation_matrix)
        euler = r.as_euler('xyz', degrees=False)  # Roll, pitch, yaw
        return euler

    def map_to_robot(self, vp_data):
        """
        Map Vision Pro hand position to robot command using calibration.

        Args:
            vp_data: Dictionary with 'right_wrist' (1,4,4) and 'right_pinch_distance'

        Returns:
            dict or None: Robot command with 'position', 'orientation', 'gripper'
        """
        if not self.control_enabled or not self.calibrated:
            return None

        # Get right hand position and orientation
        right_wrist = vp_data['right_wrist'][0]
        hand_pos = right_wrist[:3, 3]
        hand_rot = right_wrist[:3, :3]

        # Normalize hand position to [0, 1] based on calibration
        normalized = (hand_pos - self.hand_min) / (self.hand_max - self.hand_min + 1e-6)
        normalized = np.clip(normalized, 0.0, 1.0)

        # Map to robot workspace
        target_position = self.robot_min + normalized * (self.robot_max - self.robot_min)

        # Orientation (absolute)
        target_orientation = self.rotation_matrix_to_quaternion(hand_rot)

        # Gripper control
        right_pinch = vp_data['right_pinch_distance']
        if right_pinch < 0.02:
            gripper = 0.0
        elif right_pinch > 0.05:
            gripper = 1.0
        else:
            gripper = (right_pinch - 0.02) / 0.03

        return {
            "position": target_position,
            "orientation": target_orientation,
            "gripper": gripper,
            "speed_scale": 0.5 if self.fine_control_mode else 1.0
        }
