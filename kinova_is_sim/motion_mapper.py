"""
Simple Motion Mapper for Vision Pro to Kinova teleoperation.
Maps Vision Pro hand tracking data to Kinova robot commands.
"""

import numpy as np
from scipy.spatial.transform import Rotation


class SimpleMotionMapper:
    """Simple direct pose mapping from Vision Pro to Kinova robot."""

    def __init__(self, position_scale=2.5, workspace_offset=None, use_head_relative=True):
        """
        Initialize the motion mapper.

        Args:
            position_scale: Scale factor for position mapping (default: 2.5)
                           e.g., hand moves 0.2m -> robot moves 0.5m
                           Higher value = robot moves more for same hand movement
            workspace_offset: Offset to center of robot workspace [x, y, z]
                             (default: [0.5, 0.3, 0.5])
            use_head_relative: If True, use head position as reference for hand control
                              This allows user to stand anywhere and control relative to head
        """
        self.control_enabled = False
        self.speed_scale = 1.0
        self.position_scale = position_scale
        self.workspace_offset = workspace_offset if workspace_offset is not None else np.array([0.5, 0.3, 0.5])

        # Head-relative control
        self.use_head_relative = use_head_relative
        self.head_reference_pos = None  # Will be set when control is enabled
        self.hand_reference_pos = None  # Initial hand position when control starts

        # For detecting pinch events (state tracking)
        self.last_left_pinch = False
        self.last_right_pinch = False

        # Fine control mode
        self.fine_control_mode = False
        self.default_position_scale = position_scale  # Store original scale

    def detect_fist(self, finger_matrices):
        """
        Detect if hand is making a fist.

        Args:
            finger_matrices: (25, 4, 4) array of finger joint transforms

        Returns:
            bool: True if hand is in fist pose
        """
        # Simple heuristic: check if finger tips are close to palm
        # Finger tips are at indices: 4, 9, 14, 19, 24
        finger_tip_indices = [4, 9, 14, 19, 24]
        wrist_pos = finger_matrices[0, :3, 3]  # Wrist position

        distances = []
        for idx in finger_tip_indices:
            tip_pos = finger_matrices[idx, :3, 3]
            dist = np.linalg.norm(tip_pos - wrist_pos)
            distances.append(dist)

        # If average distance is small, it's a fist
        avg_distance = np.mean(distances)
        return avg_distance < 0.1  # 10cm threshold

    def detect_open_hand(self, finger_matrices):
        """
        Detect if hand is fully open (emergency stop gesture).

        Args:
            finger_matrices: (25, 4, 4) array of finger joint transforms

        Returns:
            bool: True if hand is fully open
        """
        # Check if all fingers are extended
        finger_tip_indices = [4, 9, 14, 19, 24]
        wrist_pos = finger_matrices[0, :3, 3]

        distances = []
        for idx in finger_tip_indices:
            tip_pos = finger_matrices[idx, :3, 3]
            dist = np.linalg.norm(tip_pos - wrist_pos)
            distances.append(dist)

        # If average distance is large, hand is open
        avg_distance = np.mean(distances)
        return avg_distance > 0.15  # 15cm threshold

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
        Map Vision Pro data to robot command.

        Args:
            vp_data: Dictionary containing Vision Pro tracking data:
                - 'right_wrist': (1, 4, 4) transformation matrix
                - 'left_wrist': (1, 4, 4) transformation matrix
                - 'right_fingers': (25, 4, 4) finger joint matrices
                - 'left_fingers': (25, 4, 4) finger joint matrices
                - 'right_pinch_distance': float
                - 'left_pinch_distance': float

        Returns:
            dict or None: Robot command dictionary with:
                - 'position': [x, y, z] target position
                - 'orientation': [x, y, z, w] quaternion
                - 'gripper': float 0.0-1.0 (0=closed, 1=open)
                - 'speed_scale': float
            Returns None if control is disabled.
            Returns {'emergency_stop': True} if emergency stop is detected.
        """

        # === Left hand gesture detection ===
        left_pinch = vp_data['left_pinch_distance'] < 0.02  # 2cm threshold
        left_fist = self.detect_fist(vp_data['left_fingers'])
        left_open = self.detect_open_hand(vp_data['left_fingers'])

        # Detect pinch event (edge trigger)
        left_pinch_event = left_pinch and not self.last_left_pinch
        self.last_left_pinch = left_pinch

        # Toggle control on/off with left hand pinch
        if left_pinch_event:
            self.control_enabled = not self.control_enabled
            if self.control_enabled:
                # Initialize reference positions when control is enabled
                if self.use_head_relative:
                    self.head_reference_pos = vp_data['head'][0, :3, 3].copy()
                    right_wrist_abs = vp_data['right_wrist'][0, :3, 3]
                    self.hand_reference_pos = right_wrist_abs.copy()
                    print(f"Control ENABLED (Head-relative mode)")
                    print(f"  Head reference: {self.head_reference_pos}")
                    print(f"  Hand reference: {self.hand_reference_pos}")
                else:
                    print("Control ENABLED (Absolute mode)")
            else:
                print("Control DISABLED")
                self.head_reference_pos = None
                self.hand_reference_pos = None

        # Emergency stop with left hand open
        if left_open:
            print("EMERGENCY STOP - Left hand open detected!")
            return {"emergency_stop": True}

        # Return None if control is not enabled
        if not self.control_enabled:
            return None

        # Fine control mode (left fist)
        if left_fist:
            if not self.fine_control_mode:
                print("Fine control mode ENABLED")
                self.fine_control_mode = True
            self.speed_scale = 0.3
            self.position_scale = self.default_position_scale * 0.4  # 40% of default scale
        else:
            if self.fine_control_mode:
                print("Fine control mode DISABLED")
                self.fine_control_mode = False
            self.speed_scale = 1.0
            self.position_scale = self.default_position_scale

        # === Right hand position mapping ===
        right_wrist = vp_data['right_wrist'][0]  # Shape: (4, 4)
        right_wrist_pos = right_wrist[:3, 3]  # Extract position [x, y, z]

        if self.use_head_relative and self.head_reference_pos is not None:
            # Head-relative control: compute hand position relative to head
            head_pos = vp_data['head'][0, :3, 3]

            # Current hand position relative to current head position
            hand_relative_to_head = right_wrist_pos - head_pos

            # Apply scaling to the relative displacement
            # This means small hand movements create larger robot movements
            target_position = hand_relative_to_head * self.position_scale + self.workspace_offset
        else:
            # Absolute control (original behavior)
            target_position = right_wrist_pos * self.position_scale + self.workspace_offset

        # === Right hand orientation mapping ===
        right_wrist_rotation = right_wrist[:3, :3]  # Extract 3x3 rotation matrix
        target_orientation = self.rotation_matrix_to_quaternion(right_wrist_rotation)

        # === Gripper mapping (right hand pinch) ===
        right_pinch_dist = vp_data['right_pinch_distance']

        if right_pinch_dist < 0.02:
            gripper_command = 0.0  # Fully closed
        elif right_pinch_dist > 0.05:
            gripper_command = 1.0  # Fully open
        else:
            # Linear interpolation between closed and open
            gripper_command = (right_pinch_dist - 0.02) / 0.03

        # Assemble robot command
        robot_command = {
            "position": target_position,
            "orientation": target_orientation,  # Quaternion [x, y, z, w]
            "gripper": gripper_command,
            "speed_scale": self.speed_scale
        }

        return robot_command
