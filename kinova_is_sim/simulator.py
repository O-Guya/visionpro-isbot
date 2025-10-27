"""
PyBullet Simulator for Kinova Robot.
Sets up simulation environment and loads robot model.
"""

import pybullet as p
import pybullet_data
import numpy as np
import os


class KinovaSimulator:
    """PyBullet simulator for Kinova robot arm."""

    def __init__(self, gui=True, kinova_urdf_path=None):
        """
        Initialize PyBullet simulator.

        Args:
            gui: If True, use GUI mode; otherwise use DIRECT mode
            kinova_urdf_path: Path to Kinova URDF file. If None, will try to
                            find it in pybullet_data or common locations
        """
        # Connect to PyBullet
        if gui:
            self.physics_client = p.connect(p.GUI)
            print("PyBullet GUI started")
        else:
            self.physics_client = p.connect(p.DIRECT)
            print("PyBullet DIRECT mode started")

        # Configure GUI camera
        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=50,
                cameraPitch=-35,
                cameraTargetPosition=[0.5, 0.0, 0.5]
            )

        # Set additional search paths
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Physics settings
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)  # 240 Hz simulation

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        print("Ground plane loaded")

        # Load Kinova robot
        self.robot_id = None
        self.end_effector_link_index = None
        self.kinova_urdf_path = kinova_urdf_path

        # Gripper visualization
        self.gripper_visual_ids = []
        self.gui_mode = gui

        self._load_kinova()

        # Add gripper visualization after robot is loaded
        if gui:
            self._create_gripper_visualization()

    def _load_kinova(self):
        """Load Kinova robot model."""
        # Try to find Kinova URDF
        urdf_paths_to_try = []

        if self.kinova_urdf_path is not None:
            urdf_paths_to_try.append(self.kinova_urdf_path)

        # Common URDF locations
        urdf_paths_to_try.extend([
            "kinova_gen3.urdf",
            "kinova/gen3.urdf",
            os.path.join(pybullet_data.getDataPath(), "kinova", "kinova_gen3.urdf"),
            "kinova_robotiq_85.urdf",
        ])

        # Try each path
        for urdf_path in urdf_paths_to_try:
            try:
                print(f"Trying to load URDF from: {urdf_path}")
                self.robot_id = p.loadURDF(
                    urdf_path,
                    basePosition=[0, 0, 0],
                    baseOrientation=[0, 0, 0, 1],
                    useFixedBase=True
                )
                print(f"Successfully loaded Kinova from: {urdf_path}")
                break
            except Exception as e:
                print(f"Failed to load from {urdf_path}: {e}")
                continue

        if self.robot_id is None:
            print("\n" + "="*60)
            print("WARNING: Could not find Kinova URDF file!")
            print("="*60)
            print("\nUsing fallback: Loading Kuka IIWA arm as placeholder")
            print("To use Kinova, you need to:")
            print("1. Download Kinova Gen3 URDF")
            print("2. Place it in project directory or specify path")
            print("="*60 + "\n")

            # Load Kuka IIWA as fallback (available in pybullet_data)
            self.robot_id = p.loadURDF(
                "kuka_iiwa/model.urdf",
                basePosition=[0, 0, 0],
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=True
            )
            print("Fallback robot (Kuka IIWA) loaded successfully")

        # Find end effector link
        self._find_end_effector_link()

        # Print robot info
        self._print_robot_info()

    def _find_end_effector_link(self):
        """Find the end effector link index."""
        num_joints = p.getNumJoints(self.robot_id)

        # Look for end effector link (usually named with "ee", "gripper", or last link)
        end_effector_candidates = []

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            # Check for common end effector names
            if any(keyword in link_name.lower() for keyword in ['ee', 'end', 'effector', 'gripper', 'tool']):
                end_effector_candidates.append((i, link_name))

        if end_effector_candidates:
            # Use the first candidate
            self.end_effector_link_index = end_effector_candidates[0][0]
            print(f"End effector link: {end_effector_candidates[0][1]} (index: {self.end_effector_link_index})")
        else:
            # Use the last link as end effector
            self.end_effector_link_index = num_joints - 1
            joint_info = p.getJointInfo(self.robot_id, self.end_effector_link_index)
            link_name = joint_info[12].decode('utf-8')
            print(f"Using last link as end effector: {link_name} (index: {self.end_effector_link_index})")

    def _print_robot_info(self):
        """Print robot joint information."""
        print("\n" + "="*60)
        print("Robot Joint Information:")
        print("="*60)

        num_joints = p.getNumJoints(self.robot_id)
        print(f"Total joints: {num_joints}\n")

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            link_name = joint_info[12].decode('utf-8')

            joint_type_str = {
                p.JOINT_REVOLUTE: "REVOLUTE",
                p.JOINT_PRISMATIC: "PRISMATIC",
                p.JOINT_FIXED: "FIXED",
                p.JOINT_SPHERICAL: "SPHERICAL",
                p.JOINT_PLANAR: "PLANAR"
            }.get(joint_type, "UNKNOWN")

            print(f"Joint {i}: {joint_name}")
            print(f"  Type: {joint_type_str}")
            print(f"  Link: {link_name}")

            if joint_type != p.JOINT_FIXED:
                print(f"  Limits: [{joint_info[8]:.2f}, {joint_info[9]:.2f}]")
            print()

        print("="*60 + "\n")

    def step(self):
        """Step the simulation forward."""
        p.stepSimulation()

    def reset(self):
        """Reset simulation."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self._load_kinova()

    def close(self):
        """Close the simulator."""
        p.disconnect()
        print("Simulator closed")

    def add_debug_axes(self, position, orientation, length=0.1):
        """
        Add debug coordinate axes at specified pose.

        Args:
            position: [x, y, z]
            orientation: [x, y, z, w] quaternion
            length: Length of axes lines
        """
        # Convert quaternion to rotation matrix
        rot_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # X axis (red)
        x_end = position + rot_matrix[:, 0] * length
        p.addUserDebugLine(position, x_end, [1, 0, 0], lineWidth=2, lifeTime=0.1)

        # Y axis (green)
        y_end = position + rot_matrix[:, 1] * length
        p.addUserDebugLine(position, y_end, [0, 1, 0], lineWidth=2, lifeTime=0.1)

        # Z axis (blue)
        z_end = position + rot_matrix[:, 2] * length
        p.addUserDebugLine(position, z_end, [0, 0, 1], lineWidth=2, lifeTime=0.1)

    def add_debug_sphere(self, position, radius=0.02, color=[1, 0, 0], lifetime=0.1):
        """
        Add debug sphere at specified position.

        Args:
            position: [x, y, z]
            radius: Sphere radius
            color: RGB color [r, g, b]
            lifetime: How long sphere stays (0 = one frame)
        """
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color + [0.8]
        )

        body_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=position
        )

        return body_id

    def get_robot_id(self):
        """Get PyBullet robot body ID."""
        return self.robot_id

    def get_end_effector_link_index(self):
        """Get end effector link index."""
        return self.end_effector_link_index

    def _create_gripper_visualization(self):
        """Create visual representation of gripper fingers."""
        if not self.gui_mode or self.robot_id is None:
            return

        # Create two simple boxes to represent gripper fingers
        # Left finger
        left_finger_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.01, 0.03, 0.06],  # thin, wide, long
            rgbaColor=[0.2, 0.2, 0.8, 0.8]  # Blue, semi-transparent
        )

        # Right finger
        right_finger_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.01, 0.03, 0.06],
            rgbaColor=[0.2, 0.2, 0.8, 0.8]
        )

        # Create multi-bodies for fingers (no collision, visual only)
        left_finger = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=left_finger_visual,
            basePosition=[0, 0, 0]
        )

        right_finger = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=right_finger_visual,
            basePosition=[0, 0, 0]
        )

        self.gripper_visual_ids = [left_finger, right_finger]
        print(f"✓ Gripper visualization created (IDs: {self.gripper_visual_ids})")

    def update_gripper_visualization(self, gripper_state=0.5):
        """
        Update gripper visualization position and opening.

        Args:
            gripper_state: 0.0 = fully closed, 1.0 = fully open
        """
        if not self.gripper_visual_ids or self.end_effector_link_index is None:
            return

        # Get end effector position and orientation
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        ee_pos = np.array(link_state[0])  # World position
        ee_ori = link_state[1]  # World orientation (quaternion)

        # Convert quaternion to rotation matrix
        rot_matrix = np.array(p.getMatrixFromQuaternion(ee_ori)).reshape(3, 3)

        # Gripper opening width (0.0 to 0.08 meters)
        max_opening = 0.08  # 8cm max opening
        opening_width = gripper_state * max_opening

        # Calculate finger positions (offset from end effector)
        # Assume gripper opens along Y axis
        forward_offset = rot_matrix[:, 2] * 0.08  # 8cm forward along Z
        left_offset = rot_matrix[:, 1] * (opening_width / 2)  # Half opening to the left
        right_offset = rot_matrix[:, 1] * (-opening_width / 2)  # Half opening to the right

        left_finger_pos = ee_pos + forward_offset + left_offset
        right_finger_pos = ee_pos + forward_offset + right_offset

        # Update finger positions
        p.resetBasePositionAndOrientation(
            self.gripper_visual_ids[0],
            left_finger_pos,
            ee_ori
        )

        p.resetBasePositionAndOrientation(
            self.gripper_visual_ids[1],
            right_finger_pos,
            ee_ori
        )
