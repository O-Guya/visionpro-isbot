"""
Kinova Robot Controller using PyBullet.
Handles inverse kinematics and joint control for Kinova robot arm.
"""

import numpy as np
import pybullet as p


class KinovaController:
    """Controller for Kinova robot arm in PyBullet simulation."""

    def __init__(self, robot_id, end_effector_link_index, num_joints=7):
        """
        Initialize the Kinova controller.

        Args:
            robot_id: PyBullet body ID of the robot
            end_effector_link_index: Index of the end effector link
            num_joints: Number of controllable joints (default: 7 for Gen3)
        """
        self.robot_id = robot_id
        self.end_effector_link_index = end_effector_link_index
        self.num_joints = num_joints

        # Get joint indices (exclude fixed joints)
        self.joint_indices = []
        for i in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, i)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                self.joint_indices.append(i)

        print(f"Kinova Controller initialized with {len(self.joint_indices)} joints")
        print(f"Joint indices: {self.joint_indices}")

        # Joint limits
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        for idx in self.joint_indices[:num_joints]:
            joint_info = p.getJointInfo(robot_id, idx)
            self.joint_limits_lower.append(joint_info[8])
            self.joint_limits_upper.append(joint_info[9])

        # Gripper joint indices (typically last 2-3 joints)
        # For Kinova Gen3, gripper joints are usually the last ones
        self.gripper_joint_indices = self.joint_indices[num_joints:]
        if len(self.gripper_joint_indices) > 0:
            print(f"Gripper joint indices: {self.gripper_joint_indices}")

        # Target positions for smoother control
        self.target_joint_positions = None

    def move_to_pose(self, position, orientation, speed_scale=1.0):
        """
        Move robot end effector to target pose using inverse kinematics.

        Args:
            position: [x, y, z] target position in world frame
            orientation: [x, y, z, w] target orientation as quaternion
            speed_scale: Speed scaling factor (0.0-1.0)

        Returns:
            bool: True if IK solution found, False otherwise
        """
        # Compute inverse kinematics
        joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_link_index,
            position,
            orientation,
            lowerLimits=self.joint_limits_lower,
            upperLimits=self.joint_limits_upper,
            jointRanges=[u - l for l, u in zip(self.joint_limits_lower, self.joint_limits_upper)],
            restPoses=[0.0] * self.num_joints,
            maxNumIterations=100,
            residualThreshold=1e-5
        )

        if joint_positions is None:
            print("IK solution not found!")
            return False

        # Store target positions
        self.target_joint_positions = joint_positions[:self.num_joints]

        # Apply joint commands with speed scaling
        max_velocity = 2.0 * speed_scale  # Adjust max velocity based on speed scale

        for i, (joint_idx, target_pos) in enumerate(zip(self.joint_indices[:self.num_joints],
                                                          self.target_joint_positions)):
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=500,
                maxVelocity=max_velocity
            )

        return True

    def control_gripper(self, gripper_command):
        """
        Control the gripper.

        Args:
            gripper_command: float 0.0-1.0 (0=closed, 1=open)
        """
        if len(self.gripper_joint_indices) == 0:
            return  # No gripper joints

        # Map gripper command to joint positions
        # Assuming gripper joints move in parallel
        for joint_idx in self.gripper_joint_indices:
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]

            # Linear mapping: 0.0 -> closed (lower_limit), 1.0 -> open (upper_limit)
            target_position = lower_limit + gripper_command * (upper_limit - lower_limit)

            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=target_position,
                force=100
            )

    def get_end_effector_pose(self):
        """
        Get current end effector pose.

        Returns:
            tuple: (position, orientation) where position is [x,y,z] and
                   orientation is quaternion [x,y,z,w]
        """
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        position = link_state[0]  # World position
        orientation = link_state[1]  # World orientation (quaternion)
        return position, orientation

    def get_joint_states(self):
        """
        Get current joint states.

        Returns:
            list: List of joint positions
        """
        joint_states = []
        for joint_idx in self.joint_indices[:self.num_joints]:
            state = p.getJointState(self.robot_id, joint_idx)
            joint_states.append(state[0])  # Joint position
        return joint_states

    def reset_to_home(self):
        """Reset robot to home position."""
        home_positions = [0.0] * self.num_joints

        for joint_idx, home_pos in zip(self.joint_indices[:self.num_joints], home_positions):
            p.resetJointState(self.robot_id, joint_idx, home_pos)

    def stop(self):
        """Emergency stop - hold current position."""
        current_positions = self.get_joint_states()

        for joint_idx, pos in zip(self.joint_indices[:self.num_joints], current_positions):
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=500,
                maxVelocity=0.0  # Zero velocity for immediate stop
            )

        print("Robot stopped at current position")
