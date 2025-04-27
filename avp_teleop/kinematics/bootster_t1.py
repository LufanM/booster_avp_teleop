import os
from enum import IntEnum
from typing import Optional

import genesis as gs
import numpy as np
from avp_teleop.utils.constants.robot.booster import (BASE_LINK_NAME,
                                                      HEAD_LINK_NAME,
                                                      L_EEF_LINK_NAME,
                                                      R_EEF_LINK_NAME)
from avp_teleop.utils.constants.robot.booster.transformation import (
    T_TO_LEFT_WRIST, T_TO_RIGHT_WRIST)
from avp_teleop.utils.geometry.transformations import (convert_pose_quat2mat,
                                                       quaternion_from_matrix)


# Define an enum for chirality (if needed in the future).
class Chirality(IntEnum):
    LEFT = 0
    RIGHT = 1


def avp_rel_transform_to_booster_rel_transform(avp_rel_transform, chirality: Chirality):
    """
    Convert a relative transformation matrix from the head to wrist of the Apple Vision Pro
    coordinate system to the Booster coordinate system. The returned matrix is the
    transformation from the booster's head link to the wrist link of the specified chirality.

    Args:
        avp_rel_transform (np.ndarray): The relative transformation matrix in the AVP coordinate system.
        chirality (Chirality): The chirality of the robot (left or right).

    Returns:
        np.ndarray: The transformed relative transformation matrix in the Booster coordinate system.
    """

    transform = np.eye(4)
    if chirality == Chirality.LEFT:
        transform = T_TO_LEFT_WRIST
    elif chirality == Chirality.RIGHT:
        transform = T_TO_RIGHT_WRIST

    return np.dot(avp_rel_transform, transform)


class BoosterT1IKSolver:
    """
    IK Solver for Booster T1 robots.

    This class wraps a Genesis-based IK solver to compute joint configurations for
    achieving desired left and right end-effector poses. The user may select among three
    variants of the Booster robot by specifying the variant name.
    """

    # Allowed variants.
    ALLOWED_VARIANTS = {"bimanual_fixed_head", "bimanual", "full_body"}

    def __init__(self, variant: str = "full_body", headless: bool = True):
        """
        Initialize the IK solver.

        Args:
            variant (str): Which booster variant to use. Options are "bimanual_fixed_head", "bimanual", or "full_body".
                           Defaults to "full_body".
        """

        super().__init__()

        if variant not in self.ALLOWED_VARIANTS:
            raise ValueError(
                f"Variant '{variant}' not recognized. Allowed variants: {self.ALLOWED_VARIANTS}"
            )

        self.variant = variant

        # Dynamically import the proper constants and determine the URDF filename.
        # It is assumed that the constants files contain definitions for at least:
        #   - L_EEF_LINK_NAME: left end–effector link name
        #   - R_EEF_LINK_NAME: right end–effector link name
        #   - DOF: number of degrees of freedom
        if variant == "bimanual_fixed_head":
            urdf_filename = "T1_7_dof_arms_bimanual_fixed_head.urdf"
        elif variant == "bimanual":
            urdf_filename = "T1_7_dof_arms_bimanual.urdf"
        elif variant == "full_body":
            urdf_filename = "T1_7_dof_arms_full_body.urdf"
        else:
            # Should never reach here because of the earlier check.
            raise ValueError(f"Unhandled variant: {variant}")

        # Store important constants in the instance.
        self.l_eef_name = L_EEF_LINK_NAME
        self.r_eef_name = R_EEF_LINK_NAME
        self.head_link = HEAD_LINK_NAME
        self.base_link = BASE_LINK_NAME

        # Determine the full file path for the URDF.
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../data/Booster_T1"))
        urdf_path = os.path.join(base_dir, urdf_filename)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        if not gs._initialized:
            gs.init()

        # Create a Genesis scene for IK computations.
        # We disable the viewer as visualization is not required.
        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 720),
                camera_pos=(-1.5, 0, 2.5),
                camera_lookat=(0.0, 0.0, 0.0),
                camera_fov=60,
            ),
            sim_options=gs.options.SimOptions(),
            show_viewer=not headless,
            rigid_options=gs.options.RigidOptions(
                dt=0.01,
                gravity=(0.0, 0.0, 0.0),
            ),
            vis_options=gs.options.VisOptions(
                show_world_frame=True,  # visualize the coordinate frame of `world` at its origin
                world_frame_size=1.0,  # length of the world frame in meter
                show_link_frame=True,  # do not visualize coordinate frames of entity links
            ),
        )

        # Add the robot to the scene using the URDF file.
        # The robot is to be fixed (its base does not move).
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=urdf_path,
                fixed=True,
            )
        )
        # Finalize the scene build so that kinematic computations work.
        self.scene.build()

        self.dof = self.robot.n_dofs

    @property
    def T_head_to_base(self) -> np.ndarray:
        """The transformation from the head link to the base link.

        Returns:
            np.ndarray: _description_
        """
        return np.dot(
            np.linalg.inv(self.get_link_pose(self.base_link)), self.get_link_pose(self.head_link)
        )

    def get_link_pose(self, link_name: str) -> np.ndarray:
        """Given the link name, return the pose of the link.

        Args:
            link_name (str): _description_

        Returns:
            np.ndarray: _description_
        """

        link = self.robot.get_link(link_name)
        return convert_pose_quat2mat(
            np.concatenate((link.get_pos().cpu().numpy(), link.get_quat().cpu().numpy()))
        )

    def _solve_both_eef_poses(self, left_eef_pose: np.ndarray, right_eef_pose: np.ndarray):

        # Convert the desired 4x4 poses into position and quaternion for IK.
        left_pos = left_eef_pose[:3, 3]
        left_quat = quaternion_from_matrix(left_eef_pose)

        right_pos = right_eef_pose[:3, 3]
        right_quat = quaternion_from_matrix(right_eef_pose)

        # Retrieve the link objects for the left and right end–effectors.
        left_link = self.robot.get_link(self.l_eef_name)
        right_link = self.robot.get_link(self.r_eef_name)

        qpos, error = self.robot.inverse_kinematics_multilink(
            links=[left_link, right_link],
            poss=[left_pos, right_pos],
            quats=[left_quat, right_quat],
            return_error=True,
        )
        return qpos.cpu().numpy(), error.cpu().numpy()

    def _solve_left_eef_pose(self, left_eef_pose: np.ndarray):
        # Convert the desired 4x4 poses into position and quaternion for IK.
        left_pos = left_eef_pose[:3, 3]
        left_quat = quaternion_from_matrix(left_eef_pose)

        # Retrieve the link objects for the left and right end–effectors.
        left_link = self.robot.get_link(self.l_eef_name)

        qpos, error = self.robot.inverse_kinematics(
            link=left_link,
            pos=left_pos,
            quat=left_quat,
            return_error=True,
        )
        return qpos.cpu().numpy(), error.cpu().numpy()

    def _solve_right_eef_pose(self, right_eef_pose: np.ndarray):
        # Convert the desired 4x4 poses into position and quaternion for IK.
        right_pos = right_eef_pose[:3, 3]
        right_quat = quaternion_from_matrix(right_eef_pose)

        # Retrieve the link objects for the left and right end–effectors.
        right_link = self.robot.get_link(self.r_eef_name)

        qpos, error = self.robot.inverse_kinematics(
            link=right_link,
            pos=right_pos,
            quat=right_quat,
            return_error=True,
        )
        return qpos.cpu().numpy(), error.cpu().numpy()

    def solve(
        self,
        left_eef_pose: Optional[np.ndarray],
        right_eef_pose: Optional[np.ndarray],
        current_joint_pos: np.ndarray = None,
    ):
        """
        Compute the joint configurations that achieve the desired end–effector poses.

        Args:
            left_eef_pose (np.ndarray): [4, 4] homogeneous transformation matrix of the left end–effector in world frame.
            right_eef_pose (np.ndarray): [4, 4] homogeneous transformation matrix of the right end–effector in world frame.
            current_joint_pos (np.ndarray): Current joint positions (as an initial guess for IK).

        Returns:
            tuple: (qpos, error) where
                - qpos (np.ndarray): Computed joint positions.
                - error (np.ndarray): Residual error of the IK solution.
        """

        # TODO: make sure both target poses are reachable, check by total length of the arm.
        assert left_eef_pose is not None or right_eef_pose is not None, "At least one end–effector pose must be provided."

        # Update the robot with the current joint positions.
        if current_joint_pos is None:
            current_joint_pos = self.robot.get_qpos()
        self.robot.set_dofs_position(current_joint_pos)
        self.scene.step()  # Step the simulation to update transforms

        if left_eef_pose is not None and right_eef_pose is None:
            return self._solve_both_eef_poses(left_eef_pose, right_eef_pose)
        elif left_eef_pose is None and right_eef_pose is not None:
            return self._solve_right_eef_pose(right_eef_pose)
        else:
            return self._solve_both_eef_poses(left_eef_pose, right_eef_pose)

    def forward(self, joint_angles: np.ndarray):
        """
        Compute the forward kinematics for the robot given joint angles.

        This method sets the robot to the provided joint configuration and then retrieves
        the transformation matrices (4x4 homogeneous) for the left and right end–effector links.

        Args:
            joint_angles (np.ndarray): Joint angles in radians.

        Returns:
            dict: A dictionary with keys "left_eef_pose" and "right_eef_pose" containing
                  the respective 4x4 homogeneous transformation matrices.
        """
        # Update the robot configuration.
        self.robot.set_dofs_position(joint_angles)
        self.scene.step()  # Update transforms

        # Retrieve the forward kinematics for both end–effectors.
        left_pose = self.get_link_pose(self.l_eef_name)
        right_pose = self.get_link_pose(self.r_eef_name)

        return {"left_eef_pose": left_pose, "right_eef_pose": right_pose}
