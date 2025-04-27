import os
import unittest
import numpy as np

# Import the classes and functions from your file.
# Adjust the module name ("booster_ik") if necessary.
from avp_teleop.kinematics.bootster_t1 import (
    BoosterT1IKSolver,
    avp_rel_transform_to_booster_rel_transform,
    Chirality,
)

# Also import the constant transformation matrices.
from avp_teleop.utils.constants.robot.booster.transformation import (
    T_TO_LEFT_WRIST,
    T_TO_RIGHT_WRIST,
)

solver = BoosterT1IKSolver(variant="bimanual", headless=True)


class TestBoosterT1IKSolver(unittest.TestCase):
    def setUp(self):
        """
        Create a BoosterT1IKSolver instance for testing.
        We use variant "bimanual" and headless mode to avoid viewer popups.
        """
        self.solver = solver

    def test_initialization(self):
        """Test that the solver initializes with the expected attributes."""
        # Check that the variant is one of the allowed ones.
        self.assertIn(self.solver.variant, self.solver.ALLOWED_VARIANTS)
        # Check that the link names and DOF are set.
        self.assertIsNotNone(self.solver.l_eef_name)
        self.assertIsNotNone(self.solver.r_eef_name)
        self.assertIsNotNone(self.solver.head_link)
        self.assertIsNotNone(self.solver.base_link)
        self.assertGreater(self.solver.dof, 0)

    def test_get_link_pose(self):
        """Test that get_link_pose returns a 4x4 numpy array."""
        pose = self.solver.get_link_pose(self.solver.l_eef_name)
        self.assertEqual(pose.shape, (4, 4))

    def test_T_head_to_base(self):
        """Test that the computed T_head_to_base is a 4x4 matrix."""
        T = self.solver.T_head_to_base
        self.assertEqual(T.shape, (4, 4))

    def test_forward(self):
        """Test that forward kinematics returns left and right 4x4 poses."""
        # Get the current joint configuration.
        joint_angles = self.solver.robot.get_qpos()
        fk = self.solver.forward(joint_angles)
        self.assertIn("left_eef_pose", fk)
        self.assertIn("right_eef_pose", fk)
        self.assertEqual(fk["left_eef_pose"].shape, (4, 4))
        self.assertEqual(fk["right_eef_pose"].shape, (4, 4))

    def test_solve_with_valid_poses(self):
        """
        Test that using the current (reachable) end–effector poses
        (obtained via forward kinematics) produces a valid IK solution.
        """
        current_joint_pos = self.solver.robot.get_qpos()
        fk = self.solver.forward(current_joint_pos)
        left_pose = fk["left_eef_pose"]
        right_pose = fk["right_eef_pose"]

        # Compute the IK solution.
        qpos, error = self.solver.solve(left_pose, right_pose, current_joint_pos=current_joint_pos)
        # Check that the returned joint configuration has the expected number of DOFs.
        self.assertEqual(qpos.shape[0], self.solver.dof)
        # Also verify that the error is returned as a numpy array.
        self.assertIsInstance(error, np.ndarray)

    def test_solve_right_only(self):
        """
        Test the branch where only a right end–effector pose is provided.
        (Note: according to the implementation, providing only left pose
         may not work correctly.)
        """
        current_joint_pos = self.solver.robot.get_qpos()
        fk = self.solver.forward(current_joint_pos)
        right_pose = fk["right_eef_pose"]

        # This branch calls _solve_right_eef_pose.
        qpos, error = self.solver.solve(None, right_pose, current_joint_pos=current_joint_pos)
        self.assertEqual(qpos.shape[0], self.solver.dof)

    def test_solve_assert_both_none(self):
        """
        Test that calling solve with both end–effector poses as None
        triggers an assertion error.
        """
        with self.assertRaises(AssertionError):
            self.solver.solve(None, None)

    def test_avp_rel_transform(self):
        """
        Test the helper function avp_rel_transform_to_booster_rel_transform.
        For an identity transform, the result should equal the corresponding wrist transform.
        """
        identity = np.eye(4)

        # Test for LEFT chirality.
        left_result = avp_rel_transform_to_booster_rel_transform(identity, Chirality.LEFT)
        expected_left = np.dot(identity, T_TO_LEFT_WRIST)
        np.testing.assert_array_almost_equal(left_result, expected_left)

        # Test for RIGHT chirality.
        right_result = avp_rel_transform_to_booster_rel_transform(identity, Chirality.RIGHT)
        expected_right = np.dot(identity, T_TO_RIGHT_WRIST)
        np.testing.assert_array_almost_equal(right_result, expected_right)


if __name__ == "__main__":
    # Run all the tests.
    unittest.main()
