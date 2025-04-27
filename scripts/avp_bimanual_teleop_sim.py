import sys

sys.path.append("../")
import platform
import time

import genesis as gs
import numpy as np
from avp_teleop import VisionProStreamer
from scipy.spatial.transform import Rotation as R
from avp_teleop.kinematics.bootster_t1 import (
    BoosterT1IKSolver,
    Chirality,
    avp_rel_transform_to_booster_rel_transform,
)
from avp_teleop.utils.constants.robot.booster import (
    L_EEF_LINK_NAME,
    R_EEF_LINK_NAME,
)
from avp_teleop.utils.geometry.transformations import convert_pose_mat2quat


avp_ip = "192.168.0.111"  # Change this IP as necessary.
streamer = VisionProStreamer(ip=avp_ip, record=False)
try:
    streamer.start_streaming()
except:
    print("Failed to start streaming. Please check the IP address.")

ik_solver = BoosterT1IKSolver(variant="bimanual", headless=False)


def run_sim(scene):

    i = 0
    while True:
        i += 1

        # # # move to pre-grasp pose
        # start = time.time()

        # data = streamer.latest
        # if data is None:
        #     time.sleep(0.01)
        #     continue

        # head_transform = data["head"][0]
        # left_wrist_transform = data["left_wrist"][0]
        # right_wrist_transform = data["right_wrist"][0]

        # # applying constant scaling

        # # Adjust the head transform if required.
        # # In the provided AVP streaming sample, a rotation around z by 90° is applied.
        # rotation = R.from_euler("z", [np.pi / 2])
        # adjustment_matrix = np.eye(4)
        # adjustment_matrix[:3, :3] = rotation.as_matrix()
        # head_transform = np.dot(adjustment_matrix, head_transform)

        # # === 4. COMPUTE RELATIVE TRANSFORMS: From head to wrists ===
        # # To obtain the hand (wrist) pose in the user's head frame, we compute:
        # #    T_head_to_wrist = inv(head_transform) * wrist_transform
        # T_head_to_left = np.linalg.inv(
        #     np.dot(np.linalg.inv(left_wrist_transform), head_transform)
        # )
        # T_head_to_right = np.linalg.inv(
        #     np.dot(np.linalg.inv(right_wrist_transform), head_transform)
        # )
        # T_head_to_left[2, 3] = 0.15
        # T_head_to_right[2, 3] = 0.15

        T_head_to_right = np.array([[-1, 0, 0,  0.33442654],
            [0, 0, 1, -0.38677538],
            [0, 1, 0, 0.15 ],
            [ 0.     ,     0.     ,     0.      ,    1.        ]])

        T_head_to_left = np.array([[ 1, 0, 0 , 0.33442654 ],
            [ 0, -1, 0,  0.38677538],
            [ 0, 0, -1 , 0.15],
            [ 0.       ,   0.     ,     0.       ,   1.        ]])

        # T_head_to_right = np.eye(4)
        # T_head_to_left = np.eye(4)
        # T_head_to_right[:3, :3] = R.from_euler("x", [np.pi / 2]).as_matrix()
        # T_head_to_left[:3, :3] = R.from_euler("x", [-np.pi / 2]).as_matrix()
        # T_head_to_right[:3, 3] = [0.33442654, -0.38677538, 0.15]
        # T_head_to_left[:3, 3] = [0.33442654, 0.38677538, 0.15]

        T_head_to_right[:3, 3] *= 0.80
        T_head_to_left[:3, 3] *= 0.80

        # === 5. CONVERT COORDINATE FRAMES: AVP -> Booster ===
        # Use the provided helper function for coordinate conversion.
        left_rel_booster = avp_rel_transform_to_booster_rel_transform(
            T_head_to_left, Chirality.LEFT
        )
        right_rel_booster = avp_rel_transform_to_booster_rel_transform(
            T_head_to_right, Chirality.RIGHT
        )

        # robot head to left eef
        left_eef_pose = left_rel_booster
        # left_eef_pose = np.dot(ik_solver.T_head_to_base, left_rel_booster)

        right_eef_pose = right_rel_booster
        # right_eef_pose = np.dot(ik_solver.T_head_to_base, right_rel_booster)

        qpos, error = ik_solver.solve(left_eef_pose, right_eef_pose)
        # robot.set_dofs_position(qpos[BIMANUAL_MASK[1:]], dofs_idx_local=indices)
        robot.set_dofs_position(qpos)

        time.sleep(0.05)

        scene.step()

    scene.viewer.stop()


scene = ik_solver.scene
robot = ik_solver.robot

neural_pose = robot.get_qpos()
print(neural_pose)
robot.set_dofs_position(neural_pose)
scene.step()

end_effector_l = robot.get_link(L_EEF_LINK_NAME)
end_effector_r = robot.get_link(R_EEF_LINK_NAME)

if platform.system() == "Darwin":
    gs.tools.run_in_another_thread(fn=run_sim, args=(scene,))
    scene.viewer.start()
elif platform.system() == "Linux":
    scene.viewer.start()
    run_sim(scene)
