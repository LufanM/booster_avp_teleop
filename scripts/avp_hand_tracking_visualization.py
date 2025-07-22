import sys
import time

import numpy as np
import rerun as rr
from avp_teleop import VisionProStreamer
from scipy.spatial.transform import Rotation as R

T_robot_avp = np.array([[0, 0, -1, 0], # rot z -90° and rot y -90° fixed axis: Roty(-90d)*Rotz(-90d)
                        [-1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

T_head_2booster = np.array([[0, -1, 0, 0], # rot z 90°
                            [1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]) 

T_wrist_left_2booster = np.array([[0, 1, 0, 0], # rot x 90° and roty 90° fixed axis of Wrist (Robot Convention): Roty(90°)*Rotx(90°)
                                    [0, 0, -1, 0],
                                    [-1, 0, 0, 0],
                                    [0, 0, 0, 1]]) 

T_wrist_right_2booster = np.array([[0, 1, 0, 0], # rot x -90° and rotz -90° fixed axis of Wrist(Robot Convention): Roty(-90°)*Rotx(-90°)
                                    [0, 0, 1, 0],
                                    [1, 0, 0, 0],
                                    [0, 0, 0, 1]]) 



def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret

def log_transform_axes(name: str, transform: np.ndarray, scale: float = 0.1) -> None:
    """
    Given a 4x4 transformation matrix, log a 3D axis at the joint.
    - `name` is the channel name (e.g., "head" or "right_fingers/3").
    - `scale` determines the length of the drawn axes.
    """
    # Extract the origin and the three local axes:
    origin = transform[:3, 3]
    x_dir = transform[:3, 0]
    y_dir = transform[:3, 1]
    z_dir = transform[:3, 2]

    # Compute endpoints for the axes lines:
    x_end = origin + scale * x_dir
    y_end = origin + scale * y_dir
    z_end = origin + scale * z_dir

    # Log the x-axis (red), y-axis (green), and z-axis (blue).
    # (We assume that rr.LineStrip3D takes an array of positions and an array of colors.)
    rr.log(
        f"{name}/axis/x",
        rr.LineStrips3D(np.array([origin, x_end]), colors=np.array([[255, 0, 0]])),  # red for x
    )
    rr.log(
        f"{name}/axis/y",
        rr.LineStrips3D(np.array([origin, y_end]), colors=np.array([[0, 255, 0]])),  # green for y
    )
    rr.log(
        f"{name}/axis/z",
        rr.LineStrips3D(np.array([origin, z_end]), colors=np.array([[0, 0, 255]])),  # blue for z
    )

def save_data(data, filename):
    np.savetxt(filename, data, header='time, x_l, y_l, z_l, euler_r_l, euler_p_l, euler_y_l, x_r, y_r, z_r, euler_r_r, euler_p_r, euler_y_r,', delimiter=',')

def main_avp_raw():
    # The 'spawn=True' argument opens the GUI viewer in a separate window.
    rr.init("hand_tracking", spawn=True)

    # Connect to the AVP device streaming hand tracking data.
    avp_ip = "192.168.101.131"
    streamer = VisionProStreamer(ip=avp_ip, record=False)
    streamer.start_streaming()
    start = time.time()

    while True:
        # Get the latest data from the stream
        data = streamer.latest

        # gloabl coordinate
        global_coordinate = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        log_transform_axes("global", global_coordinate) # not avp global coordinate

        # Process head data (shape: (1,4,4))
        head_mat = data["head"][0]
        log_transform_axes("head", head_mat)

        # Process right and left wrist data (each shape: (1,4,4))
        right_wrist_mat = data["right_wrist"][0]
        left_wrist_mat = data["left_wrist"][0]
   
        # # compute the relative transform from the wrist to the head
        # left_wrist_to_head = np.dot(np.linalg.inv(left_wrist_transform), head_transform)
        # print(np.linalg.inv(left_wrist_to_head))
        # right_wrist_to_head = np.dot(np.linalg.inv(right_wrist_mat), head_mat)
        # print(np.linalg.inv(right_wrist_to_head))
        
        log_transform_axes("right_wrist", right_wrist_mat)
        log_transform_axes("left_wrist", left_wrist_mat)

        # Process finger joints.
        # Each finger joint array is of shape (25,4,4). The raw avp finger data is based on wrist.So it transforms to based on avp world for visualization
        right_fingers_mat = data["right_fingers"]
        left_fingers_mat = data["left_fingers"]

        
        for i in range(right_fingers_mat.shape[0]):
            finger_mat_right = right_fingers_mat[i]
            finger_mat_right = np.dot(right_wrist_mat, finger_mat_right)
            log_transform_axes(f"right_fingers/{i}", finger_mat_right, scale=0.02)

        for i in range(left_fingers_mat.shape[0]):
            finger_mat = left_fingers_mat[i]
            finger_mat = np.dot(left_wrist_mat, finger_mat)
            log_transform_axes(f"left_fingers/{i}", finger_mat, scale=0.02)

        # Optionally, print extra scalar data to the console.
        # print("Right pinch distance:", data['right_pinch_distance'])
        # print("Left pinch distance:", data['left_pinch_distance'])
        # print("Right wrist roll:", data['right_wrist_roll'])
        # print("Left wrist roll:", data['left_wrist_roll'])

        # Small sleep to roughly target a 16 Hz update rate.
        time.sleep(1 / 16)

        if time.time() - start > 500:
            break
    rr.save("hand_tracking")
    streamer.stop_streaming()

def main_avp2robot():
    rr.init("hand_tracking", spawn=True)

    avp_ip = "192.168.101.131"
    streamer = VisionProStreamer(ip=avp_ip, record=False)
    streamer.start_streaming()

    start = time.time()

    while True:
        data = streamer.latest

        global_coordinate = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        log_transform_axes("global", global_coordinate)

        head_mat = data["head"][0]
        head_mat = T_robot_avp @ head_mat
        log_transform_axes("head", head_mat)

        # ====basis convetion transform====:
        # wrist: avp raw data is T_{avp}_{wrist}. Need transform to robot convetion world: T_{robot}_{wrist} = T_{robot}_{avp} * T_{avp}_{wrist}.
        # finger: avp raw data is todo@lufan
        right_wrist_mat = data["right_wrist"][0]
        left_wrist_mat = data["left_wrist"][0]
        T_robot_wrist_right = T_robot_avp @ right_wrist_mat
        T_robot_wrist_left = T_robot_avp @ left_wrist_mat

        log_transform_axes("T_robot_wrist_right", T_robot_wrist_right)
        log_transform_axes("T_robot_wrist_left", T_robot_wrist_left)
        euler_rw = R.from_matrix(T_robot_wrist_right[:3, :3]).as_euler('xyz', degrees=False)
        print("右手欧拉角：", euler_rw)
        # Process finger joints.
        right_fingers_mat = data["right_fingers"]
        left_fingers_mat = data["left_fingers"]

        
        for i in range(right_fingers_mat.shape[0]):
            finger_mat = right_fingers_mat[i]
            T_robot_finger_right =  T_robot_wrist_right @ finger_mat
            log_transform_axes(f"robot_right_fingers/{i}", T_robot_finger_right, scale=0.02)

        for i in range(left_fingers_mat.shape[0]):
            finger_mat = left_fingers_mat[i]
            T_robot_finger_left = T_robot_wrist_left @ finger_mat
            log_transform_axes(f"robot_left_fingers/{i}", T_robot_finger_left, scale=0.02)

        # Optionally, print extra scalar data to the console.
        # print("Right pinch distance:", data['right_pinch_distance'])
        # print("Left pinch distance:", data['left_pinch_distance'])
        # print("Right wrist roll:", data['right_wrist_roll'])
        # print("Left wrist roll:", data['left_wrist_roll'])

        # Small sleep to roughly target a 16 Hz update rate.
        time.sleep(1 / 16)

        if time.time() - start > 500:
            break
    rr.save("hand_tracking")
    streamer.stop_streaming()

def main_avp2booster():
    rr.init("hand_tracking", spawn=True)

    # print("对应旋转矩阵：",R.from_euler('zx', np.array([np.pi, -np.pi / 2]), degrees=False).as_matrix())
    # avp_ip = "192.168.101.131"
    avp_ip = "192.168.200.78"
    streamer = VisionProStreamer(ip=avp_ip, record=False)
    streamer.start_streaming()

    start = time.time()
    data_array = np.empty((0, 13))

    while True:
        data = streamer.latest

        global_coordinate = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        log_transform_axes("global", global_coordinate)

        head_mat = data["head"][0]
        head_mat = T_robot_avp @ head_mat @ T_head_2booster
        log_transform_axes("head", head_mat)

        # ====basis convetion transform====:
        '''
        wrist transform: 
        step1: avp raw data is T_{avp}_{wrist}. Need transform to robot convetion world: T_{robot}_{wrist} = T_{robot}_{avp} * T_{avp}_{wrist}.
        step2: the wrist coordinate in robot is diff from booster, Need right multiply to transform(T_{wrist}_{booster_w} )
            that is T_{robot}_{booster_w} = T_{robot}_{wrist} * T_{wrist}_{booster_w}
        finger transfrom:
        a litter diff from wrist transform:beacuse avp raw finger data is based on wrist, not in ther avp world coordinate. so the transform is:
                T_{robot}_{finger}(booster convetion) = T_{robot}_{booster_w} * T_{booster_w}_{wrist} * T_{wrist}_{finger}
        the reason for left multiply T_{booster_w}_{wrist}: T_{wrist}_{finger} is a relative description calculated without wrist transformation
        '''
        right_wrist_mat = data["right_wrist"][0]
        left_wrist_mat = data["left_wrist"][0]
        T_robot_wrist_right = T_robot_avp @ right_wrist_mat
        T_robot_wrist_left = T_robot_avp @ left_wrist_mat
        T_booster_wrist_right = T_robot_wrist_right @ T_wrist_right_2booster
        T_booster_wrist_left = T_robot_wrist_left @ T_wrist_left_2booster

        log_transform_axes("T_robot_wrist_right", T_booster_wrist_right)
        log_transform_axes("T_robot_wrist_left", T_booster_wrist_left)
        pos_lw = T_booster_wrist_left[:3, 3]
        euler_lw = R.from_matrix(T_booster_wrist_left[:3, :3]).as_euler('xyz', degrees=False)
        pos_rw = T_booster_wrist_right[:3, 3]
        euler_rw = R.from_matrix(T_booster_wrist_right[:3, :3]).as_euler('xyz', degrees=False)
        # print("右手欧拉角：", euler_rw)
        current_time = time.time() - start
        new_data = np.array([[current_time, pos_lw[0], pos_lw[1], pos_lw[2], euler_lw[0], euler_lw[1], euler_lw[2],
                                            pos_rw[0], pos_rw[1], pos_rw[2], euler_rw[0], euler_rw[1], euler_rw[2]]])
        data_array = np.vstack((data_array, new_data))
 
        # Process finger joints.
        right_fingers_mat = data["right_fingers"]
        left_fingers_mat = data["left_fingers"]

        
        for i in range(right_fingers_mat.shape[0]):
            finger_mat = right_fingers_mat[i]
            T_booster_finger_right =  T_robot_wrist_right @ finger_mat 
            log_transform_axes(f"robot_right_fingers/{i}", T_booster_finger_right, scale=0.02)

        for i in range(left_fingers_mat.shape[0]):
            finger_mat = left_fingers_mat[i]
            T_booster_finger_left = T_robot_wrist_left @ finger_mat  
            log_transform_axes(f"robot_left_fingers/{i}", T_booster_finger_left, scale=0.02)

        # Optionally, print extra scalar data to the console.
        # print("Right pinch distance:", data['right_pinch_distance'][0])
        # print("right_middle_finger_thumb_distance", data['right_pinch_distance'][1])
        # print("Left pinch distance:", data['left_pinch_distance'])
        # print("right pinch distance:", data['right_pinch_distance'])

        # print("Right wrist roll:", data['right_wrist_roll'])
        # print("Left wrist roll:", data['left_wrist_roll'])
        # print("right_pinch_distance_middle: ",data['right_pinch_distance_middle'])
        # print("Head:", data["head"][0])
        # print("wrist_rw:", data["right_wrist"][0])
        # print("pos:", data["head"][0][:3, 3] - data["right_wrist"][0][:3, 3])
        # print("left_pos:", data["left_wrist"][0][:3, 3])
        # print("right_pos:", data["right_wrist"][0][:3, 3])

        # print(time_s - time.time())
        # Small sleep to roughly target a 16 Hz update rate, due to rerunn render limits
        time.sleep(1 / 16)

        if time.time() - start > 200:
            break

    save_data(data_array, "hand_tracking_xHz.txt")

    rr.save("hand_tracking")
    streamer.stop_streaming()


def main_avp2booster_test_quest():
    # rr.init("hand_tracking", spawn=True)

    # print("对应旋转矩阵：",R.from_euler('zx', np.array([np.pi, -np.pi / 2]), degrees=False).as_matrix())
    # avp_ip = "192.168.101.131"
    avp_ip = "192.168.200.113"
    streamer = VisionProStreamer(ip=avp_ip, record=False)
    streamer.start_streaming()

    data_array = np.empty((0, 13))
    while True:
        data = streamer.latest

        global_coordinate = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
        # left_w_mat = data["left_wrist"][0]
        # right_w_mat = data["right_wrist"][0]

if __name__ == "__main__":
    # main_avp_raw()
    # main_avp2robot()
    main_avp2booster()
    # main_avp2booster_test_quest()