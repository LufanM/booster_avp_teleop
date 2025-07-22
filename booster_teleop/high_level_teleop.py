from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture, GripperMotionParameter, Quaternion, Frame, Transform, DexterousFingerParameter
import sys, time, threading
from avp_teleop import VisionProStreamer
from typing import * 
import numpy as np
from avp_map_utility import *
from scipy.spatial.transform import Rotation as R

# 定义一个全局变量用于线程间通信
should_terminate = False

# 监听用户输入的函数
def listen_for_input():
    global should_terminate
    termination_command = "quit"
    while True:
        input_str = input()
        if input_str == termination_command:
            should_terminate = True
            break

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    client = B1LocoClient()
    client.Init()
    x, y, z, yaw, pitch = 0.0, 0.0, 0.0, 0.0, 0.0

    res = 0

    while True:
        need_print = False
        input_cmd = input().strip()
        if input_cmd:
            if input_cmd == "mp":
                res = client.ChangeMode(RobotMode.kPrepare)
            elif input_cmd == "md":
                res = client.ChangeMode(RobotMode.kDamping)
            elif input_cmd == "mw":
                res = client.ChangeMode(RobotMode.kWalking)
            elif input_cmd == 'mc':
                res = client.ChangeMode(RobotMode.kCustom)
            elif input_cmd == "stop":
                x, y, z = 0.0, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "mhel":
                tar_posture = Posture()
                tar_posture.position = Position(0.35, 0.25, 0.1)
                tar_posture.orientation = Orientation(-1.57, -1.57, 0.0)
                res = client.MoveHandEndEffector(tar_posture, 2000, B1HandIndex.kLeftHand)
            elif input_cmd == "gopenl":
                motion_param = GripperMotionParameter()
                motion_param.position = 1000
                motion_param.force = 100
                motion_param.speed = 2000
                res = client.ControlGripper(motion_param, GripperControlMode.kForce, B1HandIndex.kRightHand)
            elif input_cmd == "gclosel":
                motion_param = GripperMotionParameter()
                motion_param.position = 300
                motion_param.force = 1200
                motion_param.speed = 2000
                res = client.ControlGripper(motion_param, GripperControlMode.kForce, B1HandIndex.kRightHand)
            elif input_cmd == "gft":
                src = Frame.kBody
                dst = Frame.kHead

                transform: Transform = Transform()
                res = client.GetFrameTransform(src, dst, transform)
                if res == 0:
                    quat = np.array([transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w])
                    rpy = R.from_quat(quat).as_euler('xyz', degrees=False)
                    print(f"Transform pos: {transform.position.x} {transform.position.y} {transform.position.z}")
                    print(f"Transform ori: {rpy}")
            elif input_cmd == "hcm-start":
                res = client.SwitchHandEndEffectorControlMode(True)
            elif input_cmd == "hcm-stop":
                res = client.SwitchHandEndEffectorControlMode(False)
            elif input_cmd == "teleop":
                    avp_ip = "192.168.200.45"   
                    ctl_T = 0.025

                    terminate_pinch = 1 
                    global should_terminate  # 明确声明使用全局变量
                    input_thread = threading.Thread(target=listen_for_input)
                    input_thread.start()

                    l_motion_param = GripperMotionParameter()
                    l_motion_param.position = 500
                    l_motion_param.force = 100
                    l_motion_param.speed = 2000
                    r_motion_param = GripperMotionParameter()
                    r_motion_param.position = 500
                    r_motion_param.force = 100
                    r_motion_param.speed = 2000

                    s = VisionProStreamer(ip = avp_ip, record = True)
                    s.start_streaming()
                    time.sleep(2.5) # 2s 让人手到目标位置
                    # 记录AVP中获得的初始位姿
                    latest = s.latest
                    init_head_mat = latest['head'][0]
                    init_lw_mat = latest['left_wrist'][0]
                    init_rw_mat = latest['right_wrist'][0]
                    init_booster_T_head = T_robot_avp @ init_head_mat @ T_head_booster
                    # init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_wrist_left_booster
                    # init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_wrist_right_booster
                    init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_map_grapper_left @ T_wrist_left_booster 
                    init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_map_grapper_right @ T_wrist_right_booster 
                    
                    # transform to base head from base world(booster convention), only translation
                    init_head_T_wrist_left_pos = init_booster_T_wrist_left[:3, 3] - init_booster_T_head[:3, 3]
                    init_head_T_wrist_right_pos = init_booster_T_wrist_right[:3, 3] - init_booster_T_head[:3, 3]

                    # transform to base body(booster convention), only translation 
                    init_lw_ori_rpy = R.from_matrix(init_booster_T_wrist_left[:3, :3]).as_euler('xyz', degrees = False)
                    init_rw_ori_rpy = R.from_matrix(init_booster_T_wrist_right[:3, :3]).as_euler('xyz', degrees = False)
                    booster_neck2body_pos = np.array([0.0625, 0.0, 0.30485])
                    booster_head2neck_pos = np.array([0.0613, 0.0, 0.086])
                    # map_gain = 0.476 / 0.54 # 0.54 is arm length of operator, 0.476 is the robot's 需要调整根据不同的人，主要目的是减少映射偏差
                    map_gain = 1
                    init_body_lw_pos = init_head_T_wrist_left_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos
                    init_body_rw_pos = init_head_T_wrist_right_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos
                    
                    # 机器人对应的初始位置，遥操作开始前，前2s,先让人手与robot的动作匹配上
                    init_robot_posture_lw = Posture()
                    init_robot_posture_lw.orientation = Orientation(init_lw_ori_rpy[0], init_lw_ori_rpy[1], init_lw_ori_rpy[2])
                    init_robot_posture_lw.position = Position(init_body_lw_pos[0], init_body_lw_pos[1], init_body_lw_pos[2]) 
                    init_robot_posture_rw = Posture()
                    init_robot_posture_rw.orientation = Orientation(init_rw_ori_rpy[0], init_rw_ori_rpy[1], init_rw_ori_rpy[2])
                    init_robot_posture_rw.position = Position(init_body_rw_pos[0], init_body_rw_pos[1], init_body_rw_pos[2])

                    res = client.MoveHandEndEffector(init_robot_posture_lw, 2000, B1HandIndex.kLeftHand)
                    res = client.MoveHandEndEffector(init_robot_posture_rw, 2000, B1HandIndex.kRightHand) # todo 控头
                    time.sleep(5)

                    print_time = 0 
                    start = time.time()
                    data_array = np.empty((0, 13))
                    max_test_time = 0
                    while not should_terminate:
                        now_time = time.time()
                        latest = s.latest
                        l_pinch = latest['left_pinch_distance']
                        r_pinch = latest['right_pinch_distance']
                        rock_v = latest['right_pinch_distance_middle']
                        terminate_pinch = r_pinch[1]
                        if terminate_pinch < 0.015:
                            print("quit teleoperation !!!")
                            break
                        
                        head_mat= latest['head'][0]
                        l_pinch_d = l_pinch[0]
                        r_pinch_d = r_pinch[0]
                        left_wrist_mat = latest['left_wrist'][0]
                        right_wrist_mat = latest['right_wrist'][0]
                        booster_T_head = T_robot_avp @ head_mat @ T_head_booster
                        # booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_wrist_left_booster
                        # booster_T_wrist_right = T_robot_avp @ right_wrist_mat @ T_wrist_right_booster
                        booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_map_grapper_left @ T_wrist_left_booster 
                        booster_T_wrist_right =  T_robot_avp @ right_wrist_mat @ T_map_grapper_right @ T_wrist_right_booster 

                        # # ====Control Method 1: ====
                        # #     The teleoperation command are based on the AVP data transferred to the base torso. 
                        # #     Transform to base head from base world(booster convention),only makes the translation
                        # booster_T_wrist_left[:3, 3] = booster_T_wrist_left[:3, 3] - booster_T_head[:3, 3]
                        # booster_T_wrist_right[:3, 3] = booster_T_wrist_right[:3, 3] - booster_T_head[:3, 3]
                        # # booster_T_head_lw = fast_mat_inv(booster_T_head) @ booster_T_wrist_left
                        # # booster_T_head_rw = fast_mat_inv(booster_T_head) @ booster_T_wrist_right
                        # # transform to base body(booster convention), only makes the translation
                        # lw_ori_rpy = R.from_matrix(booster_T_wrist_left[:3, :3]).as_euler('xyz', degrees = False)
                        # rw_ori_rpy = R.from_matrix(booster_T_wrist_right[:3, :3]).as_euler('xyz', degrees = False)
                        # lw_pos = booster_T_wrist_left[:3, 3] * map_gain + booster_head2neck_pos + booster_neck2body_pos
                        # rw_pos = booster_T_wrist_right[:3, 3] * map_gain + booster_head2neck_pos + booster_neck2body_pos

                        # # print("左手当前的ori", lw_ori_rpy, "对应的旋转矩阵为：", lw_ori)
                        # # print("左手当前的ori的偏差", lw_ori_bais, "\n")
                        # # print("右手当前的posture：", now_right_wrist_pose, "\n")
                        # # print("右手当前的ori", rw_ori_rpy, "对应的旋转矩阵为：", rw_ori)
                        # # print("右手当前的pos", pos, "对应的距离向量为：", now_right_wrist_pose[0][:3, 3], "\n")
                        # # print("右手当前的ori的偏差", rw_ori_bais, "\n")
                        # # print("右手当前的pos的偏差", rw_pos_bais, "\n")
                        
                        # tar_posture_lw = Posture()
                        # tar_posture_lw.orientation = Orientation(lw_ori_rpy[0], lw_ori_rpy[1], lw_ori_rpy[2])
                        # tar_posture_lw.position = Position(lw_pos[0], lw_pos[1], lw_pos[2])
                        # tar_posture_rw = Posture()
                        # tar_posture_rw.orientation = Orientation(rw_ori_rpy[0], rw_ori_rpy[1], rw_ori_rpy[2])
                        # tar_posture_rw.position = Position(rw_pos[0], rw_pos[1], rw_pos[2])
                        
                        # res = client.MoveHandEndEffector(tar_posture_lw, 20, B1HandIndex.kLeftHandTele) # 20ms更新一个数据流
                        # res = client.MoveHandEndEffector(tar_posture_rw, 20, B1HandIndex.kRightHandTele) 

                        # ==== Control Method 2: ====
                        #      The teleoperation command are based on Init posture + Deviation. So first, it should align the init posture of eef between the robot and avp data. 
                        #      In order to control the head 
                        lw_pos = booster_T_wrist_left[:3, 3]
                        rw_pos = booster_T_wrist_right[:3, 3]
                        lw_ori_rpy = R.from_matrix(booster_T_wrist_left[:3, :3]).as_euler('xyz', degrees = False)
                        rw_ori_rpy = R.from_matrix(booster_T_wrist_right[:3, :3]).as_euler('xyz', degrees = False)

                        lw_pos_bais = (lw_pos - init_booster_T_wrist_left[:3, 3]) * map_gain
                        rw_pos_bais = (rw_pos - init_booster_T_wrist_right[:3, 3]) * map_gain

                        tar_posture_lw = Posture()
                        tar_posture_lw.orientation = Orientation(lw_ori_rpy[0], lw_ori_rpy[1], lw_ori_rpy[2])
                        tar_posture_lw.position = Position(init_body_lw_pos[0] + lw_pos_bais[0], 
                                                            init_body_lw_pos[1] + lw_pos_bais[1], 
                                                            init_body_lw_pos[2] + lw_pos_bais[2])
                        tar_posture_rw = Posture()
                        tar_posture_rw.orientation = Orientation(rw_ori_rpy[0], rw_ori_rpy[1], rw_ori_rpy[2])
                        tar_posture_rw.position = Position(init_body_rw_pos[0] + rw_pos_bais[0], 
                                                            init_body_rw_pos[1] + rw_pos_bais[1], 
                                                            init_body_rw_pos[2] + rw_pos_bais[2])
                        res = client.MoveHandEndEffector(tar_posture_lw, ctl_T * 1000, B1HandIndex.kLeftHandTele) # 20ms更新一个数据流
                        res = client.MoveHandEndEffector(tar_posture_rw, ctl_T * 1000, B1HandIndex.kRightHandTele) 
                        head_ori_rpy = R.from_matrix(booster_T_head[:3, :3]).as_euler('xyz', degrees = False)
                        tar_posture_head = Posture()
                        tar_posture_head.orientation = Orientation(head_ori_rpy[0], head_ori_rpy[1], head_ori_rpy[2])
                        res = client.MoveHandEndEffector(tar_posture_head, ctl_T * 1000, B1HandIndex.kHeadTele) # 头控制

                        if rock_v < 0.22:
                            l_motion_param.position = 50
                            l_motion_param.force = 1200
                            res = client.ControlGripper(l_motion_param, GripperControlMode.kForce, B1HandIndex.kLeftHand)
                            # print("夹住")
                        else:
                            l_motion_param.position = 1000
                            l_motion_param.force = 100
                            res = client.ControlGripper(l_motion_param, GripperControlMode.kForce, B1HandIndex.kLeftHand)
                            # print("松开")
                        if rock_v < 0.22:
                            r_motion_param.position = 50
                            r_motion_param.force = 1200
                            res = client.ControlGripper(r_motion_param, GripperControlMode.kForce, B1HandIndex.kRightHand)
                            # print("夹住")
                        else:
                            r_motion_param.position = 1000
                            r_motion_param.force = 100
                            res = client.ControlGripper(r_motion_param, GripperControlMode.kForce, B1HandIndex.kRightHand)

                        elapsed_time1 = time.time() - now_time
                        print_time += elapsed_time1
                        if(print_time > 0.02):
                        #     # print("头部数据：", now_head_pose)
                        #     print("右手当前的ori的偏差", rw_ori_bais)
                        #     print("右手当前的pos的偏差", rw_pos_bais, "\n")
                            print("头部p:", head_ori_rpy[1], "   y:", head_ori_rpy[2])
                            print_time -= 0.02

                        # record data
                        # test_start = time.time()
                        new_data = np.array([[0, lw_pos[0], lw_pos[1], lw_pos[2], lw_ori_rpy[0], lw_ori_rpy[1], lw_ori_rpy[2],
                                             rw_pos[0], rw_pos[1], rw_pos[2], rw_ori_rpy[0], rw_ori_rpy[1], rw_ori_rpy[2]]])
                        current_time = time.time() - start # unaccurate, for reference only 
                        new_data[0, 0] = current_time
                        data_array = np.vstack((data_array, new_data))
                        if(current_time > 300): # 遥操时间
                            print("统计完成：")
                            break
                        # test_end = time.time()
                        # print("存数据耗时：", test_end - test_start)

                        elapsed_time2 = time.time() - now_time
                        gap_time = ctl_T - elapsed_time2
                        if gap_time < 0:
                            print("超时，剩余时间：", gap_time)
                        if gap_time > 0:               
                            time.sleep(gap_time) 

                    # 等待输入线程结束
                    save_data(data_array, 'local_sdk_data_20ms.txt')
                    print("sdk 传输时间最大为：", max_test_time)

                    input_thread.join()
                    print("Loop terminated.")
                    should_terminate = False

            if need_print:
                print(f"Param: {x} {y} {z}")
                print(f"Head param: {pitch} {yaw}")

            if res != 0:
                print(f"Request failed: error = {res}")

from robot_arm_ik.booster_ik import *

def test_pin_ik():
    arm_ik = Booster_T1_7dof_ArmIK(Visualization = True, Test = False, Debug = True)
    avp_ip = "192.168.200.128"   
    ctl_T = 0.02


    while True:
        input_cmd = input("Please enter the start signal (enter 's' to start the subsequent program):\n")
        if input_cmd.lower() == "s":
            s = VisionProStreamer(ip = avp_ip, record = True)
            s.start_streaming()
            time.sleep(2) # 2s 让人手到目标位置

            latest = s.latest
            init_head_mat = latest['head'][0]
            init_lw_mat = latest['left_wrist'][0]
            init_rw_mat = latest['right_wrist'][0]
            init_booster_T_head = T_robot_avp @ init_head_mat @ T_head_booster
            init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_wrist_left_booster
            init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_wrist_right_booster
            # init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_map_grapper_left @ T_wrist_left_booster 
            # init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_map_grapper_right @ T_wrist_right_booster 

            # transform to base head from base world(booster convention), only translation
            init_head_T_wrist_left_pos = init_booster_T_wrist_left[:3, 3] - init_booster_T_head[:3, 3]
            init_head_T_wrist_right_pos = init_booster_T_wrist_right[:3, 3] - init_booster_T_head[:3, 3]

            # transform to base body(booster convention), only translation 
            init_body_lw_ori = np.array(init_booster_T_wrist_left[:3, :3])
            init_body_rw_ori = np.array(init_booster_T_wrist_right[:3, :3])
            booster_neck2body_pos = np.array([0.0625, 0.0, 0.30485])
            booster_head2neck_pos = np.array([0.0613, 0.0, 0.086])
            # map_gain = 0.476 / 0.54 # 0.54 is arm length of operator, 0.476 is the robot's 需要调整根据不同的人，主要目的是减少映射偏差
            map_gain = 1
            init_body_lw_pos = init_head_T_wrist_left_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos
            init_body_rw_pos = init_head_T_wrist_right_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos

            # 添加pin ik可视化代码
            L_tf_target = pin.SE3(init_body_lw_ori, init_body_lw_pos)
            R_tf_target = pin.SE3(init_body_rw_ori, init_body_rw_pos)
            
            arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)

            time.sleep(3)
            while not should_terminate:
                now_time = time.time()
                latest = s.latest

                r_pinch = latest['right_pinch_distance']
                terminate_pinch = r_pinch[1]
                if terminate_pinch < 0.015:
                    print("quit teleoperation !!!")
                    break

                head_mat= latest['head'][0]
                left_wrist_mat = latest['left_wrist'][0]
                right_wrist_mat = latest['right_wrist'][0]
                booster_T_head = T_robot_avp @ head_mat @ T_head_booster
                booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_wrist_left_booster
                booster_T_wrist_right = T_robot_avp @ right_wrist_mat @ T_wrist_right_booster
                # booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_map_grapper_left @ T_wrist_left_booster 
                # booster_T_wrist_right =  T_robot_avp @ right_wrist_mat @ T_map_grapper_right @ T_wrist_right_booster 

                # =================== Control Method 2: ============
                #      The teleoperation command are based on Init posture + Deviation. So first, it should align the init posture of eef between the robot and avp data. 
                #      In order to control the head 
                lw_pos = booster_T_wrist_left[:3, 3]
                rw_pos = booster_T_wrist_right[:3, 3]
                lw_body_ori = np.array(booster_T_wrist_left[:3, :3])
                rw_body_ori = np.array(booster_T_wrist_right[:3, :3])

                lw_pos_bais = (lw_pos - init_booster_T_wrist_left[:3, 3]) * map_gain
                rw_pos_bais = (rw_pos - init_booster_T_wrist_right[:3, 3]) * map_gain
                lw_body_pos = init_body_lw_pos + lw_pos_bais
                rw_body_pos = init_body_rw_pos + rw_pos_bais

                # 添加pin ik可视化代码
                L_tf_target = pin.SE3(lw_body_ori, lw_body_pos)
                R_tf_target = pin.SE3(rw_body_ori, rw_body_pos)
                # print(f"左手： {L_tf_target}")
                # print(f"右手： {R_tf_target}")
                arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)

                elapsed_time1 = time.time() - now_time
                gap_time = ctl_T - elapsed_time1
                if gap_time < 0:
                    print("超时，剩余时间：", gap_time)
                if gap_time > 0:               
                    time.sleep(gap_time) 


if __name__ == "__main__":
    # main()
    test_pin_ik()