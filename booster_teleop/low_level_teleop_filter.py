import logging
import threading
import time
import yaml
import argparse
from booster_robotics_sdk_python import (
    ChannelFactory, 
    B1LocoClient, 
    B1LowCmdPublisher, 
    B1LowStateSubscriber, 
    LowCmd, 
    LowState, 
    B1JointCnt7DofArm,
    RobotMode,
)
from avp_teleop import VisionProStreamer
from avp_map_utility import *

from robot_arm_ik.booster_ik import *
from utils.timer import TimerConfig, Timer
from utils.command import create_prepare_cmd, create_first_frame_rl_cmd, RobotSpec
from utils.remote_control_service import RemoteControlService

SLEEP_TIME = 1

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

# 时序待完成
class Controller:
    def __init__(self, cfg_file):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        # Load config
        with open(cfg_file, "r", encoding="utf-8") as f:
            self.cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

        # Initialize components
        self._init_timer()
        self._init_low_state_values()
        self._init_communication()
        self.remoteControlService = RemoteControlService()
        self.publish_runner = None
        self.running = True

        # Init ik solver
        self.arm_ik = Booster_T1_7dof_ArmIK(Visualization = False, Test = False, Debug = False)
        self.dt = 0.002

    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.cfg["common"]["dt"]))

    def _init_low_state_values(self):
        self.state_q = np.zeros(14, dtype = np.float32)
        self.dof_pos_target = np.zeros(RobotSpec.JointCnt, dtype=np.float32)
        self.dof_tau_target = np.zeros(RobotSpec.JointCnt, dtype=np.float32)
        self.filtered_dof_pos_target = np.zeros(RobotSpec.JointCnt, dtype=np.float32)
        self.arm_joint_data = np.empty((0, 15))
        self.arm_joint_data_filtered = np.empty((0, 15))

    def _init_communication(self) -> None:
        try:
            self.low_cmd = LowCmd()
            self.low_state_subscriber = B1LowStateSubscriber(self._state_feedback_handler)
            self.low_cmd_publisher = B1LowCmdPublisher()
            self.client = B1LocoClient()

            self.low_state_subscriber.InitChannel()
            self.low_cmd_publisher.InitChannel()
            self.client.Init()
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    def log_joint_data(self):
        current_time = time.time()
        data_row = np.array([current_time])
        data_row_filtered = np.array([current_time])
        for i in range(2, 16):
            data_row = np.append(data_row, self.dof_pos_target[i])
            data_row_filtered = np.append(data_row_filtered, self.filtered_dof_pos_target[i])

        self.arm_joint_data = np.vstack((self.arm_joint_data , data_row))
        self.arm_joint_data_filtered = np.vstack((self.arm_joint_data_filtered, data_row_filtered))

    def save_joint_data(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"joint_pos_{timestamp}.txt"
        filename_filtered = f"joint_pos_filtered_{timestamp}.txt"

        header = "# timestamp"
        header_filtered = "# timestamp"
        for i in range(2, 16):
            header +=f"J{i}_q "
            header_filtered += f"J{i}_q_filtered "
        np.savetxt(filename, self.arm_joint_data, header=header.strip(), comments='', fmt='%.6f')
        np.savetxt(filename_filtered, self.arm_joint_data_filtered, header=header_filtered.strip(), comments='', fmt='%.6f')
        self.logger.info(f"Saved {self.arm_joint_data.shape[0]} joint records to {filename}")
        self.logger.info(f"Saved {self.arm_joint_data_filtered.shape[0]} joint records to {filename_filtered}")

    def _state_feedback_handler(self, low_state_msg: LowState):
        self.timer.tick_timer_if_sim()
        for i, motor in enumerate(low_state_msg.motor_state_serial[2:16]):
            self.state_q[i] = motor.q
        # print(f"listen 时间：{time.time()} 计算时间： {self.timer.get_time()}    到状态信息 ：{self.state_q}")

    def _send_cmd(self, cmd: LowCmd):
        # print(f"publish cmd {cmd.motor_cmd[2].q}")
        self.low_cmd_publisher.Write(cmd)

    def start_custom_mode_conditionally(self):
        print(f"{self.remoteControlService.get_custom_mode_operation_hint()}")
        while True:
            if self.remoteControlService.start_custom_mode():
                break
            time.sleep(0.1)
        start_time = time.perf_counter()
        create_prepare_cmd(self.low_cmd, self.cfg)

        self._send_cmd(self.low_cmd)
        send_time = time.perf_counter()
        self.logger.debug(f"Send cmd took {(send_time - start_time)*1000:.4f} ms")
        self.client.ChangeMode(RobotMode.kCustom)
        end_time = time.perf_counter()
        self.logger.debug(f"Change mode took {(end_time - send_time)*1000:.4f} ms")

    def start_rl_gait_conditionally(self):
        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        self._send_cmd(self.low_cmd)
        for i in range(RobotSpec.JointCnt):
            self.dof_pos_target[i] = self.low_cmd.motor_cmd[i].q
            self.filtered_dof_pos_target[i] = self.low_cmd.motor_cmd[i].q

        self.next_inference_time = self.timer.get_time()
        self.next_publish_time = self.timer.get_time()
        self.publish_runner = threading.Thread(target=self._publish_cmd)
        self.publish_runner.daemon = True
        self.publish_runner.start()
        print(f"{self.remoteControlService.get_operation_hint()}")

    def cleanup(self) -> None:
        """Cleanup resources."""
        self.running = False  # Ensure stop flag is set
        self.save_joint_data()

        # Close remote control service
        if hasattr(self, "remoteControlService"):
            try:
                self.logger.info("Closing remote control service...")
                self.remoteControlService.close()
            except Exception as e:
                self.logger.error(f"Error closing remote control service: {e}")
       
        
        # Close communication channels
        if hasattr(self, "low_cmd_publisher"):
            try:
                self.logger.info("Closing low command publisher...")
                self.low_cmd_publisher.CloseChannel()
            except Exception as e:
                self.logger.error(f"Error closing low command publisher: {e}")
        if hasattr(self, "low_state_subscriber"):
            try:
                self.logger.info("Closing low state subscriber...")
                self.low_state_subscriber.CloseChannel()
            except Exception as e:
                self.logger.error(f"Error closing low state subscriber: {e}")

        # Wait for publishing thread to end
        if hasattr(self, "publish_runner") and self.publish_runner is not None:
            try:
                self.logger.info("Publishing thread join...")
                self.publish_runner.join(timeout=1.0)
                if self.publish_runner.is_alive():
                    self.logger.warning("Command publishing thread didn't exit within the time limit")
            except Exception as e:
                self.logger.error(f"Error waiting for thread to end: {e}")

        self.logger.info("Cleanup complete")

    def _publish_cmd(self):
        while self.running:
            time_now = self.timer.get_time()
            if time_now < self.next_publish_time:
                time.sleep(0.001)
                continue
            self.next_publish_time += self.cfg["common"]["dt"] * 5
            self.logger.debug(f"Next publish time: {self.next_publish_time}")

            self.filtered_dof_pos_target = self.filtered_dof_pos_target * 0.8 + self.dof_pos_target * 0.2

            for i in range(RobotSpec.JointCnt):
                self.low_cmd.motor_cmd[i].q = self.filtered_dof_pos_target[i]
                self.low_cmd.motor_cmd[i].tau = self.dof_tau_target[i]

            # # Use series-parallel conversion for torque to avoid non-linearity
            # for i in self.cfg["mech"]["parallel_mech_indexes"]:
            #     self.low_cmd.motor_cmd[i].q = self.dof_pos_latest[i]
            #     self.low_cmd.motor_cmd[i].tau = np.clip(
            #         (self.filtered_dof_pos_target[i] - self.dof_pos_latest[i]) * self.cfg["common"]["stiffness"][i],
            #         -self.cfg["common"]["torque_limit"][i],
            #         self.cfg["common"]["torque_limit"][i],
            #     )
            #     self.low_cmd.motor_cmd[i].kp = 0.0

            start_time = time.perf_counter()
            self.low_cmd_publisher.Write(self.low_cmd)
            # self._send_cmd(self.low_cmd)
            # self.log_joint_data() #比较容易堵塞
            publish_time = time.perf_counter()
            self.logger.debug(f"Publish took {(publish_time - start_time)*1000:.4f} ms")
            time.sleep(0.001)

    def start_teleop(self, init_L_wrist_target, init_R_wrist_target):
        first_q, first_tau = self.arm_ik.solve_ik(init_L_wrist_target.homogeneous, init_R_wrist_target.homogeneous)
        start_time = time.time()
        homing_time = 2
        end_time = start_time + homing_time

        current_time = start_time
        while current_time < end_time:
            current_time = time.time()
            print(f"时间 {current_time}")

            gain = (current_time - start_time) / homing_time
            cmd_q = (1 - gain) * self.state_q + gain * first_q[:14]
            # todo add gravity compensation
            for i in range(2, 16):
                # self.low_cmd.motor_cmd[i].q = cmd_q[i - 2] # todo 看是不是索引对齐的
                self.dof_pos_target[i] = cmd_q[i - 2]
            
            # self._send_cmd(self.low_cmd)
            
 
    def run_teleop(self, L_wrist_target, R_wrist_target):
        sol_q, sol_tau = self.arm_ik.solve_ik(L_wrist_target.homogeneous, R_wrist_target.homogeneous)
        for i in range(2, 16):
            # self.low_cmd.motor_cmd[i].q = sol_q[i - 2]
            # self.low_cmd.motor_cmd[i].tau = sol_tau[i - 2]
            self.dof_pos_target[i] = sol_q[i - 2]
            self.dof_tau_target[i] = sol_tau[i - 2]


        # self._send_cmd(self.low_cmd)

def main():
    import signal

    def signal_handler(sig, frame):
        print("Program fully exited")
        os._exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=str, help="Name of the configuration file.")
    args = parser.parse_args()
    cfg_file = os.path.join("configs", args.config)

    ChannelFactory.Instance().Init(0)
    controller = Controller(cfg_file)

    # receive avp data, retargeting, solve ik, seed cmd
    avp_ip = "192.168.200.128"   
    ctl_T = 0.02

    # = 1： The plane of the robot's gripper is aligned with the human's palm. 
    # = 2： The normal plane of the robot's gripper is aligned with the human's palm.
    tele_mode = 1

    while True:
        input_cmd = input("Please enter the start signal (enter 's' to start the subsequent program):\n")
        if input_cmd.lower() == "s":
            global should_terminate  # 明确声明使用全局变量
            input_thread = threading.Thread(target=listen_for_input)
            input_thread.start()
            print(f"时间： {time.time()}")

            s = VisionProStreamer(ip = avp_ip, record = True)
            s.start_streaming()
            controller.start_custom_mode_conditionally()
            controller.start_rl_gait_conditionally()
            time.sleep(2) # 2s 让人手到目标位置

            latest = s.latest
            init_head_mat = latest['head'][0]
            init_lw_mat = latest['left_wrist'][0]
            init_rw_mat = latest['right_wrist'][0]
            init_booster_T_head = T_robot_avp @ init_head_mat @ T_head_booster
            if tele_mode == 1:
                init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_wrist_left_booster
                init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_wrist_right_booster
            elif tele_mode == 2:
                init_booster_T_wrist_left = T_robot_avp @ init_lw_mat @ T_map_grapper_left @ T_wrist_left_booster 
                init_booster_T_wrist_right = T_robot_avp @ init_rw_mat @ T_map_grapper_right @ T_wrist_right_booster

            # transform to base head from base world(booster convention), only translation
            init_head_T_wrist_left_pos = init_booster_T_wrist_left[:3, 3] - init_booster_T_head[:3, 3]
            init_head_T_wrist_right_pos = init_booster_T_wrist_right[:3, 3] - init_booster_T_head[:3, 3]
            # transform to base body(booster convention), only translation 
            init_body_lw_ori = np.array(init_booster_T_wrist_left[:3, :3])
            init_body_rw_ori = np.array(init_booster_T_wrist_right[:3, :3])
            booster_neck2body_pos = np.array([0.0625, 0.0, 0.30485])
            booster_head2neck_pos = np.array([0.0613, 0.0, 0.086])
            map_gain = 1
            init_body_lw_pos = init_head_T_wrist_left_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos
            init_body_rw_pos = init_head_T_wrist_right_pos * map_gain + booster_head2neck_pos + booster_neck2body_pos
            L_tf_target = pin.SE3(init_body_lw_ori, init_body_lw_pos)
            R_tf_target = pin.SE3(init_body_rw_ori, init_body_rw_pos)

            controller.start_teleop(L_tf_target, R_tf_target)
            time.sleep(1)
            while not should_terminate:
                try:
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
                    if tele_mode == 1:
                        booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_wrist_left_booster
                        booster_T_wrist_right = T_robot_avp @ right_wrist_mat @ T_wrist_right_booster
                    elif tele_mode == 2:
                        booster_T_wrist_left = T_robot_avp @ left_wrist_mat @ T_map_grapper_left @ T_wrist_left_booster 
                        booster_T_wrist_right =  T_robot_avp @ right_wrist_mat @ T_map_grapper_right @ T_wrist_right_booster
                    
                    lw_pos = booster_T_wrist_left[:3, 3]
                    rw_pos = booster_T_wrist_right[:3, 3]
                    lw_body_ori = np.array(booster_T_wrist_left[:3, :3])
                    rw_body_ori = np.array(booster_T_wrist_right[:3, :3])
                    
                    lw_pos_bais = (lw_pos - init_booster_T_wrist_left[:3, 3]) * map_gain
                    rw_pos_bais = (rw_pos - init_booster_T_wrist_right[:3, 3]) * map_gain
                    lw_body_pos = init_body_lw_pos + lw_pos_bais
                    rw_body_pos = init_body_rw_pos + rw_pos_bais

                    L_tf_target = pin.SE3(lw_body_ori, lw_body_pos)
                    R_tf_target = pin.SE3(rw_body_ori, rw_body_pos)

                    controller.run_teleop(L_tf_target, R_tf_target)

                    # print(f"左手： {L_tf_target}")
                    print(f"右手： {R_tf_target}")

                    elapsed_time1 = time.time() - now_time
                    gap_time = ctl_T - elapsed_time1
                    if gap_time < 0:
                        print("超时，剩余时间：", gap_time)
                    if gap_time > 0:               
                        time.sleep(gap_time) 
                except KeyboardInterrupt:
                    print("\nKeyboard interrupt received. Cleaning up...")
                    controller.client.ChangeMode(RobotMode.kDamping)
                    controller.cleanup()
                    break
                
            controller.client.ChangeMode(RobotMode.kDamping)
            controller.cleanup()
            s.stop_streaming()
            print("Loop terminated.")


if __name__ == "__main__":
    main()
