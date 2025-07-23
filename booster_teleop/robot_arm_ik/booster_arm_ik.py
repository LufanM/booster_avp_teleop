import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
from pinocchio import casadi as cpin    
from pinocchio.visualize import MeshcatVisualizer  
import os
import sys

class Booster_T1_7dof_ArmIK:
    def __init__(self, Visualization = False, Test = False, Debug = False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Visualization = Visualization
        self.Debug = Debug
        self.Test = Test
        if self.Test:
            self.robot = pin.RobotWrapper.BuildFromURDF('./assets/T1_7_dof_arm_serial_with_head_arm_update_limit.urdf', './assets')
        else:
            self.robot = pin.RobotWrapper.BuildFromURDF('./robot_arm_ik/assets/T1_7_dof_arm_serial_with_head_arm_update_limit.urdf', './robot_arm_ik/assets')

        self.mixed_jointsToLockIDs = [
                                        "AAHead_yaw",
                                        "Head_pitch",
                                        "Left_Hip_Pitch",
                                        "Left_Hip_Roll",
                                        "Left_Hip_Yaw",
                                        "Left_Knee_Pitch",
                                        "Left_Ankle_Pitch",
                                        "Left_Ankle_Roll",
                                        "Right_Hip_Pitch",
                                        "Right_Hip_Roll",
                                        "Right_Hip_Yaw",
                                        "Right_Knee_Pitch",
                                        "Right_Ankle_Pitch",
                                        "Right_Ankle_Roll",
                                        "Waist"
                                    ]


        self.initialJointConfig = np.array([0.0] * self.robot.model.nq)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=self.initialJointConfig,
        )

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        print(f"reduced_robot joint number is: {self.reduced_robot.model.nq}")

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("Left_Hand_Roll")
        self.R_hand_id = self.reduced_robot.model.getFrameId("Right_Hand_Roll")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3, 3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3, 3]
                )
            ],
        )

        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf_l = self.opti.parameter(4,4)
        self.param_tf_r = self.opti.parameter(4,4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )

        self.opti.minimize(40 * self.translational_cost + 1 * self.rotation_cost + 0.1 * self.regularization_cost + 0.1 * self.smooth_cost)
        # self.opti.minimize(50 * self.translational_cost + 0.5 * self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)


        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-10
            },
            'print_time':False,# print cal time or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)

        self.vis = None
        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=[101, 102], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = ['L_ee_target', 'R_ee_target']
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 10
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )

        if self.Debug:
            self.model = pin.Model(self.reduced_robot.model)
            self.data = self.model.createData()
            self.L_eef_id = self.model.getFrameId("Left_Hand_Roll")
            self.R_eef_id = self.model.getFrameId("Right_Hand_Roll")

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)
        if self.Visualization:
            self.vis.viewer['L_ee_target'].set_transform(left_wrist)   # for visualization
            self.vis.viewer['R_ee_target'].set_transform(right_wrist)  # for visualization

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)

            
            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            if self.Debug:
                stats = self.opti.stats()
                # 获取目标函数值
                obj_value = self.opti.value(self.opti.f)   

                trans_error_vec = casadi.sumsqr(self.translational_error(sol_q, left_wrist, right_wrist))
                rot_error_vec = casadi.sumsqr(self.rotational_error(sol_q, left_wrist, right_wrist))

                pin.forwardKinematics(self.model, self.data, sol_q)
                pin.updateFramePlacements(self.model, self.data)

                eef_l = self.data.oMf[self.L_eef_id]
                eef_r = self.data.oMf[self.R_eef_id]

                # rpy格式转换
                eef_l_rpy = np.concatenate([eef_l.translation, pin.rpy.matrixToRpy(eef_l.rotation)])
                eef_r_rpy = np.concatenate([eef_r.translation, pin.rpy.matrixToRpy(eef_r.rotation)])

                # print(f"计算的当前关节重力矩为：{sol_tauff}")
                # print(f"位置误差: {trans_error_vec} m^2 ")
                # print(f"旋转误差: {rot_error_vec} rad^2 ")
                # print(f"目标位姿： {left_wrist},   右手：{right_wrist}")
                # print(f"ik解位姿： 左={eef_l_rpy}, 右 = {eef_r_rpy}")
                print(f"目标函数值: {obj_value}    迭代次数： {stats['iter_count']}")
                # print(f"sol_q 的关节角：  {sol_q}")

            if self.Visualization:
                self.vis.display(sol_q)  # for visualization
  

            return sol_q, sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
        

if __name__ == "__main__":
    # test
    arm_ik = Booster_T1_7dof_ArmIK(Visualization = True, Test = True, Debug = True)

    # initial positon  
    pos1_l = [0.32, 0.39, 0.1, -1.57, -1., 0.]
    pos1_r = [0.32, -0.39, 0.1, 1.57, -1., 0.0,]


    Rot_l = pin.utils.rpyToMatrix(pos1_l[3], pos1_l[4], pos1_l[5])
    Rot_r = pin.utils.rpyToMatrix(pos1_r[3], pos1_r[4], pos1_r[5])

    L_tf_target = pin.SE3(
        Rot_l,
        np.array(pos1_l[:3]),
    )

    R_tf_target = pin.SE3(
        Rot_r,
        np.array(pos1_r[:3]),
    )

    times = []
    for _ in range(10):
        start_time = time.time()
        arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)
        # 计算耗时（秒）
        elapsed_time = (time.time() - start_time) * 1000
        times.append(elapsed_time)
        time.sleep(0.1)


    print(f"最小耗时: {min(times):.3f} ms")
    print(f"最大耗时: {max(times):.3f} ms")
    print(f"平均耗时: {sum(times)/len(times):.3f} ms")
    print(f"时间 序列为：{times}")

