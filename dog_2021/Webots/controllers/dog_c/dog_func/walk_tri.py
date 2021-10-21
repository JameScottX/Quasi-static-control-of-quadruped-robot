#   dog robot tricycle gait move funcion
#   Updated in 2020 4/27
#   Author Junwen Cui / JameScottX
#   Other: 

import numpy as np
from dog_func.stone import Stone as st
from comm.action.raw import Raw as raw
from comm.dynamics.wbc_f import WBC 
from comm.action.gait_parttern import TriGait
from comm.action.traj_jerk import TRAJ_J
from comm.action.rhythm import Rhy_TriWalk
from comm.action.traj_group import *

# from comm.msg2ros import MSG2ROS

import matplotlib.pyplot as plt


# dr_xyz = data_record(data_name = ['CoM-x','CoM-y'])
# dr_foot_xyz = data_record(data_name = ['fault-x','fault-y'])

# dr_opt = data_record(data_name = ['phi','theta','h'])
# dr_des_curve = [data_record(data_name = ['CoM-x','CoM-y']) for i in range(20)]
# dr_foot_curve = [data_record(data_name = ['fault-x','fault-y']) for i in range(20)]


# msg2ros = MSG2ROS()


class Walk2():
    '''
    四足机器人walk类，包含多种移动步态
    继承 点跟踪类 和 trot节律类
    '''

    def __init__(self,robot):

        #对象 
        self.__robot = robot

        self.body_z = robot.body_normal_z
        
        self.wbc = WBC(robot)
        self.trigait = TriGait(robot.body_normal_z)
        self.traj_j = TRAJ_J()
        self.traj_l = [TRAJ_J() for _ in range(4)]
        self.rhy = Rhy_TriWalk()

        #设置步态序列长度
        self.gait_squence = [ [0.0, 0.08] for _ in range(24) ] 
        self.move_speed_sta_end = [[0.,0.,0.], [0.,0.,0.]]
        #身体移动 时间
        self.move_T = 24
        self.move_t = 0.
        self.move_speed_keep = [0.1*24 /self.move_T,0.,0.]

        #腿 时间
        self.l_t = np.zeros(4,dtype=float)
        self.l_T = 0.4 * np.ones(4,dtype=float)

        #身体旋转 时间
        self.rotat_t = 0.
        self.rotat_T = 1.
        self.rotat_stand_h = -robot.body_normal_z

        self.body_flg = 0
        self.rotat_flag = 0

        self.leg_flg = 0
        self.cross_flg = 0

        self.process = 0 
        self.time_start = [0,0,0]
        
    def body_plan(self): 
        #身体位移规划
        #一个周期结束时，刷新目标点
        index_now = self.trigait.tricycle_peroid_init(self.wbc.foot_p_glb, self.wbc.q2[:3])
        self.trigait.tricycle_foot_locate(self.gait_squence, index_now, self.wbc.foot_p_glb,  self.wbc.q2[:6])
        b_xyz = self.trigait.body_cen_g_loc[:,:3]
       
        b_speed, b_acc = self.traj_j.xyz_speed_default(b_xyz, \
            self.move_speed_sta_end, self.move_speed_keep,1)

        self.traj_j.traj_Xs(b_xyz, b_speed, b_acc, self.move_T)

        self.move_t = 0.
        self.trigait.gait_count = 0  #初始化步态索引

    def body_rotat(self):
        # 身体姿态规划 
        s =  [0, 0.5, 1]
        ds_sta_end = [0.,0.]  #  s的 开始和结束 速度
        T_all =2*np.sum(self.l_T)/4
        ds,dds = self.traj_j.s_speed_default(s,ds_sta_end, 1/T_all)
        self.traj_j.traj_s(s, ds, dds, T_all)
        self.rotat_t = 0.
        self.rotat_T = T_all
        self.rotat_start = self.wbc.q2[3:6] #[0,0,self.wbc.q2[5]]
        # yaw_d = self.trigait.body_cen_g_loc[self.trigait.gait_count][5]

        yaw_d = 0.
        pitch_d = 0.
        roll_d = 0.
        if self.trigait.gait_count < 6:
            pitch_d = 0.3
        elif self.trigait.gait_count > 6 and self.trigait.gait_count <10:
            pitch_d = 0.0
        elif self.trigait.gait_count > 10 and self.trigait.gait_count <18:
            pitch_d = -0.3
        self.rotat_end = [roll_d, pitch_d, yaw_d]


    def leg_plan(self):
        #摆动腿规划
        self.wbc.activate_leg = np.ones(4)
        #此处转化腿为质心 坐标系下 
        swing_tar = self.trigait.gait_foot_locate[self.trigait.gait_count] - self.wbc.q2[:3]
        l_xyz, l_speed, l_acc = traj_3pots(self.wbc.foot_p_com[self.sw_id], swing_tar, self.l_T[self.sw_id])
        self.traj_l[self.sw_id].traj_Xs(l_xyz, l_speed, l_acc, self.l_T[self.sw_id])
        self.l_t = np.zeros(4,dtype=float)

        self.wbc.activate_leg[self.sw_id] = 0

    def leg_change(self):
        # 抬腿规划后 进行着陆判断
        if self.leg_flg == 1:
            ratio = self.l_t[self.sw_id] / self.traj_l[self.sw_id].T
            if ratio > 0.5 and self.__robot.touchstate[self.sw_id] ==1:
                self.trigait.gait_count += 1
                self.leg_flg = 0
                self.rotat_flag = 0


    def cross_diag(self):
        #重心过中线后在进行 腿部轨迹规划 
        if (self.sw_id ==2 or self.sw_id ==3):
            temp = self.rhy.line_dis_j(self.wbc.foot_p_com, self.sw_id)
            if temp >= 0.02:   # 安全裕度
                self.cross_flg = 1
            else:
                self.cross_flg = 0
        elif (self.sw_id ==0 or self.sw_id ==1):
            self.cross_flg = 1

    def main_loop(self, isok = False):

        if not isok:
            return

        # 刷新参数
        self.wbc.wbc_refresh()

        # 几个计时器 
        self.move_t += self.__robot.real_time
        self.l_t += self.__robot.real_time
        self.rotat_t += self.__robot.real_time

        # 身体轨迹规划 和 落脚点规划
        if self.body_flg == 0 or self.trigait.gait_count == 4*5:
            self.body_plan()
            self.body_flg =1

        # 身体 姿态规划 
        if self.rotat_flag == 0 :
            self.body_rotat()
            self.rotat_flag = 1

        # 步态 和 落脚点 规划 
        self.sw_id = self.trigait.gait_foot_index[self.trigait.gait_count]
        # 判断是否 过线
        self.cross_diag()  

        # 腿部运动轨迹规划 
        if self.leg_flg == 0 and self.cross_flg == 1:
            self.leg_plan()
            self.leg_flg = 1

        # 判断是否落地
        self.leg_change()   
        
        s = self.traj_j.traj_ts_f(self.rotat_t)
        rpy_cont = self.wbc.rotate_s(s,  self.rotat_start, self.rotat_end)
        move_cont = self.traj_j.traj_tj_f(self.move_t)
        # move_cont[2] = self.rotat_stand_h
        # print(move_cont)
        body_cont = np.hstack( (move_cont, rpy_cont) )

        # 腿部规划后执行轨迹跟踪
        if self.leg_flg ==1:
            leg_cont  = self.traj_l[self.sw_id].traj_tj_f(self.l_t[self.sw_id])
        else:
            leg_cont = np.zeros(3,dtype=float) 
            self.wbc.activate_leg = np.ones(4)

        # self.wbc.activate_leg = np.ones(4)
        # body_cont = np.hstack( (np.array([0.,0,self.__robot.body_normal_z]), np.array([0,0,0]) ) )
        
        leg_cont = np.array([leg_cont for _ in range(4)])
        # 全身控制
        tau_ff = self.wbc.dynamics_tau_out(body_cont, [0,0,0,0,0,0], leg_cont)
        # WBC 扭矩设置 
        st.wbc_torque_set(self.__robot, tau_ff)



        # try:
        #     if msg2ros.count_rec() == 0:
        #         msg2ros.send_joint(self.wbc.q2[6:])
        #         msg2ros.send_body(self.wbc.q[:7])
        #         msg2ros.send_qp_force(self.wbc.foot_p_com_RT, self.wbc.body_qp_force)
        #         msg2ros.send_foot(self.wbc.foot_p_glb)

        #         msg2ros.send_traj([self.wbc.q2[:3], \
        #             self.wbc.foot_p_glb[0],\
        #             self.wbc.foot_p_glb[1],\
        #             self.wbc.foot_p_glb[2],\
        #             self.wbc.foot_p_glb[3]])
        #         # msg2ros.send_traj(self.wbc.q2[:3])
        #         if self.broken_flag:
        #             temp = np.append( self.wbc.pos_tar_get('BRF_leg1_2_BRF_leg2'), np.array([1,0,1,1]))
        #             msg2ros.send_temp(temp)
        # except:
        #     pass




