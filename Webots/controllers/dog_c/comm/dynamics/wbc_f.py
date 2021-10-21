#   Whole Body Control
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 

import sys
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from  comm.dynamics.wbc_base import *
from cvxopt import solvers, matrix
solvers.options['show_progress'] = False

class WBC(object):

    def __init__(self, robot):
        self.__robot = robot

        model_path = "./urdf/"
        srdf_path = model_path + "dog_c.srdf"

        self.pin_robot = RobotWrapper.BuildFromURDF(model_path + "dog_c.urdf", \
            [model_path], pin.JointModelFreeFlyer())
        self.rmodel = self.pin_robot.model
        # pin.loadReferenceConfigurations(self.rmodel, srdf_path, False)
        self.rdata = self.rmodel.createData()
        # 输出模型 信息 
        self.show_model()

        # 获得 腿部 和 身体 框架  ID 
        self.frame_foot_name = ['ALF_foot','BRF_foot','CLH_foot','DRH_foot']
        self.foot_id = []
        self.body_id = self.rmodel.getFrameId('body')
        for i in range(4):
            name = self.frame_foot_name[i]
            self.foot_id.append(self.rmodel.getFrameId(name))
        # WBC PD 
        self.body_k = np.array([4000 for _ in range(3)] + [20000 for _ in range(3)] ,dtype=np.float64)
        self.body_d = np.array([200 for _ in range(3)] + [300 for _ in range(3)] ,dtype=np.float64)
        # QP 优化得到分配力
        self.body_qp_force = np.array( [ [0.,0.,0.] for i in range(4)] ,dtype=np.float64)
        # 惩罚因子
        self.R = 1.0  
        # 腿部末端 在标准质心及全局框架下 
        self.foot_p_com = np.array( [ [0.,0.,0.] for i in range(4)] ,dtype=np.float64)
        # self.foot_p_com_RT = np.array( [ [0.,0.,0.] for i in range(4)] ,dtype=np.float64)
        self.foot_p_glb = np.array( [ [0.,0.,0.] for i in range(4)] ,dtype=np.float64)
        # 腿部功能激活 1 落地 0 抬腿
        self.activate_leg = [1 for _ in range(4)]
        # 摆动腿 位置 及 PD 参数
        self.swing_p_base = np.array( [ [0.0,0.0,0.0] for i in range(4)] ,dtype=np.float64)
        self.swleg_k = np.array(50 ,dtype=np.float64)
        self.swleg_d = np.array(0.7 ,dtype=np.float64)
        # 摆动腿 添加额外力 控制量
        self.foot_add_touchdown = np.zeros(4)

    def show_model(self):
        # 显示模型信息 
        for f in self.rmodel.frames:
            print(f.name, '   ', f.parent)

    def wbc_refresh(self):
        # 刷新 刚体及关节
        self.q, self.q2 = q_group(self.__robot)
        self.dq = dq_group(self.__robot)
        self.ddq = ddq_group(self.__robot)
        # 前向运动学   4*3  foot_p_com 已经在质心标准坐标系下
        self.foot_p_com, self.foot_p_glb, self.foot_p_com_aligned = self.foot_pos_get() 
        # 无用 
        # self.foot_p_com_RT = self.rotate_p_com(self.q2[3:6], self.foot_p_com)

    def dynamics_tau_out(self, q_d, dq_d, sw_pos):
        # body期望 
        self.q_d = q_d
        self.dq_d = dq_d
        # 摆动腿期望 已进行矫正
        self.swing_p_base = self.rotate_p_base([self.q2[3], self.q2[4], 0], sw_pos) 
        # 动力学第二项 
        # self.h = pin.rnea(self.rmodel, self.rdata, self.q, self.dq, self.ddq)
        # 惯性矩阵 
        self.M = pin.crba(self.rmodel, self.rdata, self.q)

        tau_b = self.rigid_body()
        tau_sw = self.swing_leg()

        add_  = self.touchdown_force()
        tau_sw += add_

        S  = [np.zeros(6)]
        for t in range(4):
            if self.activate_leg[t] == 0:
                S.append(np.zeros(3))
            else:
                S.append(np.ones(3))
        S = np.hstack( S )
        S = np.mat(np.diag(S))
        # print(S)
        # print(tau_sw)
        # print(tau_b)
        tau_ff = tau_sw - tau_b  #+ S*tau_ff
        return tau_ff

    def rigid_body(self):
        # 简化刚体 WBC 控制器
        M_body = np.array(self.M[0:6,:])
        # h_body = np.array(self.h[0:6])

        # ddq_ = np.array(self.ddq[0:6])
        q_  = np.array(self.q2[0:6])
        dq_ = np.array(self.dq[0:6])

        ext_part = self.pd_cal(q_, dq_, self.q_d, self.dq_d, self.body_k, self.body_d)
        # print(ext_part)
        ddq_tar = ext_part
        ddq_new = np.hstack( (ddq_tar, self.ddq[6:]) )

        tau_b =  np.mat(M_body) * np.mat(ddq_new).T 

        ext_tau = np.mat( [0.0 for i in range(18)] ).T
        if np.sum(self.activate_leg) == 0:   #没有支撑腿时
            return ext_tau
        # WBC A矩阵构建 
        A = suport_body_martix(self.activate_leg, self.foot_p_com_aligned)  
        F_dim = np.shape(A)[1]
        R = self.R * np.identity(F_dim)
        # 约束项 
        G,h_ = force_constrint(self.activate_leg)
        # QP 优化 
        F = self.cvx_opt(A,R,tau_b,G,h_)
        # 对于摆动腿 末端力为 0 
        insert_ = np.zeros((3,1), dtype = np.float64)
        for t in range(4):   #  补 抬腿 0 操作 
            if self.activate_leg[t] == 0:
                F = np.insert(F,(t)*3,insert_,axis = 0)
        # 记录输出力  在质心坐标系下 
        self.body_qp_force = np.array(F)
        # 矢量力转扭矩
        ext_tau = self.contact_leg_f2tau(F)

        #这里对 非支撑腿 和姿态维持力 清0
        # S  = [np.zeros(6)]
        # for t in range(4):
        #     if self.activate_leg[t] == 0:
        #         S.append(np.zeros(3))
        #     else:
        #         S.append(np.ones(3))
        # S = np.hstack( S )
        # S = np.diag(S)

        # tau_ff = S * tau_ff
        # print(tau_ff)
        return ext_tau

    def cvx_opt(self,A,R,tau_b,G,h):
        # CVX 来优化 QP 问题 
        P_temp = np.array((A.T*A + R)).T
        cvx_P = matrix( P_temp )

        q_temp = np.array((-2*tau_b.T * A))[0]
        cvx_q = matrix( q_temp )

        G_temp = np.array(G.T)
        cvx_G = matrix( G )

        h_temp = np.array(h.T)[0]
        cvx_h = matrix( h_temp )
        F_out = solvers.qp(cvx_P, cvx_q, G=cvx_G,h=cvx_h)

        return np.array(F_out['x'])

    def contact_leg_f2tau(self, force):
        # 接触腿 矢量力转关节扭矩 
        # CoM frame
        tau_q = np.mat([0.0 for _ in range(18)],dtype=np.float64).T
        for i in range(4):
            pin.computeFrameJacobian(self.rmodel, self.rdata, self.q, self.foot_id[i])
            #LOCAL 代表本地坐标系
            #WORLD 代表世界坐标系
            #LOCAL_WORLD_ALIGNED 本地对齐到世界
            pin.updateFramePlacement(self.rmodel, self.rdata, self.foot_id[i])
            Ji = pin.getFrameJacobian(self.rmodel, self.rdata ,self.foot_id[i],\
                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            force_ = force[3*i:3*(i+1),]
            #补上 转矩控制为0
            insert_ = np.zeros((3,1), dtype = np.float64)
            force_ = np.insert(force_,3,insert_,axis = 0)   
            f_ = np.mat(force_)
            Ji_ = np.mat(Ji).T
            tau_q += Ji_ * f_
        
        #选择矩阵
        S = np.hstack( (np.zeros(6) , np.ones(12) ))
        S = np.diag(S)

        return S * tau_q   #去除姿态部分

    def foot_pos_get(self):
        # 足端位置获得 
        # CoM frame, Global frame and aligned CoM frame
        #必须要计算的
        pin.computeJointJacobians(self.rmodel, self.rdata,self.q)
        #使用frame 必须要计算的
        # pin.framesForwardKinematics(self.rmodel, self.rdata,self.q)
        #在要求获得 frame 速度和 加速度时使用
        pin.forwardKinematics(self.rmodel, self.rdata, self.q, self.dq, self.ddq)

        foot_p = []
        for i in range(self.__robot.leg_num):
            pin.updateFramePlacement(self.rmodel, self.rdata, self.foot_id[i])
            foot_p.append(self.rdata.oMf[self.foot_id[i]].translation)

        foot_p = np.array(foot_p)
        pin.updateFramePlacement(self.rmodel, self.rdata, self.body_id)
        body_p = self.rdata.oMf[self.body_id].translation

        # 这里需要注意 脚的位置已经在质心标准坐标系下 
        foot_p_com_aligned = foot_p - body_p
        foot_p_com = self.rotate_p_base([0,0,self.q2[5]], foot_p_com_aligned)

        return np.array(foot_p_com), np.array(foot_p), np.array(foot_p_com_aligned)
    
    def tar_pos_get_com(self, name):
        # 目标 frame 位置获得  
        # CoM frame
        #必须要计算的
        pin.computeJointJacobians(self.rmodel, self.rdata,self.q)
        #使用frame 必须要计算的
        # pin.framesForwardKinematics(self.rmodel, self.rdata,self.q)
        #在要求获得 frame 速度和 加速度时使用
        pin.forwardKinematics(self.rmodel, self.rdata, self.q, self.dq, self.ddq)
        id_tar = self.rmodel.getFrameId(name)
        pin.updateFramePlacement(self.rmodel, self.rdata, id_tar)
        pin.updateFramePlacement(self.rmodel, self.rdata, self.body_id)

        tar_p = self.rdata.oMf[id_tar].translation
        body_p = self.rdata.oMf[self.body_id].translation

        return self.rotate_p_base([0,0,self.q2[5]],tar_p - body_p)

    def swing_leg(self):
        # 摆动腿跟踪期望值 通过 PD 获得扭矩
        # Base frame
        ext_part = np.array([0.0 for _ in range(12)])
        if np.sum(self.activate_leg) != 4:  #有一条腿不是支撑腿时
            leg_q_d = np.array([0.0 for _ in range(12)])
            for i in range(4):
                if self.activate_leg[i] == 0:
                    temp = self.foot_inverse_kine(i,self.swing_p_base[i])   #反向运动学
                    leg_q_d[3*i:3*(i+1)] = np.array(temp[3*i:3*(i+1)])
                    # leg_q_d[3*i:3*(i+1)] = np.array([0., -0.5,1])
                    
            q_sw = self.q[7:]
            dq_sw = self.dq[6:]
            q_swd = leg_q_d
            dq_swd = np.array([0.0 for _ in range(12)])   #阻尼

            ext_part = self.pd_cal2(q_sw, dq_sw, q_swd, dq_swd, self.swleg_k, self.swleg_d)
          
        #选择矩阵
        ext_part = np.hstack( (np.zeros(6) , ext_part ))
        #这里对 支撑腿 和姿态维持力 清0
        S  = [np.zeros(6)]
        for t in range(4):
            if self.activate_leg[t] == 0:
                S.append(np.ones(3))
            else:
                S.append(np.zeros(3))
        S = np.hstack( S )
        S = np.diag(S)
        tau_sw = np.dot(S,ext_part)
        return np.mat(tau_sw).T

    def foot_inverse_kine(self, index,  d_pos):
        # 伪逆求腿部逆向运动学 
        # Base frame
        foot_id = self.foot_id[index]
        DT = self.__robot.real_time
        damp   = 0.00001
        eps    = 0.002
        d_pos = np.array(d_pos)

        com_frame = [0.,0.,0.,0.,0.,0.,1.]
        new_q = q_body_q_leg(com_frame, self.q2[6:] )

        pin.forwardKinematics(self.rmodel, self.rdata, new_q)
        pin.updateFramePlacement(self.rmodel, self.rdata, foot_id)
        J = pin.computeFrameJacobian(self.rmodel, self.rdata,new_q,foot_id,\
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        Jx = J[0:3,6:]  # 排除末端姿态  排除身体位置和姿态

        temp_q = np.array(self.q)[7:]
        for i in range(60):
            err = d_pos - self.rdata.oMf[foot_id].translation
            v =  Jx.T.dot(np.linalg.solve(Jx.dot(Jx.T) + damp * np.eye(3), err))
            temp_q +=0.1*v
            if np.linalg.norm(err)< eps:
                break
            com_frame = [0.,0.,0.,0.,0.,0.,1.]
            new_q = q_body_q_leg(com_frame, temp_q )
            pin.forwardKinematics(self.rmodel, self.rdata, new_q)
            pin.updateFramePlacement(self.rmodel, self.rdata, foot_id)
            J = pin.computeFrameJacobian(self.rmodel, self.rdata,new_q,foot_id,\
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Jx = J[0:3,6:]  # 排除末端姿态  排除身体位置和姿态
        # print('foot_inverse_kine', i)
        return temp_q
       
    def pd_cal(self,q,dq,q_d,dq_d,K,D):
        # WBC PD控制器 
        out_ = K[:3] * (q_d[:3]-q[:3]) +  D[:3] * (dq_d[:3] - dq[:3])
        diff_rotate = self.rotate_tans(q[3:],q_d[3:])
        out_2 = K[3:] * diff_rotate +  D[3:] * (dq_d[3:] - dq[3:])
        return np.hstack( (out_, out_2) )

    def pd_cal2(self,q,dq,q_d,dq_d,K,D ):
        # 原始 PD 控制器 
        out_ = K * (q_d - q) + D * (dq_d - dq)
        return out_

    def foot_J_get(self, index):
        # 获得腿部末端 雅克比矩阵
        foot_id = self.foot_id[index]
        pin.forwardKinematics(self.rmodel, self.rdata, self.q)
        pin.updateFramePlacement(self.rmodel, self.rdata, foot_id)
        J = pin.computeFrameJacobian(self.rmodel, self.rdata,self.q,foot_id,\
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        Jx = J[0:3, 6+ index*3 : 9 + index*3]  # 排除末端姿态  排除身体位置和姿态
        return Jx

    def touchdown_force(self, force = [0.,0., -5]):
        # 摆动腿 添加 额外力
        force_add = np.hstack( [force for _ in range(4)] )
        force_add = np.mat(force_add).T
        tau_add = self.contact_leg_f2tau(force_add)

        S  = [np.zeros(6)]
        for t in range(4):
            if self.foot_add_touchdown[t] == 1:
                S.append(np.ones(3))
            else:
                S.append(np.zeros(3))
        S = np.hstack( S )
        S = np.diag(S)

        return S * tau_add

    def rotate_tans(self, q, q_d):
        #角度差异转到 旋转矩阵 空间中操作 
        R_q = pin.rpy.rpyToMatrix(np.array(q))
        R_q_d = pin.rpy.rpyToMatrix(np.array(q_d))
        diff = pin.rpy.matrixToRpy( np.dot(R_q_d, R_q.T) )
        return diff

    def rotate_s(self, s, rpy_sta, rpy_end):
        #姿态角 变化过渡  s = 0~1
        R_sta = pin.rpy.rpyToMatrix(np.array(rpy_sta))
        R_end = pin.rpy.rpyToMatrix(np.array(rpy_end))
        R_now = np.dot(R_sta, pin.exp( pin.log(   np.dot(R_sta.T, R_end) *s  ) ) )
        rpy_now = pin.rpy.matrixToRpy(R_now)
        return rpy_now

    def rotate_p_com(self, q, pos):
        #将轨迹 转换到 CoM frame 下 
        q_new = np.array([q[0], q[1], q[2]])
        R_q = pin.rpy.rpyToMatrix(q_new)
        pos_T = np.array(pos).T
        return np.dot(R_q,pos_T).T

    def rotate_p_base(self, q, pos):
        #将轨迹 转换到 Base frame 下
        q_new = np.array([q[0], q[1], q[2]])
        R_q = pin.rpy.rpyToMatrix(q_new)
        pos_T = np.array(pos).T
        return np.dot(R_q.T,pos_T).T


    def pin_test(self):


        name = 'ALF_foot'
        id = self.rmodel.getFrameId(name)
        pin.computeJointJacobians(self.rmodel, self.rdata,self.q)

        #使用frame 必须要计算的
        pin.framesForwardKinematics(self.rmodel, self.rdata,self.q)

        #在要求获得 frame 速度和 加速度时使用
        # pin.forwardKinematics(self.rmodel, self.rdata, self.q, self.dq, self.ddq)
        # pin.updateFramePlacement(self.rmodel, self.rdata, id_)
      
        pin.computeFrameJacobian(self.rmodel, self.rdata, self.q, id)
        #LOCAL 代表本地坐标系
        #WORLD 代表世界坐标系
        #LOCAL_WORLD_ALIGNED 本地对齐到世界
        Ji = pin.getFrameJacobian(self.rmodel, self.rdata ,id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        force = [0,0,100]
        # speed_q = np.mat([ 0,0,0, 0,0,0, 0,0,10, 0,0,0, 0,0,0, 0,0,0]).T
        f_ = np.mat(force).T
        Ji_ = np.mat(Ji).T
        print(Ji_ * f_)













