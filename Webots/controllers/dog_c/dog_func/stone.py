#   dog robot move funcion
#   Updated in 2020 4/27
#   Author Junwen Cui / JameScottX
#   Other: 


import numpy as np
import sys
# sys.path.append('/home/jame/dog_2020/Webots/controllers/dog_c/')
from comm.action.raw import Raw as rw
from comm.action.basis import Force,Pos, Angle, R_Matrix, R_Matrix_inv, R_Matrix_yaw, vallimit
from comm.dynamics.new_vmc import force_out_sepa,force_out_2


class Stone():
    error_last = [[0 for j in range(3)] for i in range(4)]

    one_fault_flag = 0 
    one_fault_reme = 0.
    @classmethod
    def leg_spring(cls, robot, id_, next_pot, f_add = np.array([0,0,0]), K = 20000 , D = 20):
        '''腿部弹簧'''
        D_ = D / robot.real_time
        cls.error_last[id_],force_new = rw.foot_force_out(robot.leg_pos_now[id_],next_pot,
                                         cls.error_last[id_],K,D_)
        force_new += np.array(f_add)
        return force_new


    @classmethod
    def pots_set(cls, robot, pots):
        '''腿部末端位置设置函数'''
        pos_ = Pos()
        pos_.assig(pots[0])
        robot.Leg_lf.set_position(pos_)
        pos_.assig(pots[1])
        robot.Leg_rf.set_position(pos_)
        pos_.assig(pots[2])
        robot.Leg_lb.set_position(pos_)
        pos_.assig(pots[3])
        robot.Leg_rb.set_position(pos_)


    @classmethod
    def angle_set(cls, robot, angs):
        ang = Angle()
        ang.assig(angs[0])
        robot.Leg_lf.set_angle(ang)
        ang.assig(angs[1])
        robot.Leg_rf.set_angle(ang)
        ang.assig(angs[2])
        robot.Leg_lb.set_angle(ang)
        ang.assig(angs[3])
        robot.Leg_rb.set_angle(ang)

    @classmethod
    def force_set(cls, robot, force, inverse = False):

        if inverse:
            force = -np.array(force)
        force_ = Force()
        force_.assig(force[0])
        robot.Leg_lf.set_force(force_)
        force_.assig(force[1])
        robot.Leg_rf.set_force(force_)
        force_.assig(force[2])
        robot.Leg_lb.set_force(force_)
        force_.assig(force[3])
        robot.Leg_rb.set_force(force_)

    @classmethod
    def wbc_torque_set(cls, robot, torque):
        
        toq_trans = []
        for i in range(4):
            toq_trans.append( np.array(torque[6+3*i:6+3*(i+1), ]).T[0] )
        robot.Leg_lf.set_torque(toq_trans[0])
        robot.Leg_rf.set_torque(toq_trans[1])
        robot.Leg_lb.set_torque(toq_trans[2])
        robot.Leg_rb.set_torque(toq_trans[3])


    @classmethod
    def wbc_torque_set_onefault(cls, robot, torque):
        #单关节 损失 扭矩设置函数
        if cls.one_fault_flag==0:
            cls.one_fault_flag =1
            cls.one_fault_reme = robot.Leg_rf.angle.thign

        toq_trans = []
        for i in range(4):
            toq_trans.append( np.array( torque[6+3*i:6+3*(i+1),]).T[0] )
        
        toq_trans = np.array(toq_trans)

        swing_torque = vallimit(float(toq_trans[1][0]), -49, 49)
        thign_torque = vallimit(float(toq_trans[1][1]), -49, 49)
        calf_torque = vallimit(float(toq_trans[1][2]), -49, 49)

        robot.Leg_lf.set_torque(toq_trans[0])

        robot.Leg_rf.swing_motor.setTorque(swing_torque)
        # robot.Leg_rf.thign_motor.setTorque(thign_torque)
        robot.Leg_rf.calf_motor.setTorque(calf_torque)
        robot.Leg_rf.thign_motor.setPosition(float(cls.one_fault_reme))
        # print(robot.Leg_rf.angle.thign)

        
        robot.Leg_lb.set_torque(toq_trans[2])
        robot.Leg_rb.set_torque(toq_trans[3])

    @classmethod
    def force_set_R_T(cls, robot, force, is_GRF = False):

        force_mat = np.mat(force).T
        force_ = np.mat(R_Matrix_inv(robot.attitude.roll,robot.attitude.pitch,robot.attitude.yaw)) * force_mat
        force_GRF = np.array(-force_.T)
        cls.force_set(robot,force_GRF)

    @classmethod
    def pd_control(cls, err, err_last, K, D):

        temp_  = err - err_last
        out_ = K * err +  D * temp_
        return out_


        