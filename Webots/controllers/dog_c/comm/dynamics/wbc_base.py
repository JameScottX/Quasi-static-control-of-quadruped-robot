#   Basic WBC file
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 


import numpy as np
from scipy.spatial.transform import Rotation as R

def ddq_group(robot):

    temp2 = robot.ddattitude.draw()
    temp1 = np.array([0,0,-9.81])

    ddq_body = np.hstack( (temp1, temp2))
    
    ddq = np.array([0., 0., 0.])

    ddq_leg = np.hstack(( ddq, ddq, ddq, ddq ))

    return np.hstack((ddq_body, ddq_leg))

def dq_group(robot):
    # 速度量整理 
    temp1 = robot.gps_speed.draw()
    temp2 = robot.dattitude.draw()
    # temp1 = [0,0,0]
    # temp2 = [0,0,0]
    dq_body = np.hstack( (temp1,temp2))
    
    dq1 = robot.Leg_lf.dangle.draw()
    dq2 = robot.Leg_rf.dangle.draw()
    dq3 = robot.Leg_lb.dangle.draw()
    dq4 = robot.Leg_rb.dangle.draw()

    dq_leg = np.hstack(( dq1, dq2, dq3, dq4 ))
    # print(dq_leg)
    # print(dq_body)
    return np.hstack((dq_body, dq_leg))

def q_group(robot):
    # 位置和角度 整合 
    xyz = robot.gps_pos_now.draw()
    rpy = robot.attitude.draw()

    temp2 = [rpy[2],rpy[1],rpy[0]]
    r = R.from_euler('zyx', temp2)
    temp2 = r.as_quat()

    q_body = np.hstack( (xyz,temp2))
    q_body2  = np.hstack( (xyz,rpy))

    q1 = robot.Leg_lf.angle.draw()
    q2 = robot.Leg_rf.angle.draw()
    q3 = robot.Leg_lb.angle.draw()
    q4 = robot.Leg_rb.angle.draw()

    q_leg = np.hstack(( q1, q2, q3, q4 ))
    # print(q_leg)
    # print('body',q_body)

    return np.hstack((q_body, q_leg)), np.hstack((q_body2, q_leg))

def q_body_q_leg(q_body, q_leg, onlyadd = False ):
    # 返回 q 组成 
    if np.shape(q_body)[0] == 7 or onlyadd:
        return np.hstack( (q_body, q_leg) )
    elif np.shape(q_body)[0] == 6:
        rpy = q_body[3:]
        temp2 = [rpy[2],rpy[1],rpy[0]]
        r = R.from_euler('zyx', temp2)
        temp2 = r.as_quat()
        return np.hstack( (q_body[:3], temp2, q_leg ) )
    else:
        print(' q_body_q_leg error ! ')


def force_constrint(touchstate):
    '''对力进行约束直接对该函数进行修改

        C1 * f <= [f_max,f_max,f_z_max].T
        C2 * f >= [f_min, f_min, f_z_min].T

    '''
    u = .4 #摩擦系数倒数
    f_z_max = 500 #z轴所能贡献的最大力
    f_z_min = 2   #z轴所能贡献的最小力
    f_min = -50 # x,y 轴最小力 （其实已经被u给约束）
    f_max = 50
    
    # C1 = np.mat([ [u_, 0, -1],
    #               [0, u_, -1],
    #               [0, 0, 1]   ])
    # _right = np.mat([ 0, 0, f_z_max]).T

    fric_lim_x = np.array([[1,0,-u],
                           [-1,0,-u]],dtype=np.float64)

    fric_lim_y = np.array([[0,1,-u],
                           [0,-1,-u]],dtype=np.float64)

    z_limit = np.array([[0,0,1], [0,0,-1]],dtype=np.float64)

    h = np.mat( [0.,0.,0.,0., f_z_max, -f_z_min], dtype=np.float64).T

    G = np.vstack( (fric_lim_x, fric_lim_y, z_limit) )
    
    cout = 0
    for i in touchstate:
        if i ==1:
            cout += 1

    zeros_ = np.zeros(np.shape(G))

    G1__ = np.hstack( (G, zeros_ ,zeros_ ,zeros_) )
    G2__ = np.hstack( (zeros_, G ,zeros_ ,zeros_) )
    G3__ = np.hstack( (zeros_, zeros_ ,G ,zeros_) )
    G4__ = np.hstack( (zeros_, zeros_ ,zeros_ ,G) )

    G__ = np.vstack( ( G1__, G2__,G3__,G4__ ) )
    h__ = np.vstack( (h, h, h ,h) )

    (row,col )= np.shape(G)


    G = G__[:row*cout,:col*cout]
    h = h__[:row*cout]

    return G,h

def suport_body_martix(touchstate,foot_pos):
    # WBC A矩阵构建 
    touch_id = []
    A_down_i = []
    A_up_i = []

    t = 0
    for i in range(4):
        if touchstate[i] ==1:
            touch_id.append(i)
            A_up_i.append(np.identity(3))
            A_down_i.append(skew_sym_matrix(foot_pos[i]))
            t+=1

    A_down = np.hstack(A_down_i)
    A_up = np.hstack(A_up_i)
    A = np.vstack((   A_up   ,   A_down   ))

    return np.mat(A)



def skew_sym_matrix(vector):
    '''计算一个向量的斜对称矩阵'''
    return np.mat([[0, -vector[2], vector[1]],
                   [vector[2],0, -vector[0]],
                   [-vector[1], vector[0], 0]], dtype=np.float64)







