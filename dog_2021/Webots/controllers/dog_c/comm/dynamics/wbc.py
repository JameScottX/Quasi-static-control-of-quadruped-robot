#   Whole Body Control - Hand make
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 

import numpy as np
import numba as nb

if __name__ == '__main__':
    from qp_solver import *
else:
    from comm.dynamics.qp_solver import *
    from comm.action.basis import R_Matrix 


def kd_cal(dis_,speed_des,dis_last,K,D):
    '''
    KD计算函数 使用 numba进行加速
    numba 不支持内部 强制转化为 numpy.array
    输入必须为 np.array
    @nb.njit()   #不使用要快
    '''

    dis_speed = -dis_ + dis_last   #这里的间隔时间 将整合到  D  中  比如 D = D / 0.002
    out_ = K * dis_ +  D * (speed_des - dis_speed) 
    return out_

def skew_sym_matrix(vector):
    '''计算一个向量的斜对称矩阵'''
    return np.mat([[0, -vector[2], vector[1]],
                   [vector[2],0, -vector[0]],
                   [-vector[1], vector[0], 0]])


def force_constrint(output_dim, num, touchstate):
    '''对力进行约束直接对该函数进行修改

    '''
    u = .6 #摩擦系数倒数
    f_z_max = 300 #z轴所能贡献的最大力
    f_z_min = 50   #z轴所能贡献的最小力
    f_min = -50 # x,y 轴最小力 （其实已经被u给约束）
    f_max = 50
    

    cout = 0
    for i in touchstate:
        if i ==1:
            cout += 1

    fric_lim_x = np.array([[1,0,-u],
                           [-1,0,-u]])

    fric_lim_y = np.array([[0,1,-u],
                           [0,-1,-u]])

    z_limit = np.array([[0,0,1], [0,0,-1]])

    b_= np.mat( [0,0,0,0, f_z_max, f_z_min] ).T

    A_ = np.vstack( (fric_lim_x, fric_lim_y, z_limit) )

    zeros_ = np.zeros(np.shape(A_))

    M1__ = np.hstack( (A_, zeros_ ,zeros_ ,zeros_) )
    M2__ = np.hstack( (zeros_, A_ ,zeros_ ,zeros_) )
    M3__ = np.hstack( (zeros_, zeros_ ,A_ ,zeros_) )
    M4__ = np.hstack( (zeros_, zeros_ ,zeros_ ,A_) )

    M = np.vstack( ( M1__, M2__,M3__,M4__ ) )
    gamma = np.vstack( (b_, b_, b_ ,b_) )


    (row, col )= np.shape(A_)

    M = M[:row*cout,:col*cout]
    gamma = gamma[:row*cout]

    return M, gamma


def qp_solve_wbc(leg_pos_nowg, touchstate, Fm, Ft, W_, mg):
    '''
    A*f = b
    优化问题为 (A*f-b).T * (A*f-b) + f.T*w*f

    Fm: 刚体矢量力
    Ft: 刚体转矩

    输出 按照接触腿部id的力矢量

    '''

    touch_id = []
    A_down_i = []
    A_up_i = []

    t = 0
    for i in range(4):
        if touchstate[i] ==1:
            touch_id.append(i)
            A_up_i.append(np.identity(3))
            A_down_i.append(skew_sym_matrix(leg_pos_nowg[i]))
            t+=1

    A_down = np.hstack(A_down_i)
    A_up = np.hstack(A_up_i)
    b = np.vstack((   np.mat(Fm+ np.array([0.,0.,mg])).T   ,   np.mat(Ft).T   ))
    A = np.vstack((   A_up   ,   A_down   ))

    f_dim = np.shape(A)[1]  #获得
    w = np.mat(W_ * np.identity(f_dim))

    E = 2*A.T*A + w
    F = -2*A.T*b

    temp = int(f_dim/3)  #有几条腿着地
    # mg_sep = mg/temp  #分摊重力

    M, gamma = force_constrint(f_dim, temp, touchstate)
    eta, lambda_ = HildrethQPP(E, F, M, gamma)

    eta_ = np.reshape(eta,(temp,3)) 

    for i in range(4):
        if touchstate[i] ==0:
            eta_ = np.insert(eta_, i, values=np.array([0.,0.,0.]), axis=0)

    return np.array(eta_)



def body_pd(real_time, now_state, d_state, d_state_dot, K, D, err_x, err_a):
    '''
    身体刚体PD控制器
    '''

    x_now = np.array(now_state[:3])
    a_now = np.array(now_state[3:])
    x_d = np.array(d_state[:3])
    a_d= np.array(d_state[3:])

    x_d_dot = np.array(d_state_dot[:3])
    a_d_dot = np.array(d_state_dot[3:])

    K_x = K[0]
    K_a = K[1]
    D_x = D[0] / real_time
    D_a = D[1] / real_time
    
    err_x_ = x_d - x_now 
    err_a_ = a_d - a_now
    Fm = kd_cal(err_x_, x_d_dot, np.array(err_x), K_x, D_x)
    Ft = kd_cal(err_a_, a_d_dot, np.array(err_a), K_a, D_a)

    
    return Fm, Ft, err_x_, err_a_


if __name__ == '__main__':
    
    eta_ = qp_solve_wbc([[0.26, 0.13,-0.4], [0.26, -0.13,-0.4], [-0.26, 0.13,-0.4],[-0.26, -0.13,-0.4]], [1,0,1,1], [0,0,0],[0,0,0],1, 16)
    print(eta_)

    err_x, err_a = [10,0,0],[0,0,0]
    Fm, Ft, err_x, err_a = body_pd(0.002, [2,0,0,0,0,0],[3,0,0,0,0,0],[0,0,0,0,0,0],[10,10],[1,0], err_x, err_a)
    print(Fm, Ft, err_x, err_a)



