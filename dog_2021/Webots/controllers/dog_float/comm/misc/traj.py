#   Trajectory formation method
#   Created in 2019 10/23
#   Vision = 1.0
#   Author Junwen Cui

import numpy as np
from scipy.interpolate import interp1d

#脚步悬空时的轨迹生成  摆线轨迹  
#输入参数   现在脚的位置 和 期望脚的位置

def foot_traj(now_pos,ep_pos,high,p_len):

    traj = [[0,0,0] for i in range(p_len)]  #xyz轴的三维轨迹数据

    return  traj




#此函数 获得腿部摆动轨迹 默认使用 cubic 方法
#输入参数   当前脚的坐标 和期望的坐标
#           轨迹高度，点集密集程度
#输出为     点集 x,y,z坐标
def foot_traj2(now_pos,ep_pos,high,p_len = 50):

    if not len(now_pos) == len(ep_pos) == 3:
        raise ValueError('now_pos and ep_pos must be the same shape!')

    p_cal_x = np.array([now_pos[0], (ep_pos[0]+now_pos[0])/2, ep_pos[0]])
    p_cal_y = np.array([now_pos[1], (ep_pos[1]+now_pos[1])/2, ep_pos[1]])

    f_ = interp1d(p_cal_x,p_cal_y,kind = 'cubic')  #'linear','zero', 'slinear', 'quadratic'(2次), 'cubic'(3次), 4, 5

    xx = np.linspace(now_pos[0],ep_pos[0], p_len)
    yy = f_(xx)
    zz = np.linspace(now_pos[2],ep_pos[2], p_len)

    return xx,yy,zz

#此函数 依据功能函数 求得坐标点
#输入参数   当前的点x坐标 及功能函数
#           时候求取斜率 step_acc为x轴精度
#输出为     当前点x，y坐标或者斜率
def cal_traj(x_in,func,slope = False,step_acc = 0.1):

    if step_acc == 0:
        raise ValueError('step_acc can not be zero!')

    if slope :
        y_1 = func(x_in - step_acc/2)
        y_2 = func(x_in + step_acc/2)

        return x_in,(y_2-y_1)/step_acc
    else :
        y_out = func(x_in)
        return x_in,y_out
    

#此函数 获得点集后 求取多种样条函数
#输入参数   已知的点集
#           样条曲线方法  'linear','zero', 'slinear', 'quadratic'(2次), 'cubic'(3次), 4, 5
#输出为 功能函数
def func_traj(data_x,data_y,func = 'cubic'):

    data_x,data_y = np.array(data_x),np.array(data_y)
     
    if not len(data_x) == len(data_y):
        raise ValueError('data_x and data_y must have the same shape!')

    func_out = interp1d(data_x,data_y,kind = func)

    return func_out





