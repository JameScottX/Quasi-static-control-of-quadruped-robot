#   Pace for tracking method
#   Updated in 2019 12/18
#   Vision = 1.0
#   Author Junwen Cui
#   Other: use numba to accelarate

import numpy as np
import numba as nb


#在点轨迹中寻找 下一个点
#此函数 numba 加速反而变慢
#@nb.njit()
def find_nowpoint(points,pos_now,n_dim_):

    num_ = np.shape(points)[0]
    small_dir = 30000.
    small_index = 0
    for i in range(num_):
     
        temp = np.sum(np.abs(points[i] - pos_now))

        if small_dir >= temp:
            small_dir = temp
            small_index = i
    
    if small_index >= num_-1:
        small_index = num_-1
    else:
        small_index+=1
    
    return small_index,small_dir




#微分-速度控制函数
#输入参数为   点集 1Xn  单位 m
#             当前点位置 1X3   单位 m
#             期望速度  单位 m/s
#             微分周期  单位 s
#返回         下一个位置  1X3 array   
def pace2next(points,pos_now,speed_d,period,last_pos,**kwargs):

    n_dim_ = 3
    K = 10
    for name, value in kwargs.items():
        if name == 'n_dim_': n_dim_ = value
        elif name == 'K' : K = value

    points,pos_now,last_pos = np.array(points),np.array(pos_now),np.array(last_pos)

    if np.shape(points)[1] != np.shape(pos_now)[0] != n_dim_:
        raise ValueError('pace2next has the shape error!')

    index_,_ = find_nowpoint(points,pos_now,n_dim_) #寻找目标点索引

    dir_ = points[index_] - pos_now   #矢量差 只确定了方向 非切线 点到点
    #print(dir_)
    dir_sum = np.sqrt(np.sum( np.square(dir_)))

    new_speed = period * speed_d / dir_sum * dir_               #期望速度矢量

    last_peed = (pos_now - last_pos)/period                     #上次的速度矢量

    pos_new = pos_now + K * (new_speed - last_peed)

    return pos_new









    












