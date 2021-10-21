#   Basic motion methods
#   Updated in 2019 12/20
#   Vision = 2.0
#   Author Junwen Cui
#   Other: use numba to accelarate

import numpy as np
import numba as nb

class Pos:

    def __init__(self, a = 0, b = 0, c = 0):

        self.x = a
        self.y = b
        self.z = c

class Speed:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.x = a
        self.y = b
        self.z = c

class Force:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.x = a
        self.y = b
        self.z = c

class Torque:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.swing = a
        self.thign = b
        self.calf = c

class Attitude:

    def __init__(self, a = 0, b = 0, c = 0):

        self.roll = a
        self.pitch = b
        self.yaw = c


#旋转矩阵计算 使用 numba进行加速
#输入参数为   roll pitch yaw
#返回         数组 由于np.mat不被支持           
@nb.njit()    #加速两倍
def R_Matrix(theat,zhong,alph):
        #x轴旋转角 theat
        #y轴旋转角 zhong
        #z轴旋转角 alph
        R_ = np.array([[                                   np.cos(alph)*np.cos(zhong),                                  -np.sin(alph)*np.cos(zhong),             np.sin(zhong)],
                     [ np.sin(alph)*np.cos(theat) + np.cos(alph)*np.sin(theat)*np.sin(zhong), np.cos(alph)*np.cos(theat) - np.sin(alph)*np.sin(theat)*np.sin(zhong), -np.cos(zhong)*np.sin(theat)],
                     [ np.sin(alph)*np.sin(theat) - np.cos(alph)*np.cos(theat)*np.sin(zhong), np.cos(alph)*np.sin(theat) + np.sin(alph)*np.cos(theat)*np.sin(zhong),  np.cos(theat)*np.cos(zhong)]])
    
        return R_

#旋转逆矩阵计算 使用 numba进行加速
#输入参数为   roll pitch yaw
#返回         数组 由于np.mat不被支持   
@nb.njit()    #加速两倍
def R_Matrix_inv(theat,zhong,alph):
        #x轴旋转角 theat
        #y轴旋转角 zhong
        #z轴旋转角 alph
    R_inv = np.array([[  np.cos(alph)*np.cos(zhong), np.sin(alph)*np.cos(theat) + np.cos(alph)*np.sin(theat)*np.sin(zhong), np.sin(alph)*np.sin(theat) - np.cos(alph)*np.cos(theat)*np.sin(zhong)],
                      [ -np.sin(alph)*np.cos(zhong), np.cos(alph)*np.cos(theat) - np.sin(alph)*np.sin(theat)*np.sin(zhong), np.cos(alph)*np.sin(theat) + np.sin(alph)*np.cos(theat)*np.sin(zhong)],
                      [            np.sin(zhong),                                 -np.cos(zhong)*np.sin(theat),                                  np.cos(theat)*np.cos(zhong)]])

    return R_inv



#限幅函数
def trans2max(value, min, max):  

    if(value > max):
        value = max
    if(value < min):
        value = min

    return value
#反向运动学 输出 单位 rad
def IK(leg, module_info):

    pos_ = Pos()

    pos_.x = leg.position_f.x
    pos_.y = leg.position_f.y  
    pos_.z = leg.position_f.z

    #以下为重心高度  此处不使用   腿部伸长高度依据髋关节坐标
    body_height  = module_info['body_height'] 

    Length1 = module_info['swing_lenth']
    Length2 = module_info['thign_lenth']
    Length3 = module_info['calf_lenth']
    
    y_z = np.square(pos_.y) + np.square(pos_.z)
    x_y_z = np.square(pos_.x) + y_z
    
    l1 = np.square(Length2)
    l2 = np.square(Length3)
    
    D = 1

    if pos_.z > 0: 
        D = 1
    else:
        D = -1

    leg.swing_angle_f = D * np.arccos(np.abs(pos_.y)/np.sqrt(y_z))
    leg.thign_angle_f = np.arccos((x_y_z + l1 -l2)/(2*np.sqrt(x_y_z)*Length2)) + np.arcsin(pos_.x/np.sqrt(x_y_z))
    leg.calf_angle_f = -np.arccos((x_y_z-l1-l2)/(2*Length2*Length3))

#正向运动学 输出 单为 m
def DK(leg, module_info):

    Length1 = module_info['swing_lenth'] 
    Length2 = module_info['thign_lenth'] 
    Length3 = module_info['calf_lenth'] 
    #以下为重心高度  此处不使用   腿部伸长高度依据髋关节坐标
    body_height  = module_info['body_height'] 

    theta1 = leg.swing_angle
    theta2 = leg.thign_angle
    theta3 = leg.calf_angle
    
    pos = Pos()

    pos.x = Length2*np.sin(theta2) + Length3*np.cos(theta2)*np.sin(theta3) + Length3*np.cos(theta3)*np.sin(theta2)
    pos.y = Length3*np.sin(theta3)*np.cos(theta1)*np.sin(theta2) - Length2*np.cos(theta1)*np.cos(theta2) - Length3*np.cos(theta3)*np.cos(theta1)*np.cos(theta2) - Length1*np.cos(theta1)
    pos.z = Length1*np.sin(theta1) + Length2*np.cos(theta2)*np.sin(theta1) + Length3*np.cos(theta3)*np.cos(theta2)*np.sin(theta1) - Length3*np.sin(theta3)*np.sin(theta1)*np.sin(theta2)

    leg.position = pos


#反向扭矩计算 输出 单为 Nm
def torque2force(leg, module_info):
    
    L1 = module_info['swing_lenth'] 
    L2 = module_info['thign_lenth'] 
    L3 = module_info['calf_lenth'] 

    theta1 = leg.swing_angle
    theta2 = leg.thign_angle
    theta3 = leg.calf_angle


    a = [0, 
         np.sin(np.conj(theta1))*np.conj(L1) + np.cos(np.conj(theta2))*np.sin(np.conj(theta1))*np.conj(L2) + np.cos(np.conj(theta2))*np.cos(np.conj(theta3))*np.sin(np.conj(theta1))*np.conj(L3) - np.sin(np.conj(theta1))*np.sin(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3), 
         np.cos(np.conj(theta1))*np.conj(L1) + np.cos(np.conj(theta1))*np.cos(np.conj(theta2))*np.conj(L2) + np.cos(np.conj(theta1))*np.cos(np.conj(theta2))*np.cos(np.conj(theta3))*np.conj(L3) - np.cos(np.conj(theta1))*np.sin(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3)]

    b = [ np.cos(np.conj(theta2))*np.conj(L2) + np.cos(np.conj(theta2))*np.cos(np.conj(theta3))*np.conj(L3) - np.sin(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3),                              
          np.cos(np.conj(theta1))*np.sin(np.conj(theta2))*np.conj(L2) + np.cos(np.conj(theta1))*np.cos(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3) + np.cos(np.conj(theta1))*np.cos(np.conj(theta3))*np.sin(np.conj(theta2))*np.conj(L3),
          - np.sin(np.conj(theta1))*np.sin(np.conj(theta2))*np.conj(L2) - np.cos(np.conj(theta2))*np.sin(np.conj(theta1))*np.sin(np.conj(theta3))*np.conj(L3) - np.cos(np.conj(theta3))*np.sin(np.conj(theta1))*np.sin(np.conj(theta2))*np.conj(L3)]

    c = [np.cos(np.conj(theta2))*np.cos(np.conj(theta3))*np.conj(L3) - np.sin(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3),
         np.cos(np.conj(theta1))*np.cos(np.conj(theta2))*np.sin(np.conj(theta3))*np.conj(L3) + np.cos(np.conj(theta1))*np.cos(np.conj(theta3))*np.sin(np.conj(theta2))*np.conj(L3),                                                                           
         - np.cos(np.conj(theta2))*np.sin(np.conj(theta1))*np.sin(np.conj(theta3))*np.conj(L3) - np.cos(np.conj(theta3))*np.sin(np.conj(theta1))*np.sin(np.conj(theta2))*np.conj(L3)]
 
    J = np.matrix([a, b, c])

    force = np.matrix([leg.force_f.x, leg.force_f.y, leg.force_f.z])

    torque = J.dot(force.T)

    leg.torque_f.swing = torque[0]
    leg.torque_f.thign = torque[1]
    leg.torque_f.calf = torque[2]


