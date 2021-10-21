# -*- coding: utf-8 -*-

#   Quadruped robots main function
#   Created in 2020 8/2
#   Vision = 1.1
#   坐标系已经纠正 依据右手坐标系

import sys

import numpy as np
from controller import *
from comm.action.basis import Force,Attitude,Pos
from comm.base.leg import Leg


#webots模型参数名称
module_name = {'lf':{'swing_motor_s': 'lf_h_1_m', 'thignleg_motor_s': 'lf_h_2_m', 'calfleg_motor_s': 'lf_h_3_m', 
                     'swing_positionsensor_s': 'lf_h_1_ps', 'thign_positionsensor_s': 'lf_h_2_ps', 'calf_positionsensor_s': 'lf_h_3_ps', 
                     'foot_touchsensor_s': 'lf_touch'}, 
               
               'lb':{'swing_motor_s': 'lb_h_1_m', 'thignleg_motor_s': 'lb_h_2_m', 'calfleg_motor_s': 'lb_h_3_m', 
                     'swing_positionsensor_s': 'lb_h_1_ps', 'thign_positionsensor_s': 'lb_h_2_ps', 'calf_positionsensor_s': 'lb_h_3_ps', 
                     'foot_touchsensor_s': 'lb_touch'}, 
               
               'rf':{'swing_motor_s': 'rf_h_1_m', 'thignleg_motor_s': 'rf_h_2_m', 'calfleg_motor_s': 'rf_h_3_m', 
                     'swing_positionsensor_s': 'rf_h_1_ps', 'thign_positionsensor_s': 'rf_h_2_ps', 'calf_positionsensor_s': 'rf_h_3_ps', 
                     'foot_touchsensor_s': 'rf_touch'}, 
               
               'rb':{'swing_motor_s': 'rb_h_1_m', 'thignleg_motor_s': 'rb_h_2_m', 'calfleg_motor_s': 'rb_h_3_m', 
                     'swing_positionsensor_s': 'rb_h_1_ps', 'thign_positionsensor_s': 'rb_h_2_ps', 'calf_positionsensor_s': 'rb_h_3_ps', 
                     'foot_touchsensor_s': 'rb_touch'},
               'imu': 'imu',
               'gps':'gps',
               'gyro':'gyro'
           
               }


#模型基本参数 单位 m 0.026
module_info = {'virtual_height':0.02,'body_height':0.04,'swing_lenth': 0, \
               'thign_lenth': 0.320, 'calf_lenth': 0.320, 'body_width': 0.360, 'body_length': 0.520,\
               'freq_ms': 2,}

class Dog(Supervisor):

    def __init__(self):

        super(Dog, self).__init__()
        self.freq = module_info['freq_ms']  # ms
        self.real_time = self.freq / 1000  #  s
        self.time = 0
        self.leg_num = 4
        
        self.attitude = Attitude()  #机身姿态 roll pitch yaw
        self.dattitude = Attitude()
        self.dattitude_last = Attitude()
        self.ddattitude = Attitude()

        self.imu = InertialUnit(module_name['imu'])
        self.imu.enable(self.freq)

        self.gps = GPS(module_name['gps'])
        self.gps.enable(self.freq)
        
        self.gyro = Gyro(module_name['gyro'])
        self.gyro.enable(self.freq)

        self.gps_pos_now = Pos()   #最后矫正坐标
        self.gps_pos = Pos()       #此处 gps 为矫正的初始化坐标
        self.gps_pos_last = Pos()
        self.gps_pos_bias = Pos()  #矫正 gps临时变量
        self.gps_speed = Pos()

        #腿部属性初始化
        self.Leg_lf = Leg(self,module_name['lf'], module_info, 'LF' )
        self.Leg_lb = Leg(self,module_name['lb'], module_info, 'LB')
        self.Leg_rf = Leg(self,module_name['rf'], module_info, 'RF')
        self.Leg_rb = Leg(self,module_name['rb'], module_info, 'RB')

        #**************Robot attributes**************
        self.Swing_Lenth = module_info['swing_lenth']
        self.Thign_Lenth = module_info['thign_lenth']
        self.Calf_Lenth = module_info['calf_lenth']
        self.Body_Width = module_info['body_width']
        self.Body_Length = module_info['body_length']
        self.Virtual_Height = module_info['virtual_height']
        self.Body_Height = module_info['body_height']

        #腿部髋关节坐标 分别为 LF RF LB RB 此为常量
        self.leg_loc_virtul = np.array([[self.Body_Length/2 , self.Body_Width/2, self.Virtual_Height],\
                                 [self.Body_Length/2 , -self.Body_Width/2, self.Virtual_Height],\
                                 [-self.Body_Length/2 , self.Body_Width/2, self.Virtual_Height],\
                                 [-self.Body_Length/2 , -self.Body_Width/2, self.Virtual_Height]]) 

        #这里是每个hip的坐标
        self.leg_loc_real = np.array([[self.Body_Length/2 ,self.Body_Width/2,-self.Body_Height],\
                                 [self.Body_Length/2 ,-self.Body_Width/2,-self.Body_Height],\
                                 [-self.Body_Length/2 ,self.Body_Width/2,-self.Body_Height],\
                                 [-self.Body_Length/2 ,-self.Body_Width/2,-self.Body_Height]])

        #对角腿角度
        self.leg_loc_ang = np.arctan2(self.Body_Width, self.Body_Length)

        self.mg_N = 16.44 * 9.81           #此为常量

        self.leg_normal_len = 0.32*np.sqrt(2) - 0.0 #腿一般伸长
        self.body_normal_z = self.leg_normal_len + self.Body_Height #直线一般保持的坐标

        #**************状态记录量**************
        #腿是否归位， 1表示归位
        self.leg_reset = [1,1,1,1]


    def refresh(self):  
        '''
        四足综合刷新函数
        '''

        #腿部参数跟新
        self.Leg_lf.refresh()
        self.Leg_lb.refresh()
        self.Leg_rf.refresh()
        self.Leg_rb.refresh()

        self.touchstate = np.array([self.Leg_lf.touchstate,self.Leg_rf.touchstate,
                                    self.Leg_lb.touchstate,self.Leg_rb.touchstate])
        # print(self.touchstate)


        #姿态角度获取
        imu = self.imu.getRollPitchYaw()  #角度 方向在此矫正
        self.attitude.assig([imu[0],-imu[1],imu[2]]) #ok
        # print(self.attitude)
        #姿态 角速度 和 角加速度 计算
        gyro = self.gyro.getValues()
        self.dattitude.assig( [gyro[0],-gyro[2],gyro[1]] )
        ddattitude = (self.dattitude - self.dattitude_last) / self.real_time
        self.ddattitude.assig(ddattitude)
        self.dattitude_last.assig( [gyro[0],-gyro[2],gyro[1]] )
        # print(self.ddattitude)

        #gps位置获取
        pos = self.gps.getValues() # 方向在此矫正
        self.gps_pos.assig([pos[0],-pos[2],pos[1]])

        #gps速度计算
        speed = (self.gps_pos - self.gps_pos_last)/self.real_time
        self.gps_speed.assig( speed )
        self.gps_pos_last.assig(self.gps_pos.draw())
        

        #矫正后坐标在此处刷新
        self.gps_pos_now.assig(self.gps_pos - self.gps_pos_bias)
        # print(self.gps_pos_now.draw())
        self.leg_pos_now = np.array([robot.Leg_lf.position.draw(),
                                     robot.Leg_rf.position.draw(),
                                     robot.Leg_lb.position.draw(),
                                     robot.Leg_rb.position.draw(),
                                     ])     #初始化 腿部位置误差
        # leg position in CoM frame
        self.leg_pos_nowg = self.leg_pos_now + self.leg_loc_real  

        #定时器
        self.time = self.time + self.real_time

    def leg_diag_ang(self):
        ang_ = self.attitude.roll * np.cos(self.leg_loc_ang)
        return ang_

    def stand_up(self):  #站立及初始化一些参数
        #gps bias
        self.gps_pos_bias.assig([self.gps_pos[0], self.gps_pos[1], self.gps_pos[2] -self.body_normal_z])



robot = Dog()
timestep,times = int(robot.getBasicTimeStep()),0

#力控全局变量 多个函数访问
force = [[0 for j in range(3)] for i in range(4)]


'''
dog_b测试部分代码
'''

#行走类

from dog_func.walk_tri import Walk2
from dog_func.ready import Ready

ready = Ready(robot)
walk2 = Walk2(robot)

from comm.dynamics.wbc_f import * 

while robot.step(timestep) != -1: 
 
    times += 1
    robot.refresh()

    if( ready.prepare() == 0):
        robot.stand_up()
        pass
    else:
        
        
        walk2.main_loop(True)

    #     # print(robot.Leg_lf.force.draw(),' ', robot.Leg_lb.force.draw())
    #     walk.main_loop(True)
    #     # print(robot.gps_pos_now)
    

    pass






