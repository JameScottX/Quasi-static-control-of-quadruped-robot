#   Dog robot leg class
#   Update in 2020 03/23
#   Author Junwen Cui / JameScottX
#   Other: 

import numpy as np
from controller import *
from comm.action.basis import Force,Pos,Angle, R_Matrix_inv,\
    Torque,DK,IK,vallimit,force2torque,PD_ang


class Leg():

    def __init__(self, robot,  module_name, module_info, name):

        self.module_info = module_info
        self.freq = robot.freq
        self.real_time = robot.real_time
        self.name = name

        # self.speed = Speed()   #当前速度
        # self.speed_f = Speed() #期望速度
      
        self.position = Pos()   #当前位置
        self.position_f = Pos() #期望位置

        self.torque = Torque()
        self.torque_f = Torque() #期望扭矩
        
        self.force = Force()
        self.force_f = Force() #期望足端力
        self.touchstate = 0

        self.angle = Angle()   #当前关节角度
        self.angle_f = Angle() #期望关节角度
        self.angle_last = Angle() #上一次关节角度

        self.dangle = Angle() #角速度

        self.swing_motor = Motor(module_name['swing_motor_s'])
        self.thign_motor = Motor(module_name['thignleg_motor_s'])
        self.calf_motor = Motor(module_name['calfleg_motor_s'])

        self.swing_motor.setControlPID(20,0,2)
        self.thign_motor.setControlPID(20,0,2)
        self.calf_motor.setControlPID(20,0,2)

        self.swing_motor.enableForceFeedback(self.freq)
        self.thign_motor.enableForceFeedback(self.freq)
        self.calf_motor.enableForceFeedback(self.freq)

        self.swing_positionsensor = PositionSensor(module_name['swing_positionsensor_s'])
        self.thign_positionsensor = PositionSensor(module_name['thign_positionsensor_s'])
        self.calf_positionsensor = PositionSensor(module_name['calf_positionsensor_s'])

        self.swing_positionsensor.enable(self.freq)
        self.thign_positionsensor.enable(self.freq)
        self.calf_positionsensor.enable(self.freq)

        self.touchsensor = TouchSensor(module_name['foot_touchsensor_s'])  #注意传感器姿态与世界坐标系一致
        self.touchsensor.enable(self.freq)

        self.err_stc = Angle()


    def refresh(self, type = 'all',touch_forcey = 4): 
        '''刷新电机位置并解算足端位置，采集足端压力'''
        if(type ==  'all'):

            self.angle.swing = self.swing_positionsensor.getValue()
            self.angle.thign = self.thign_positionsensor.getValue()
            self.angle.calf = self.calf_positionsensor.getValue()
            
            dangle_ =self.angle - self.angle_last
            self.dangle.assig(dangle_ / self.real_time) 

            self.angle_last.assig(self.angle.draw())

            DK(self, self.module_info)    #腿部xyz位置刷新    
            # print(self.angle.calf)
            # #腿部脚步传感器位置刷新
            
            touch_ = np.array(self.touchsensor.getValues())/10
            #方向在此矫正
            self.force.assig(touch_)
            #腿部综合力判断
            if self.force.y > touch_forcey:
                self.touchstate = 1
            else:
                self.touchstate = 0



    def __set_torque(self, torque): 
        '''设置电机扭矩'''
        swing_torque = vallimit(torque[0], -49, 49)
        thign_torque = vallimit(torque[1], -49, 49)
        calf_torque = vallimit(torque[2], -49, 49)
        
        self.swing_motor.setTorque(float(swing_torque))
        self.thign_motor.setTorque(float(thign_torque))
        self.calf_motor.setTorque(float(calf_torque))

    def __set_motorspeed(self,speed): 
        '''设置电机速度'''
        self.swing_motor.setVelocity(speed[0])
        self.thign_motor.setVelocity(speed[1])
        self.calf_motor.setVelocity(speed[2])

    def __set_motorposition(self):
        '''设置电机位置'''

        self.swing_motor.setPosition(self.angle_f.swing)
        self.thign_motor.setPosition(self.angle_f.thign)
        self.calf_motor.setPosition( self.angle_f.calf)

    def __set_motorposition_pd(self):
        '''PD控制器设置电机位置'''
        # print(self.err_stc) [self.angle_f[0], self.angle_f[1] ,self.angle_f[2]]
        f_stc = PD_ang(self, self.angle.draw(), 
                    self.angle_f.draw(), 200,2/self.real_time)    
        self.torque_f.assig(f_stc)

    def set_force(self, force): 
        '''设置足端力矢量'''
        self.force_f = force
        force2torque(self, self.module_info)
        self.__set_torque(self.torque_f)
        return 0


    def set_position_pd(self,pos,f_add = Force(),s_ratio =[1,0]):
        '''力位混空函数'''
        self.position_f = pos
        IK(self, self.module_info)
        # print(self.angle_f)
        # print(self.err_stc)
        self.__set_motorposition_pd()
        t_f = self.torque_f.draw()
        self.force_f = f_add
        force2torque(self, self.module_info)
        t_add = self.torque_f.draw()
        t__ = s_ratio[0]*t_f + s_ratio[1] * t_add 

        self.torque_f.assig(t__)
        self.__set_torque(self.torque_f)

    def set_position(self, pos): 
        '''设置足端位置'''
        self.position_f = pos
        IK(self, self.module_info)
        self.__set_motorposition()

        return 0

    def set_angle(self,angle):
        '''直接设置电机角度'''
        self.angle_f = angle
        self.__set_motorposition()

    def set_torque(self,torque):
        '''设置关节扭矩'''
        self.__set_torque(torque)
        return 0