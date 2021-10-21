#   Quadruped robot leg class
#   Created in 2019 10/20
#   Vision = 1.0
#   Author Junwen Cui

from controller import *
from comm.action.motion import *

class Leg():

    def __init__(self, module_name, module_info):

        self.module_info = module_info

        self.speed = Speed()   #当前速度
        self.speed_f = Speed() #期望速度
      
        self.position = Pos()   #当前位置
        self.position_f = Pos() #期望位置

        self.torque_f = Torque() #期望扭矩
        
        self.force = Force()
        self.force_f = Force() #期望足端力

        self.swing_angle = 0
        self.thign_angle = 0
        self.calf_angle = 0

        self.swing_angle_f = 0 #期望电机位置
        self.thign_angle_f = 0
        self.calf_angle_f = 0

        self.touchstate = 0

        self.swing_motor = Motor(module_name['swing_motor_s'])
        self.thign_motor = Motor(module_name['thignleg_motor_s'])
        self.calf_motor = Motor(module_name['calfleg_motor_s'])

        self.swing_motor.enableTorqueFeedback(2)
        self.thign_motor.enableTorqueFeedback(2)
        self.calf_motor.enableTorqueFeedback(2)

        self.swing_positionsensor = PositionSensor(module_name['swing_positionsensor_s'])
        self.thign_positionsensor = PositionSensor(module_name['thign_positionsensor_s'])
        self.calf_positionsensor = PositionSensor(module_name['calf_positionsensor_s'])
        self.swing_positionsensor.enable(2)
        self.thign_positionsensor.enable(2)
        self.calf_positionsensor.enable(2)

        self.touchsensor = TouchSensor(module_name['foot_touchsensor_s'])
        self.touchsensor.enable(2)

    def refresh(self, type = 'all'): #刷新电机位置并解算足端位置，采集足端压力

        if(type ==  'all'):

            self.swing_angle = self.swing_positionsensor.getValue()
            self.thign_angle = self.thign_positionsensor.getValue()
            self.calf_angle = self.calf_positionsensor.getValue()

            DK(self, self.module_info)    #腿部xyz位置刷新    

            #腿部脚步传感器位置刷新
            self.force = self.touchsensor.getValues()
            for i in range(3):
                self.force[i] /= 10

            #腿部综合力判断
            if( np.sqrt(np.square(self.force[0]) + np.square(self.force[1]) + np.square(self.force[2])) >= 3 ):
                self.touchstate = 1
            else:
                self.touchstate = 0

    def __set_torque(self, swing_torque, thign_torque, calf_torque): #设置电机扭矩

        #swing_torque = trans2max(swing_torque, -20, 20)
        #thign_torque = trans2max(thign_torque, -20, 20)
        #calf_torque = trans2max(calf_torque, -20, 20)

        self.swing_motor.setTorque(float(swing_torque))
        self.thign_motor.setTorque(float(thign_torque))
        self.calf_motor.setTorque(float(calf_torque))

    def __set_motorspeed(self, swing_speed, thign_speed, calf_speed): #设置电机速度

        self.swing_motor.setVelocity(swing_speed)
        self.thign_motor.setVelocity(thign_speed)
        self.calf_motor.setVelocity(calf_speed)

    def __set_motorposition(self): #设置电机位置

        self.swing_motor.setPosition(self.swing_angle_f)
        self.thign_motor.setPosition(self.thign_angle_f)
        self.calf_motor.setPosition( self.calf_angle_f)


    def set_force(self, force): #设置足端力矢量
        
        self.force_f = force

        torque2force(self, self.module_info)

        self.__set_torque(self.torque_f.swing, self.torque_f.thign, self.torque_f.calf)

        return 0

    def set_velocity(self, speed): #设置足端速度矢量

        return 0

    def set_position(self, pos): #设置足端位置

        self.position_f = pos
        IK(self, self.module_info)

        self.__set_motorposition()

        return 0