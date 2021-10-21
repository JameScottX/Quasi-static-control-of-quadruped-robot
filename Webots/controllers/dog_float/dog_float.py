#   Quadruped robots main function
#   Created in 2019 10/20
#   Vision = 1.0

import numpy as np
from controller import *
from comm.base.leg import *
from comm.action.motion import *
from comm.action.new_vmc import *
from comm.dynamics.dy_jump import *
from comm.misc.plotf import *
from comm.action.new_vmc import *


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
               'imu': 'imu' ,
               'gps':'gps'
               }


#模型基本参数 单位 m
module_info = {'body_height':0.026,'real_body_height':0.02,'swing_lenth': 0, 'thign_lenth': 0.160, 'calf_lenth': 0.160, 'body_width': 0.130, 'body_length': 0.260}

class Dog(Supervisor):

    def __init__(self):

        super(Dog, self).__init__()

        self.attitude = Attitude()  #机身姿态 roll pitch yaw

        self.imu = InertialUnit(module_name['imu'])
        self.imu.enable(1)

        self.gps = GPS(module_name['gps'])
        self.gps.enable(1)
        
        self.gps_pos_now = Pos()   #最后矫正坐标
        self.gps_pos = Pos()       #此处 gps 为矫正的初始化坐标
        self.gps_pos_bias = [0,0,0]#矫正 gps临时变量

        self.Leg_lf = Leg(module_name['lf'], module_info)
        self.Leg_lb = Leg(module_name['lb'], module_info)
        self.Leg_rf = Leg(module_name['rf'], module_info)
        self.Leg_rb = Leg(module_name['rb'], module_info)

        self.Swing_Lenth = module_info['swing_lenth']
        self.Thign_Lenth = module_info['thign_lenth']
        self.Calf_Lenth = module_info['calf_lenth']
        self.Body_Width = module_info['body_width']
        self.Body_Length = module_info['body_length']
        self.Body_Height = module_info['body_height']
        self.Real_Body_Height = module_info['real_body_height']

        #腿部髋关节坐标 分别为 LF RF LB RB 此为常量
        self.leg_loc = np.array([[-0.13,self.Body_Height,0.065],[-0.13,self.Body_Height,-0.065],[0.13,self.Body_Height,0.065],[0.13,self.Body_Height,-0.065]])
        self.mg_N = 16.44 * 9.81           #此为常量

        self.leg_desire_touch = [0,1,1,0]  #此为 腿部是否着地 记录变量

        #   Dynamic function
        self.dynamics_init()


    #四足综合刷新函数
    def refresh(self):  

        #腿部参数跟新
        self.Leg_lf.refresh()
        self.Leg_lb.refresh()
        self.Leg_rf.refresh()
        self.Leg_rb.refresh()

        self.touchstate = np.array([self.Leg_lf.touchstate,self.Leg_rf.touchstate,self.Leg_lb.touchstate,self.Leg_rb.touchstate])
       
        inertial = self.imu.getRollPitchYaw()  #角度 方向在此矫正
        self.attitude.roll = -inertial[0]    # x轴
        self.attitude.pitch = -inertial[1]   # z轴
        self.attitude.yaw = inertial[2]      # y轴

        pos = self.gps.getValues()        # 方向在此矫正
        self.gps_pos.x = -pos[0]
        self.gps_pos.y = pos[1]
        self.gps_pos.z = -pos[2]

        #矫正后坐标在此处刷新
        self.gps_pos_now.x = self.gps_pos.x  - self.gps_pos_bias[0] 
        self.gps_pos_now.y = self.gps_pos.y  - self.gps_pos_bias[1] 
        self.gps_pos_now.z = self.gps_pos.z  - self.gps_pos_bias[2]

        self.leg_pos_now = np.array([[robot.Leg_lf.position.x, robot.Leg_lf.position.y, robot.Leg_lf.position.z],
                                     [robot.Leg_rf.position.x, robot.Leg_rf.position.y, robot.Leg_rf.position.z],
                                     [robot.Leg_lb.position.x, robot.Leg_lb.position.y, robot.Leg_lb.position.z],
                                     [robot.Leg_rb.position.x, robot.Leg_rb.position.y, robot.Leg_rb.position.z],
                                     ])     #初始化 腿部位置误差
        

    def forceset(self, force,reaction_force):

        forceclass = Force()

        if not (reaction_force == 1 or -1):
            raise ValueError('reaction_force must be 1 or -1')

        if not np.shape(np.array(force)) == (4,3):
            raise ValueError('force shape must be 4x3')


        force_new = []
        for i in range(4):
            m_force = np.mat([[force[i][0]],[force[i][1]],[force[i][2]]])
            m_R = np.mat(R_Matrix(self.attitude.roll,self.attitude.yaw,self.attitude.pitch))  #腿部坐标系需要单独进行转换

            force_new.append(np.squeeze(np.array(np.linalg.inv(m_R) * m_force)))


        force = force_new
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force[0][0], reaction_force * force[0][1], reaction_force * force[0][2]      
        self.Leg_lf.set_force(forceclass)
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force[2][0], reaction_force * force[2][1], reaction_force * force[2][2]
        self.Leg_lb.set_force(forceclass)
        forceclass.x ,forceclass.y,forceclass.z = reaction_force * force[1][0], reaction_force * force[1][1], reaction_force * force[1][2]
        self.Leg_rf.set_force(forceclass)
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force[3][0], reaction_force * force[3][1], reaction_force * force[3][2]
        self.Leg_rb.set_force(forceclass)

    def forcetest(self, f, ang):

        force_f = Force(np.sin(f), f, 0)
        force_b = Force(np.sin(f), f, 0)

        self.Leg_lf.set_force(force_f)
        self.Leg_lb.set_force(force_b)
        self.Leg_rf.set_force(force_f)
        self.Leg_rb.set_force(force_b)

    #运动初始化 函数
    def dynamics_init(self):

        self.body_error_pos_last = [[0 for j in range(3)] for i in range(4)]     #初始化 位置误差
        self.leg_error_pos_last = [[0 for j in range(3)] for i in range(4)]      #初始化 腿部位置误差

        self.one_jump = OneJump()                                                #单一跳跃初始化

    def stand_up(self):  #站立及初始化一些参数

        pos_y  = -0.200 -self.Real_Body_Height
        pos = Pos(0, -0.200 , 0)
        self.Leg_lf.set_position(pos)
        self.Leg_lb.set_position(pos)
        self.Leg_rf.set_position(pos)
        self.Leg_rb.set_position(pos)
        #gps 初始偏置 确定 单位
        self.gps_pos_bias[0], self.gps_pos_bias[1],self.gps_pos_bias[2] = self.gps_pos.x, self.gps_pos.y + pos_y, self.gps_pos.z


    def leg_fly_pos_protect(self,leg,pos):

        if leg.touchstate != 1:

            pass

    def one_jump_func(self):   

        #身体当前位置
        now_state = np.array([0, robot.gps_pos_now.y, 0, robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch])
        #print(np.shape(now_state))
        #节律控制 获取期望位置
        jump_info = self.one_jump.jump(now_state, self.touchstate)
        print(self.touchstate)
        #jump_info = 'end_jump'
        if type(jump_info) != str:
            #身体控制  VMC求取 反作用力
            self.body_error_pos_last,force = force_out(now_state, jump_info, self.body_error_pos_last,robot.leg_loc, 
                                                       self.one_jump.tar_K, self.one_jump.tar_D)
            force = np.clip(force,-self.one_jump.force_limit,self.one_jump.force_limit)
            self.forceset(force,-1)   #此处反作力

            #腿部位置误差 刷新
            self.leg_error_pos_last = [[0 for j in range(3)] for i in range(4)]
            
        else :
            if jump_info !='done':

                #now_pos = np.array([[robot.Leg_lf.position.x, robot.Leg_lf.position.y, robot.Leg_lf.position.z],
                #           [robot.Leg_rf.position.x, robot.Leg_rf.position.y, robot.Leg_rf.position.z],
                #           [robot.Leg_lb.position.x, robot.Leg_lb.position.y, robot.Leg_lb.position.z],
                #           [robot.Leg_rb.position.x, robot.Leg_rb.position.y, robot.Leg_rb.position.z]
                #           ])

                ###腿部位置控制
                #self.leg_error_pos_last, force = foot_force_out(now_pos, self.one_jump.leg_pos_control, self.leg_error_pos_last,
                #                                                  self.one_jump.tar_K, self.one_jump.tar_D) 
                #force = np.clip(force,-self.one_jump.force_limit,self.one_jump.force_limit)
                #self.forceset(np.array(force),1)    #此处不是反作用力

                pos = Pos()
                pos.x,pos.y,pos.z = 0,-0.150,0
                

                self.Leg_lf.set_position(pos)
                self.Leg_lb.set_position(pos)
                self.Leg_rf.set_position(pos)
                self.Leg_rb.set_position(pos)

            elif jump_info == 'done':
                #身体位置误差 刷新
                self.body_error_pos_last = [[0 for j in range(3)] for i in range(4)]

         

robot = Dog()
timestep,times = int(robot.getBasicTimeStep()),0

error_last = [[0 for j in range(3)] for i in range(4)]

exp_state = [0,0.220,0,0,0.,0]

force = [[0 for j in range(3)] for i in range(4)]

#数据记录
dr_xyz= data_record(data_name = ['x','y','z'])




step_ = 0.01
init_ = 0.0
init_2 = 0.0
temp =False
while robot.step(timestep) != -1: 
 
    times += 1
    robot.refresh()

    if(times <  (1000/timestep) * 0.01):
        robot.stand_up()
        pass

    else:

        if times % 2 == 0:
            init_+=step_
            if init_ > np.pi/2 and temp == False:
                temp = True
            if temp:

                init_2-= step_

        z_step = np.sin(init_)*0.06  #0.1
        x_step = np.sin(init_2 )*0.06 

        exp_state = [0,0.220,0,x_step,0.,z_step]
    
        now_state = [robot.gps_pos_now.x, robot.gps_pos_now.y, robot.gps_pos_now.z, robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch]
        #4000  robot.mg_N/robot.touchnum
        error_last,force =  force_out(now_state, exp_state, error_last ,robot.leg_loc ,
                                      robot.mg_N ,robot.touchstate,8000 , 120000)
        robot.forceset(force,-1)
        
        dr_xyz.save_data_times(times*timestep/1000,
                               [robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch])
        if times > (1000/timestep) * 8 and not dr_xyz.is_saved:
            dr_xyz.is_saved = True
            data_to_save('float',np.array(dr_xyz.data_y).T,np.array(dr_xyz.data_x).T,'.txt')
            print('save is ok!')