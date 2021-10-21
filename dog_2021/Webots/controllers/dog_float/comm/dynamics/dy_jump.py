#   Multiple jumpping motion of quadruped robots
#   Created in 2019 10/25
#   Vision = 1.0
#   Author Junwen Cui

import numpy as np

class OneJump():

    def __init__(self):
        
         self.state_recod = 0   # 0 跳跃 1 缓冲

         self.one_jump_ready = np.array([0,0.15,0,0,0,0])  #
         self.end_jump_pos =   np.array([-0.0,0.240,0,0,0,0])  #
         self.__ready = False

         
         self.leg_pos_tor_control = False          #跳跃离地标志
         self.leg_pos_control = np.array([[0,-0.15,0] for _ in range(4)])       #跳跃离地后的位置控制  

         self.tar_K,self.tar_D = 150/0.2 + 200 , 5000    #跳跃的时候 KD 设置
         self.shake_cout = 0
         self.force_limit = 20

         self.desire_speed = 0.0 * 2/1000       #    单位 m/s


    #跳跃流程方法
    def jump(self,now_body_pos,
             foot_onground
             ):

        if self.state_recod  == 0:      #########准备阶段#########
            self.tar_K,self.tar_D =  0 ,-5000    #跳跃的时候 KD 设置
            self.force_limit = 10             #单位 N    限制输出力
            self.desire_speed = 0 * 2/1000

            self.__ready = self.___jump(state = 'ready',
                pos_judge = True,now_pos = now_body_pos, 
                jump_tar=self.one_jump_ready,name = 'ready')

            if self.__ready == False:               
                return self.one_jump_ready
            else :
                self.state_recod +=1
                
                return self.end_jump_pos

        if self.state_recod == 1:       #########起跳阶段#########

            #self.end_jump_pos =   np.array([-0.01,0.250,0,0,0,0])  #
            self.tar_K,self.tar_D = 900 ,-0  #跳跃的时候 KD 设置
            self.force_limit = 200
            self.desire_speed = 0.5 * 2/1000
            self.__ready = self.___jump(now_body_pos,now_pos = now_body_pos, jump_tar=self.end_jump_pos,name = 'end')

            if self.__ready == False:
                return self.end_jump_pos
            else :
                self.state_recod +=1
                return 'end_jump'

        if self.state_recod == 2:       #########收腿阶段#########
            count_ = 0
            self.tar_K,self.tar_D = 500,-5000        #腿部位置 控制 K D
            self.force_limit = 200
            self.desire_speed = 0 * 2/1000
            for i in foot_onground:      
                if i== 0 : count_+=1                        #检测腿离地
            if count_ > 1:
                self.state_recod +=1

                return 'uping'
            else:
                return 'end_jump'

        if self.state_recod ==3:
            count_ = 0
            for i in foot_onground:      
                if i==1 : count_+=1                        #检测腿落地
            if count_ > 1:
                self.state_recod=0
                return 'done'
            else:
                return 'end_jump'
    

    #跳跃位置比较判断
    def ___jump(self,state,threadhold = 0.0,**kwargs):
        speed_judge = False;now_speed = []
        pos_judge =True;now_pos = []
        jump_tar = []
        for name, value in kwargs.items():

            if name == 'speed_judge':speed_judge = value
            elif name == 'pos_judge': pos_judge = value
            elif name == 'now_speed':  now_speed = value
            elif name == 'now_pos' : now_pos = value
            elif name == 'jump_tar' : jump_tar =value

        if not speed_judge and not pos_judge:
            raise ValueError('speed mode and pos mode must be chosen one!')

        is_ok = False 
        if pos_judge:
            if np.shape(jump_tar) != np.shape(now_body_pos):
                raise ValueError('jump_tar and now_body_pos must be the shape!')

            if state == 'ready':
                if now_body_pos[1] < jump_tar[1] +  threadhold:    #在Y轴上比较
                    is_ok = True
            elif state == 'end':
                if now_body_pos[1] > jump_tar[1] -  threadhold:    #在Y轴上比较
                    is_ok = True

        return is_ok














