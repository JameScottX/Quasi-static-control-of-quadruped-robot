#   Stand ready function
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 


import numpy as np
from comm.action.basis import Force,Pos,vallimit
from dog_func.stone import Stone as st
from comm.action.traj_jerk import TRAJ_J

class Ready():

    def __init__(self,robot):

        self._robot = robot
        self.stand_high = robot.leg_normal_len

        self._sequence = np.array([0 for _ in range(10)])
        self.one_taken = [False for _ in range(10)]

        self.pot_n = []

        self.time_count =0

        self.traj_j = TRAJ_J()
        self.timer = 0.

    def prepare(self):

        self.time_count+=1
        self.timer += self._robot.real_time

        if self.time_count <100:#首先让腿初始化到一定角度
            st.angle_set(self._robot, [[0,-0.1,0.2],[0,-0.1,0.2],[0,-0.1,0.2],[0,-0.1,0.2]])
            return self._sequence[0]
        
        if self.one_taken[0] == False:
            self.one_taken[0] = True
            self.temp_target= np.array([-0.0,0,-self.stand_high])
            

            b_xyz = [ self._robot.leg_pos_now[0], np.array([0.0,0,-self.stand_high]) ]
            move_speed_sta_end= [ [0.,0.,0.], [0.,0.,0.] ] 
            move_speed_keep= [0.,0.,0.] 
            b_speed, b_acc = self.traj_j.xyz_speed_default(b_xyz, \
                        move_speed_sta_end, move_speed_keep,1)
            self.traj_j.traj_Xs(b_xyz, b_speed, b_acc, 1)

            self.timer = 0.
        
        self.pot_n = []
        for i in range(4):
            self.pot_n.append(self.traj_j.traj_tj_f(self.timer))

        st.pots_set(self._robot, self.pot_n)
        #条件判断
        dis_pos = np.sum(np.square(self._robot.leg_pos_now[0] - self.temp_target))
        if dis_pos<0.00001:
            self._sequence[0] = 1
            pass
        
        return self._sequence[0]