#   giat parttern
#   Updated in 2020 8/6
#   Author Junwen Cui / JameScottX
#   Other:  坐标系已经纠正 依据右手坐标系

import numpy as np


def R_Matrix(psi):
    '''
    旋转矩阵计算 
    输入参数为   yaw
    返回         np.mat

    z轴旋转角 psi
    旋转依据机器人坐标系
    '''

    R_ = np.mat([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
    return R_


class TriGait(object):

    def __init__(self, body_normal_z):

        self.gait_dir_xy = 0.0  #步长方向
        self.gait_length_xy = 0.0 #此为步长的一半

        self.gait_tricycle_sequence_default = [2,0,3,1]
        self.gait_g_center_sequence =[]

        self.gait_stand_z = body_normal_z

        self.HIP_Y = 0.13
        
        self.gait_stand_norm = np.array([ [0.26,self.HIP_Y,body_normal_z],\
                                          [0.26,-self.HIP_Y,body_normal_z], \
                                          [-0.26,self.HIP_Y,body_normal_z], \
                                          [-0.26,-self.HIP_Y,body_normal_z] ] )
        self.gait_hip_poss = np.array([ [0.26,self.HIP_Y,-0.0], [0.26,-self.HIP_Y,-0.0], [-0.26,self.HIP_Y,-0.0], [-0.26,-self.HIP_Y,-0.0] ] )
        self.gait_count = 0

        pass
    
    def __g_tar_xy(self):
        """[计算重心目标点xy平面]
        """        
        x = self.gait_length_xy * np.cos(self.gait_dir_xy)
        y = self.gait_length_xy * np.sin(self.gait_dir_xy)
        return x, y

    def __peroid_add(self, now, peroid =4, step = 1):

        temp = 0   #序列的索引
        for i in range(len(self.gait_tricycle_sequence_default)):
            if now == self.gait_tricycle_sequence_default[i]:
                temp = i
                break
        temp = temp+1
        if temp >= peroid:
            temp=0

        return self.gait_tricycle_sequence_default[temp]


    def tricycle_peroid_init(self, poss, body_pos):
        """[静态行走步态初始化周期]
            待修改和完善
        """
                
        g_center_l, g_center_r = np.array([0.,0.,0.]),np.array([0.,0.,0.])  #左右支撑重心计算
        g_center_l +=( np.array(poss[0]) + np.array(poss[1]) + np.array(poss[2]) )
        g_center_r +=( np.array(poss[0]) + np.array(poss[1]) + np.array(poss[3]) )

        g_center_l /=3
        g_center_r /=3
        #为了使 相对于COM frame
        g_center_l -=np.array(body_pos)
        g_center_r -=np.array(body_pos)
        #在 x-y 平面 进行决策
        g_center_l[2] = 0.0
        g_center_r[2] = 0.0

        tempx, tempy= self.__g_tar_xy()

        g_c_l_cost = np.linalg.norm( g_center_l - np.array([tempx, tempy, 0.]), ord=2)
        g_c_r_cost = np.linalg.norm( g_center_r - np.array([tempx, tempy, 0.]), ord=2)

        if g_c_l_cost - g_c_r_cost > 0.02: 
            self.gait_tricycle_sequence_default = [3,1,2,0]
        elif g_c_l_cost - g_c_r_cost < -0.02:
            self.gait_tricycle_sequence_default = [2,0,3,1] 
        else:
            self.gait_tricycle_sequence_default = [2,0,3,1]
        return self.gait_tricycle_sequence_default[0]

    def __tricycle_foot_tar_xy(self):
        # 两侧腿 朝某个角度位移一定步长 
        x, y = self.__g_tar_xy()
        temp1_ = np.array([ x, y, 0.])
        temp2_ = np.array([ x, -y, 0.])
        temp=[]
        temp.append(temp1_)
        temp.append(temp1_)
        temp.append(temp2_)
        temp.append(temp2_)
        return np.array(temp)

    def __g_centerget(self, poss, indexs):
        # 计算重心 
        temp = np.zeros(np.shape(poss[0]),dtype=float)
        len_ = len(indexs)
        for i in indexs:
            temp += poss[i]
        return temp/len_

    def vet_ang(self, v1, v2):
        return np.arccos(np.dot(v1,v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))) 

    def tricycle_foot_locate(self, sequence, index_now, poss_now, body_poss, future_step = 10*10):
        """[计算未来落脚点和支撑三角形重心]
        """

        index_ = index_now   #当前抬腿
        seq_cout = 0
        self.gait_foot_locate = []
        self.gait_foot_index = []
        
        # 支撑腿中心为重心投影点 为重心期望点
        g_cen_sum = np.zeros(3,dtype=np.float64)
        # 当前重心位置 
        self.body_cen_g_loc = [  body_poss   ]

        # 计算重心 全局框架
        if index_ ==2 or index_ ==0:
            g_cen_sum = self.__g_centerget(poss_now, [0,1,3])
        elif index_ ==3 or index_ ==1:
            g_cen_sum = self.__g_centerget(poss_now, [0,1,2])
        # 计算重心位移 
        self.body_cen_g_loc.append( np.hstack( (np.array(g_cen_sum) + np.array([0., 0., self.gait_stand_z]),\
            np.array([0., 0., body_poss[5]])) ))

        body_yaw = body_poss[5]   #初始化导航角
        # R = R_Matrix(body_yaw)  #获得旋转矩阵
        # gait_hip_poss = np.array((R * np.mat(self.gait_hip_poss).T).T)  #标准的 hip 位置
        # poss_now_ = np.array((R * np.mat(poss_now).T).T)  #接触点 位置
        poss_now_ = np.array(poss_now)
        g_cen_temp = np.array(g_cen_sum)#self.__g_centerget(poss_now_, [0,1,2,3])  #重心位置
        
        # 计算未来步态和重心轨迹
        for i in range(future_step):
            if seq_cout >= len(sequence):  #取决于前进序列
                break
            #前进方向 及 步长
            self.gait_dir_xy = sequence[seq_cout][0]
            self.gait_length_xy = sequence[seq_cout][1]

            #计算单侧腿 旋转矩阵
            R_ = R_Matrix( body_yaw )  
            #当前抬腿期望落点 
            foot_new = self.__tricycle_foot_tar_xy()   
            foot_temp = np.array((R_ * np.mat(foot_new[index_]).T).T)
            gait_hip_poss = np.array((R_ * np.mat(self.gait_hip_poss).T).T)
            #全局规划落脚点 包含重心项   hip的位置   期望超前多少角度 
            poss_now_[index_] = g_cen_temp + gait_hip_poss[index_] + foot_temp
            self.gait_foot_locate.append(np.array(poss_now_[index_])) 
            self.gait_foot_index.append(index_)

            #添加重心轨迹
            if index_ == 0 or index_ == 1 :
                seq_cout+=1
                #刷新身体姿态
                body_yaw+=self.gait_dir_xy
                #计算重心 全局框架
                if index_ ==0:
                    g_cen_sum = self.__g_centerget(poss_now_, [0,1,2])
                    # print("l", self.__g_centerget(poss_now_, [0,1,2]))
                else:
                    g_cen_sum = self.__g_centerget(poss_now_, [0,1,3])
                    # print("r", self.__g_centerget(poss_now_, [0,1,3]))

                self.body_cen_g_loc.append( np.hstack( (np.array(g_cen_sum) + np.array([0, 0, self.gait_stand_z]),\
            np.array([0., 0., body_yaw])) ))
                #计算迈腿的虚拟框架
                g_cen_temp = np.array(g_cen_sum) #self.__g_centerget(poss_now_, [0,1,2,3])

            index_ = self.__peroid_add(index_)    #更新下个抬腿

        self.body_cen_g_loc = np.array(self.body_cen_g_loc)
        self.gait_foot_locate = np.array(self.gait_foot_locate)
        self.gait_foot_index = np.array(self.gait_foot_index)
        self.gait_count = 0


if __name__ == '__main__':

    tri_gait = TriGait(0.44)
    sequence = [ [0.1, 0.2] for _ in range(10)]+ [ [-0.1,0.2] for _ in range(10) ]
    poss_now = np.array([[0.36,tri_gait.HIP_Y,0], [0.36,-tri_gait.HIP_Y,0], [-0.16,tri_gait.HIP_Y,0], [-0.16,-tri_gait.HIP_Y,0]] )
    body_poss = np.hstack( (np.array([0.1, 0, tri_gait.gait_stand_z]), np.zeros((3))))

    index_now = tri_gait.tricycle_peroid_init(poss_now, body_poss[:3])

    print(index_now)
    print(body_poss)
    tri_gait.tricycle_foot_locate(sequence, index_now, poss_now,  body_poss)

    # print(tri_gait.gait_foot_locate)
    # print(tri_gait.body_cen_g_loc)

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # 空间三维画图

    data = np.array(tri_gait.gait_foot_locate)
    # print(len(data))
    x = data[:, 0]  
    y = data[:, 1]  
    z = data[:, 2] 

    # 绘制散点图
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z,color='r')
    temp = 10*np.random.rand() + 2
    for i in range(80):

        if i %4 ==0:
            temp = 10*np.random.rand() + 2
        if i %2 ==0:
            x_ = x[i:i+2]
            y_ = y[i:i+2]
            z_ = z[i:i+2]
            ax.plot(x_,y_,z_,alpha=0.5,lw =temp )

    data = np.array(tri_gait.body_cen_g_loc)
    print(data)
    x = data[:, 0]  
    y = data[:, 1]  
    z = data[:, 2] 
    colors = np.random.rand(len(x))
    ax.scatter(x, y, z, c=colors)
    
    ax.set_xlim(-0, 4)
    ax.set_ylim(-1,1)
    ax.set_zlim( -1,1)
    # ax.set_aspect('equal')

    # 添加坐标轴(顺序是Z, Y, X)
    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()







