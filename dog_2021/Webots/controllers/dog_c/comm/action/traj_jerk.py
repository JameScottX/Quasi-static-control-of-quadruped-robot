#   Mini Jerk function file
#   Updated in 2020 1/12
#   Author Junwen Cui / JameScottX
#   Other: use numba to accelarate

import numpy as np
import numba as nb

class TRAJ_J(object):

    def __init__(self):

        self.Q_JX= []
        self.Q_JY= []
        self.Q_JZ= []
        self.Q_JS= []

        self.__len = 0
        self.__T_piece = 0.1
        self.T = 0


        self.__len_s = 0
        self.__T_piece_s =  0.1
        self.T_s = 0

        pass


    def _jerk_core(self, X, t_, t_2):
        '''最小jerk核心函数'''
        if type(X) is not np.ndarray:
            X = np.array(X)
        if not np.shape(X) == (6,):
            raise ValueError('jerk_func X must be (6,)')
        
        T = np.mat([[np.power(t_, 5), np.power(t_, 4),np.power(t_, 3),np.power(t_, 2), t_, 1],
            [5*np.power(t_, 4), 4*np.power(t_, 3),3*np.power(t_, 2),2*t_, 1, 0],
            [20*np.power(t_, 3), 12*np.power(t_, 2),6*t_, 2, 0, 0],
            [np.power(t_2, 5), np.power(t_2, 4),np.power(t_2, 3),np.power(t_2, 2), t_2, 1],
            [5*np.power(t_2, 4), 4*np.power(t_2, 3),3*np.power(t_2, 2),2*t_2, 1, 0],
            [20*np.power(t_2, 3), 12*np.power(t_2, 2),6*t_2, 2, 0, 0]])

        Q = np.linalg.solve(T,X)
        return Q

    def traj_mult_pots_Q(self, Xs,Ys,Zs, T):

        """[产生五次样条的Q序列
            点之间时间等分]
        """        
        self.Q_JX, self.Q_JY, self.Q_JZ = [], [], []
        Xs_, Ys_, Zs_ = np.array(Xs), np.array(Ys), np.array(Zs)
        self.__len = np.shape(Xs_)[0]  #此为两个点 序列
        self.__T_piece = T  / (self.__len)
        self.T = T #记录总的时间

        for i in range(self.__len):
            self.Q_JX.append( self._jerk_core(Xs_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )
            self.Q_JY.append( self._jerk_core(Ys_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )
            self.Q_JZ.append( self._jerk_core(Zs_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )


    def traj_tj_f(self, t):
        """[jerk 轨迹输出]

        Returns:
            [array]: [x y z]
        """        
        temp_i = 0
        
        t_all = (self.__len) * self.__T_piece
        if t_all <=t:  #解决时间超出问题
            temp_i = self.__len-1
            std= np.array([np.power(t_all, 5), np.power(t_all, 4),np.power(t_all, 3),np.power(t_all, 2), t_all, 1])
        else:
            for i in range(1,self.__len+1):
                if i * self.__T_piece > t :
                    temp_i =i-1
                    break
            std= np.array([np.power(t, 5), np.power(t, 4),np.power(t, 3),np.power(t, 2), t, 1])
        
        t_out = np.array([np.dot(std,self.Q_JX[temp_i]), np.dot(std,self.Q_JY[temp_i]), np.dot(std,self.Q_JZ[temp_i])])
        return t_out


    def traj_init_Xs(self, xyz, speed, acc):

        Xs, Ys, Zs = [], [], []
        for i in range(len(xyz)-1):
            Xs.append(np.array([ xyz[i][0], speed[i][0], acc[i][0], xyz[i+1][0], speed[i+1][0], acc[i+1][0] ]))
            Ys.append(np.array([ xyz[i][1], speed[i][1], acc[i][1], xyz[i+1][1], speed[i+1][1], acc[i+1][1] ]))
            Zs.append(np.array([ xyz[i][2], speed[i][2], acc[i][2], xyz[i+1][2], speed[i+1][2], acc[i+1][2] ]))

        return np.array(Xs),  np.array(Ys), np.array(Zs)


    def traj_Xs(self, xyz, speed, acc, T):
        #总的路径规划函数
        Xs, Ys, Zs = self.traj_init_Xs(xyz, speed, acc)
        self.traj_mult_pots_Q(Xs, Ys, Zs,T)


    
    def xyz_speed_default(self, xyz, speed_sta_end, speed_keep,  T):

        xyz = np.array(xyz)
        n0  =  np.shape(xyz)[0]

        speed = []
        speed.append(speed_sta_end[0])

        for _ in range( 1, n0-1 ):
            speed.append(speed_keep)

        speed.append(speed_sta_end[1])

        acc= [ [0.,0.,0.] for _ in range(n0) ]

        return np.array(speed), np.array(acc)


    def traj_s_Q(self, s, T):

        self.Q_JS =[]

        self.__len_s = np.shape(s)[0]  #此为两个点 序列
        self.__T_piece_s = T  / (self.__len_s )
   
        for i in range(self.__len_s):
            self.Q_JS.append( self._jerk_core(s[i].T, self.__T_piece_s*i, self.__T_piece_s*(i+1)) )
    
    def traj_init_s(self, s, ds, dds):

        S = []
        for i in range(len(s)-1):
            S.append(np.array([ s[i], ds[i], dds[i], s[i+1], ds[i+1], dds[i+1] ]))

        return S

    def traj_s(self, s, ds, dds, T):
        #总的路径规划函数
        S = self.traj_init_s(s, ds, dds)
        self.traj_s_Q(S,T)

    def traj_ts_f(self, t):
        """[jerk 轨迹输出]
        """        
        temp_i = 0
        
        t_all = (self.__len_s) * self.__T_piece_s
        if t_all <=t:  #解决时间超出问题
            temp_i = self.__len_s-1
            std= np.array([np.power(t_all, 5), np.power(t_all, 4),np.power(t_all, 3),np.power(t_all, 2), t_all, 1])
        else:
            for i in range(1,self.__len_s+1):
                if i * self.__T_piece_s > t :
                    temp_i =i-1
                    break
            std= np.array([np.power(t, 5), np.power(t, 4),np.power(t, 3),np.power(t, 2), t, 1])
        
        t_out = np.dot(std,self.Q_JS[temp_i])
        return t_out

    def s_speed_default(self, s, speed_sta_end, speed_keep):
        #设置 s变化的 速度和加速度 
        s = np.array(s)
        n0  =  np.shape(s)[0]
        speed = []
        speed.append(speed_sta_end[0])
        for _ in range( 1, n0-1 ):
            speed.append(speed_keep)
        speed.append(speed_sta_end[1])
        acc= [ 0. for _ in range(n0) ]

        return np.array(speed), np.array(acc)

    



if __name__ == '__main__':

    traj_j = TRAJ_J()
    xyz = np.array([ [-1,0,0], [0,0,1] , [1,0,0], [1,0,-1] ] )
    speed = np.array([ [0,0,0],[4,0,0],[0,0,0],[0,0,0]  ])
    acc = np.zeros((4,3), dtype=float)
    traj_j.traj_Xs(xyz,speed,acc,1)


    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    #定义图像和三维格式坐标轴
    fig=plt.figure()
    ax1 = Axes3D(fig)
    #坐标在此矫正
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    data = []
    for i in range(50):
        t = i/50
        data_ = traj_j.traj_tj_f(t)

        data.append(data_)
    data = np.array(data).T

    ax1.scatter3D(data[0],data[1],data[2], cmap='Blues')  #绘制散点图

    range = 3
    ax1.set_xlim(-range,range)
    ax1.set_ylim(-range,range)
    ax1.set_zlim(-range,range)
    # ax1.plot3D(data_[2],data_[0],data_[1],'gray')    #绘制空间曲线
    # 
    # 
    plt.show()



    # temp = [[-1,0,0],[2,0,0],[3,-0,0],[4,-0,0]]
    # temp2 = [[0,0,0],[0,0,0],[0,-0,0],[0.0,-0,0]]
    # temp3 = [[0,0,0],[0.0,0,0],[0.0,-0,0],[0.0,-0,0]]

    # Xs, Ys, Zs = traj_j.traj_init_Xs(temp, temp2,temp3)
    # traj_j.traj_mult_pots_Q(Xs, Ys, Zs, 10)

    # all_ = []
    # for t in range(0,200):

    #     all_.append(traj_j.traj_tj_f(t*0.1))

    # print(all_)




