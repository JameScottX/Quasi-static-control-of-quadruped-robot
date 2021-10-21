#   Cubic spline function file
#   Updated in 2020 1/12
#   Author Junwen Cui / JameScottX
#   Other: use numba to accelarate


import numpy as np
import numba as nb

class TRAJ_C(object):

    def __init__(self):

        self.Q_CX= []
        self.Q_CY= []
        self.Q_CZ= []
        self.__len = 0
        self.__T_piece = 0.1

        pass
    
    def _cubic_core(self, X, t_, t_2 ):
        """[最小cubilc核心函数]
        """        
        if type(X) is not np.ndarray:
            X = np.array(X)
        if not np.shape(X) == (4,):
            raise ValueError('_cubic_core X must be (4,)')
        
        T = np.mat([[np.power(t_, 3),np.power(t_, 2), t_, 1],
                    [3*np.power(t_, 2),2*t_, 1, 0],
                    [np.power(t_2, 3),np.power(t_2, 2), t_2, 1],
                    [3*np.power(t_2, 2),2*t_2, 1, 0]])

        Q = np.linalg.solve(T,X)
        return Q


    def traj_mult_pots_Q(self, Xs,Ys,Zs, T):

        """[产生三次样条的Q序列
            点之间时间等分]
        """        
        self.Q_CX, self.Q_CY, self.Q_CZ = [], [], []
        Xs_, Ys_, Zs_ = np.array(Xs), np.array(Ys), np.array(Zs)
        self.__len = np.shape(Xs_)[0]  #此为两个点 序列
        self.__T_piece = T  / self.__len
   
        for i in range(self.__len):
            self.Q_CX.append( self._cubic_core(Xs_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )
            self.Q_CY.append( self._cubic_core(Ys_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )
            self.Q_CZ.append( self._cubic_core(Zs_[i].T, self.__T_piece*i, self.__T_piece*(i+1)) )

    def traj_tc_f(self, t):
        """[cubic spline 轨迹输出]

        Returns:
            [array]: [x y z]
        """        
        temp_i = 0
        
        t_all =self.__len* self.__T_piece
        if t_all <=t:  #解决时间超出问题
            temp_i = self.__len-1
            std = np.array([np.power(t_all, 3),np.power(t_all, 2), t_all, 1])
        else:
            for i in range(1,self.__len+1):
                if i * self.__T_piece > t :
                    temp_i =i-1
                    break
            std = np.array([np.power(t, 3),np.power(t, 2), t, 1])
        
        t_out = np.array([np.dot(std,self.Q_CX[temp_i]), np.dot(std,self.Q_CY[temp_i]), np.dot(std,self.Q_CZ[temp_i])])
        return t_out
    

    def traj_init_Xs(self, xyz, speed):

        Xs, Ys, Zs = [], [], []
        for i in range(len(xyz)-1):
            Xs.append(np.array([ xyz[i][0], speed[i][0], xyz[i+1][0], speed[i+1][0] ]))
            Ys.append(np.array([ xyz[i][1], speed[i][1], xyz[i+1][1], speed[i+1][1] ]))
            Zs.append(np.array([ xyz[i][2], speed[i][2], xyz[i+1][2], speed[i+1][2] ]))

        return np.array(Xs),  np.array(Ys), np.array(Zs)

# traj_c = TRAJ_C()
# temp = [[1,0,0],[2,0,0],[3,-0,0],[4,-0,0]]
# temp2 = [[0,0,0],[0.5,0,0],[0.5,-0,0],[0.0,-0,0]]
# Xs, Ys, Zs = traj_c.traj_init_Xs(temp, temp2)
# traj_c.traj_mult_pots_Q(Xs, Ys, Zs, 10)

# all_ = []
# for t in range(0,200):

#     all_.append(traj_c.traj_tc_f(t*0.1))

# print(all_)