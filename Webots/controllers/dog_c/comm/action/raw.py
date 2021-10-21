#   Ordinary position or force control method
#   Updated in 2020 1/14
#   Author Junwen Cui / JameScottX
#   Other: use numba to accelarate

import numpy as np
import numba as nb
# from comm.action.basis import R_Matrix, Force, Torque, Pos


class Raw():

    @staticmethod
    def kd_cal(dis_,speed_des,dis_last,K,D):
        '''
        KD计算函数 使用 numba进行加速
        numba 不支持内部 强制转化为 numpy.array
        输入必须为 np.array
        @nb.njit()   #不使用要快
        '''


        dis_speed = - dis_ +  dis_last  #这里的间隔时间 将整合到  D  中  比如 D = D / 0.002
        out_ = K * dis_ +  D * (speed_des - dis_speed) 
        return out_

    @classmethod
    def foot_force_support(cls,pos_now,pos_desire,dis_xyz_asse_last,mg_N,touchstate,K,D,**kwargs):
        '''
        单腿位置支撑弹簧虚拟函数
        输入参数为   当前腿部当前位置和期望位置  len=3  坐标依据腿部单独的坐标系
                    上次位置误差，在外部程序中由微分决定
                    机器人质量和脚部接触状态
                    K,D 为弹簧系数和阻尼系数,这里腿部通用         
                    脚步期望力   len=3 
        '''


        speed_desire = 0  #期望速度
        for name, value in kwargs.items():
            if name == 'speed_desire': speed_desire = value

        if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
            raise ValueError('pos_desire and  pos_now must have the same shape!')

        dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

        if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
            raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

        if type(dis_xyz_asse_last) is not np.ndarray:
            dis_xyz_asse_last = np.array(dis_xyz_asse_last)
        out_ = cls.kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

        #依据接触给腿添加重力分量
        t = 0
        for i in touchstate: 
            t +=1 if i==1 else 0
        if t ==0: t=1
        #print(t)
        mg_N_ = mg_N / t
        
        out_[1] -= mg_N_

        return dis_xyz_asse, out_


    @classmethod
    def foot_force_out(cls,pos_now,pos_desire,dis_xyz_asse_last, K,D,**kwargs):
        '''
        单腿位置弹簧虚拟函数
        输入参数为   当前腿部当前位置和期望位置  len=3  坐标依据腿部单独的坐标系
                    上次位置误差，在外部程序中由微分决定
                    K,D 为弹簧系数和阻尼系数,这里腿部通用
                    脚步期望力   len=3 
        '''


        speed_desire = 0  #期望速度
        for name, value in kwargs.items():
            if name == 'speed_desire': speed_desire = value

        if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
            raise ValueError('pos_desire and  pos_now must have the same shape!')

        dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

        if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
            raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

        if type(dis_xyz_asse_last) is not np.ndarray:
            dis_xyz_asse_last = np.array(dis_xyz_asse_last)
        out_ = cls.kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

        return dis_xyz_asse, out_

    @classmethod
    def foot_force_out_all(cls, pos_now,pos_desire,dis_xyz_asse_last, K,D,**kwargs):

        '''
        全腿位置弹簧虚拟函数
        输入参数为   当前腿部当前位置和期望位置  4x3 --默认 or 2X3   坐标依据腿部单独的坐标系
                    上次位置误差，在外部程序中由微分决定
                    K,D 为弹簧系数和阻尼系数,这里腿部通用
                    表示计算的哪几条腿
                    脚步期望力   nX3
        '''


        speed_desire = 0  #期望速度
        leg_use = [1,1,1,1] # --默认

        for name, value in kwargs.items():
            if name == 'speed_desire': speed_desire = value
            elif name == 'leg_use': leg_use = value

        if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
            raise ValueError('pos_desire and  pos_now must have the same shape!')

        dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

        if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
            raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

        leg_use_ = []
        for i in leg_use:
            leg_use_.append([i,i,i])

        if type(dis_xyz_asse_last) is not np.ndarray:
            dis_xyz_asse_last = np.array(dis_xyz_asse_last)

        kd_out = cls.kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)
        out_ = np.array(leg_use_) * kd_out 

        return dis_xyz_asse, out_   #nX3




#**********************************************************************************************************       
#********************************************以下为三足支撑步态基本功能************************************
#**********************************************************************************************************
    @classmethod
    def support_cog(cls,sup_pos):
        '''获得支撑三角形的中心'''
        sup_pos_ = np.array(sup_pos)
        shape_ = np.shape(sup_pos_)[0]
        center =  np.sum(sup_pos_, axis=0)/shape_
        return center


    @classmethod
    def cog_in_sup(cls, sup_pos, center_):
        '''面积法 判断重心是否在支撑三角形内'''

        sup_pos_ = [np.array(sup_pos[i])[:-1] for i in range(3)]

        ori = sup_pos_[0]
        ori_a = sup_pos_[1] - ori
        ori_b = sup_pos_[2] - ori

        #求整个三角形面积
        area_all = np.abs(np.cross(ori_a,ori_b))/2
        # print(area_all)
        
        c_list_ = []
        for i in range(3):
            c_list_.append(sup_pos_[i] - center_[:-1])

        #分散求三角形面积
        area__=0.
        for i in range(3):
            temp = i+1
            if temp >2:
                temp = 0
            area__ += np.abs(np.cross(c_list_[i],c_list_[temp]))/2

        # print(area__)

        if np.abs(area_all - area__) < 0.001:
            return True
        else :
            return False
            



if __name__ == '__main__':
    raw = Raw()
    supp = np.array( [[0,0,0],[0,1,0],[3,1,0]])
    a = raw.com_in_sup(supp, [2,1,1])
    print(a)







