#   Basic motion methods
#   Updated in 2020 8/6
#   Author Junwen Cui / JameScottX
#   Other:  坐标系已经纠正 依据右手坐标系

import numpy as np

class Pos(object):

    def __init__(self, a = 0, b = 0, c = 0):

        self.x = a
        self.y = b
        self.z = c
    def __del__(self):
        pass
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y) +",z:"+str(self.z) 

    def __add__(self,add):

        x = self.x + add[0]
        y = self.y + add[1]
        z = self.z + add[2]
        return np.array([x,y,z])

    def __sub__(self,sub):
        x = self.x - sub[0]
        y = self.y - sub[1]
        z = self.z - sub[2]
        return np.array([x,y,z])

    def __getitem__(self,index):

        temp= [self.x,self.y,self.z]
        return temp[index]

    def assig(self,data):
        self.x,self.y,self.z = data[0],data[1],data[2]
        pass
    def draw(self):
        return np.array([self.x,self.y,self.z])
    def norm2(self):
        return np.sqrt(np.sum(np.square(self.draw())))    
    

class Angle:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.swing = a
        self.thign = b
        self.calf = c

    def __del__(self):
        pass
    def __str__(self):
        return "s:"+str(self.swing)+",t:"+str(self.thign) +",c:"+str(self.calf) 

    def __add__(self,add):

        swing = self.swing + add[0]
        thign = self.thign + add[1]
        calf = self.calf + add[2]
        return np.array([swing,thign,calf])

    def __sub__(self,sub):
        swing = self.swing - sub[0]
        thign = self.thign - sub[1]
        calf = self.calf - sub[2]
        return np.array([swing,thign,calf])

    def __getitem__(self,index):

        temp= [self.swing,self.thign,self.calf]
        return temp[index]

    def assig(self,data):
        self.swing,self.thign,self.calf = data[0],data[1],data[2]
        pass
    def draw(self):
        return np.array([self.swing,self.thign,self.calf])


# class Speed:

#     def __init__(self, a = 0, b = 0, c = 0):
        
#         self.x = a
#         self.y = b
#         self.z = c

#     def __del__(self):
#         pass
#     def __str__(self):
#         return "x:"+str(self.x)+",y:"+str(self.y) +",z:"+str(self.z)

#     def __add__(self,add):

#         x = self.x + add[0]
#         y = self.y + add[1]
#         z = self.z + add[2]
#         return np.array([x,y,z])

#     def __sub__(self,sub):
#         x = self.x - sub[0]
#         y = self.y - sub[1]
#         z = self.z - sub[2]
#         return np.array([x,y,z])

#     def __getitem__(self,index):

#         temp= [self.x,self.y,self.z]
#         return temp[index]

#     def assig(self,data):
#         self.x,self.y,self.z = data[0],data[1],data[2]
#         pass
#     def draw(self):
#         return np.array([self.x,self.y,self.z])
#     def norm2(self):
#         return np.sqrt(np.sum(np.square(self.draw())))   

class Force:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.x = a
        self.y = b
        self.z = c

    def __del__(self):
        pass
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y) +",z:"+str(self.z)

    def __add__(self,add):

        x = self.x + add[0]
        y = self.y + add[1]
        z = self.z + add[2]
        return np.array([x,y,z])

    def __sub__(self,sub):
        x = self.x - sub[0]
        y = self.y - sub[1]
        z = self.z - sub[2]
        return np.array([x,y,z])

    def __getitem__(self,index):

        temp= [self.x,self.y,self.z]
        return temp[index]

    def assig(self,data):
        self.x,self.y,self.z = data[0],data[1],data[2]
        pass
    def draw(self):
        return np.array([self.x,self.y,self.z])
    def norm2(self):
        return np.sqrt(np.sum(np.square(self.draw())))

class Torque:

    def __init__(self, a = 0, b = 0, c = 0):
        
        self.swing = a
        self.thign = b
        self.calf = c

    def __del__(self):
        pass
    def __str__(self):
        return "s:"+str(self.swing)+",t:"+str(self.thign) +",c:"+str(self.calf)

    def __add__(self,add):

        swing = self.swing + add[0]
        thign = self.thign + add[1]
        calf = self.calf + add[2]
        return np.array([swing,thign,calf])

    def __sub__(self,sub):
        swing = self.swing - sub[0]
        thign = self.thign - sub[1]
        calf = self.calf - sub[2]
        return np.array([swing,thign,calf])

    def __getitem__(self,index):

        temp= [self.swing,self.thign,self.calf]
        return temp[index]

    def assig(self,data):
        self.swing,self.thign,self.calf = data[0],data[1],data[2]
        pass
    def draw(self):
        return np.array([self.swing,self.thign,self.calf])

class Attitude:

    def __init__(self, a = 0, b = 0, c = 0):

        self.roll = a
        self.pitch = b
        self.yaw = c
    def __del__(self):
        pass
    def __str__(self):
        return "r:"+str(self.roll)+",p:"+str(self.pitch) +",y:"+str(self.yaw)

    def __add__(self,add):

        roll = self.roll + add[0]
        pitch = self.pitch + add[1]
        yaw = self.yaw + add[2]
        return np.array([roll,pitch,yaw])

    def __sub__(self,sub):
        roll = self.roll - sub[0]
        pitch = self.pitch - sub[1]
        yaw = self.yaw - sub[2]
        return np.array([roll,pitch,yaw])

    def __getitem__(self,index):

        temp= [self.roll,self.pitch,self.yaw]
        return temp[index]
    def assig(self,data):
        self.roll,self.pitch,self.yaw = data[0],data[1],data[2]
        pass
    def draw(self):
        return np.array([self.roll,self.pitch,self.yaw])



# @nb.njit()    #加速两倍
def R_Matrix(phi,theta,psi):
    '''
    旋转矩阵计算 使用 numba进行加速
    输入参数为   roll pitch yaw
    返回         数组 由于np.mat不被支持  

    x轴旋转角 phi
    y轴旋转角 theta
    z轴旋转角 psi
    '''
    R_ = np.array([[                              np.cos(psi)*np.cos(theta),                             -np.cos(theta)*np.sin(psi),          -np.sin(theta)],
    [ np.cos(phi)*np.sin(psi) - np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(theta)*np.sin(phi)],
    [ np.sin(phi)*np.sin(psi) + np.cos(phi)*np.cos(psi)*np.sin(theta), np.cos(psi)*np.sin(phi) - np.cos(phi)*np.sin(psi)*np.sin(theta),  np.cos(phi)*np.cos(theta)]])

    return R_

# @nb.njit()    #加速两倍
def R_Matrix_inv(phi,theta,psi):
    '''
    旋转逆矩阵计算 使用 numba进行加速
    输入参数为   roll pitch yaw
    返回         数组 由于np.mat不被支持

    x轴旋转角 phi
    y轴旋转角 theta
    z轴旋转角 psi
    '''

    R_inv = np.array([[  np.cos(psi)*np.cos(theta), np.cos(phi)*np.sin(psi) - np.cos(psi)*np.sin(phi)*np.sin(theta), np.sin(phi)*np.sin(psi) + np.cos(phi)*np.cos(psi)*np.sin(theta)],
    [ -np.cos(theta)*np.sin(psi), np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(psi)*np.sin(theta), np.cos(psi)*np.sin(phi) - np.cos(phi)*np.sin(psi)*np.sin(theta)],
    [          -np.sin(theta),                             -np.cos(theta)*np.sin(phi),                              np.cos(phi)*np.cos(theta)]])
 
    return R_inv


def R_Matrix_yaw(psi):

    R_ = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
    return R_


def vallimit(value, min, max):  
    '''
    限幅函数
    '''
    if value > max : 
        value = max 
    elif  value < min : value = min
    return value


def PD_ang(leg, pos_n, pos_d, K, D):
    '''关节角度PD控制器'''
    dis_ = np.array(pos_d) - np.array(pos_n)
    dis_asse_last =leg.err_stc.draw()
    f_stc = K * dis_ + D * (dis_-dis_asse_last)
    leg.err_stc.assig(dis_)
    return f_stc


def DK(leg, module_info):

    L1 = module_info['swing_lenth']
    L2 = module_info['thign_lenth']
    L3 = module_info['calf_lenth']

    theta1 = leg.angle.swing
    theta2 = leg.angle.thign
    theta3 = leg.angle.calf

    x = L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)
    y = L2*np.cos(theta2)*np.sin(theta1) - L1*np.cos(theta1) + L3*np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - L3*np.sin(theta1)*np.sin(theta2)*np.sin(theta3)
    z = L3*np.cos(theta1)*np.sin(theta2)*np.sin(theta3) - L2*np.cos(theta1)*np.cos(theta2) - L3*np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - L1*np.sin(theta1)

    leg.position.assig([x,y,z])



def DK2(leg, theats):

    L1 = leg.module_info['swing_lenth']
    L2 = leg.module_info['thign_lenth']
    L3 = leg.module_info['calf_lenth']

    theta1 = theats[0]
    theta2 = theats[1]
    theta3 = theats[2]

    x = L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)
    y = L2*np.cos(theta2)*np.sin(theta1) - L1*np.cos(theta1) + L3*np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - L3*np.sin(theta1)*np.sin(theta2)*np.sin(theta3)
    z = L3*np.cos(theta1)*np.sin(theta2)*np.sin(theta3) - L2*np.cos(theta1)*np.cos(theta2) - L3*np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - L1*np.sin(theta1)

    return np.array([x,y,z])

def J_get(leg, theats):

    L1 = leg.module_info['swing_lenth']
    L2 = leg.module_info['thign_lenth']
    L3 = leg.module_info['calf_lenth']

    theta1 = theats[0]
    theta2 = theats[1]
    theta3 = theats[2]


    J = np.mat([[                                                                                                                             0,                L3*np.cos(theta2 + theta3) + L2*np.cos(theta2),              L3*np.cos(theta2 + theta3)],
    [ L1*np.sin(theta1) + L2*np.cos(theta1)*np.cos(theta2) + L3*np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - L3*np.cos(theta1)*np.sin(theta2)*np.sin(theta3), -np.sin(theta1)*(L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)), -L3*np.sin(theta2 + theta3)*np.sin(theta1)],
    [ L2*np.cos(theta2)*np.sin(theta1) - L1*np.cos(theta1) + L3*np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - L3*np.sin(theta1)*np.sin(theta2)*np.sin(theta3),  np.cos(theta1)*(L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)),  L3*np.sin(theta2 + theta3)*np.cos(theta1)]])
 
    return J


def IK(leg,module_info):

    L1 = module_info['swing_lenth']
    L2 = module_info['thign_lenth']
    L3 = module_info['calf_lenth']

    tar = leg.position_f.draw()
    thetas = leg.angle.draw()
    xyz = DK2(leg, thetas)
    J = J_get(leg, thetas)
    J__ = np.linalg.pinv(J)
    exyz = np.mat(tar - xyz).T
    thetas_ = J__ * exyz
    thetas += np.squeeze(np.array(thetas_).T)

    for i in range(15):
                    
        temp = np.squeeze(thetas.T)
        xyz = DK2(leg, temp)

        if np.linalg.norm(tar - xyz) < 1e-4:
            break

        J = J_get(leg , thetas)
        J__ = np.linalg.pinv(J)
        exyz = np.mat(tar - xyz).T
        thetas_ = J__* exyz
        thetas += np.squeeze(np.array(thetas_).T)
    
    leg.angle_f.assig(np.squeeze(thetas.T))


def force2torque(leg, module_info):

    force = np.mat(leg.force_f.draw()).T
    thetas = leg.angle.draw()
    J_T = J_get(leg,thetas).T
    torque = J_T * force
    leg.torque_f.assig(np.squeeze(np.array(torque).T))


if __name__ == '__main__':

    L1 = 0
    L2 = 0.32
    L3 = 0.32
    theta1 = 0
    theta2 = -1.5708/2
    theta3 = 1.5708

    J = np.mat([[                                                                                                                             0,                L3*np.cos(theta2 + theta3) + L2*np.cos(theta2),              L3*np.cos(theta2 + theta3)],
        [ L1*np.sin(theta1) + L2*np.cos(theta1)*np.cos(theta2) + L3*np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - L3*np.cos(theta1)*np.sin(theta2)*np.sin(theta3), -np.sin(theta1)*(L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)), -L3*np.sin(theta2 + theta3)*np.sin(theta1)],
        [ L2*np.cos(theta2)*np.sin(theta1) - L1*np.cos(theta1) + L3*np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - L3*np.sin(theta1)*np.sin(theta2)*np.sin(theta3),  np.cos(theta1)*(L3*np.sin(theta2 + theta3) + L2*np.sin(theta2)),  L3*np.sin(theta2 + theta3)*np.cos(theta1)]])

    temp  = J.T * np.mat([0,0,100]).T
    print(temp)




