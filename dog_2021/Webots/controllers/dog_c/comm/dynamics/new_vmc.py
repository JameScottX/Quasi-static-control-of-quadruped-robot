#   Virtual suspension model control method
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: use numba to accelarate

import numpy as np
import numba as nb
from comm.action.basis import R_Matrix


def kd_cal(dis_,speed_des,dis_last,K,D):
    '''
    KD计算函数 使用 numba进行加速
    numba 不支持内部 强制转化为 numpy.array
    输入必须为 np.array
    @nb.njit()   #不使用要快
    '''


    dis_speed = -dis_ + dis_last
    out_ = K * dis_ +  D * (speed_des - dis_speed) 
    return out_

def desire_pos_cal(body_now,body_deire,corner_pos,*args):
    '''
    计算出脚步期望力依据位置误差
    输入参数为   当前身体姿态与目标身体姿态
                髋关节坐标，参数固定
                double/single 是否将位置和姿态分开
    每次计算的是一只腿部位置
    输出的一个array 分别为x y z 方向的矢量
    '''


    if  len(corner_pos)!=3:
        raise ValueError('corner_pos_ have the wrong length!')

    if not (len(body_now) == len(body_deire)):
        raise ValueError('body_now and body_deire must have the same length!')

    if args==None:
        raise ValueError('desire_pos_cal must has args values!')

    corner_pos_m = np.mat([[corner_pos[0]],[corner_pos[1]],[corner_pos[2]]])  #corner_pos 的 xyz坐标

    #desire 部分
    body_derire_xyz = np.mat([[body_deire[0]],[body_deire[1]],[body_deire[2]]])   #身体期望 的 xyz坐标
    R_mat = np.mat(R_Matrix(body_deire[3],body_deire[4],body_deire[5]))    
    Expect_ang = R_mat * corner_pos_m

    #now 部分
    body_now_zyx = np.mat([[body_now[0]],[body_now[1]],[body_now[2]]])   #身体now 的 xyz坐标
    R_mat = np.mat(R_Matrix(body_now[3],body_now[4],body_now[5]))
    Now_ang = R_mat * corner_pos_m

    #print(Now_xyz)
    #print(Expect_xyz)

    dis_xyz = 1.*(body_derire_xyz - body_now_zyx)  #身体期望 的 xyz 矢量
    dis_ang = 1.*(Expect_ang - Now_ang)            #身体期望 的 roll pitch yaw 转化xyz 矢量

    if args[0] == 'double':

        dis_xyz_  = np.squeeze(np.array(dis_xyz))  
        dis_ang_  = np.squeeze(np.array(dis_ang))  
        return dis_xyz_,dis_ang_

    elif args[0] == 'single':

        dis_xyz_ang = dis_xyz + dis_ang
        dis_xyz_ang_  = np.squeeze(np.array(dis_xyz_ang)) 
        return dis_xyz_ang_
    else :
        raise ValueError('desire_pos_cal args must be double or single')

def desire_pos_cal_asse(body_now,body_deire,corner_pos_,*args):
    '''
    计算出脚步期望力位置误差
    输入参数为  当前身体姿态与目标身体姿态  len = 6
               髋关节坐标，参数固定 nX3
                **kwargs包含
                biped,double/single
    输出一个    4X3 array分别代表 LF RF LB RB 的 x y z 的方向量  --默认
                2X3 array分别代表 LF RF 的 x y z 的方向量
                1X3 array分别代表 Leg 的 x y z 的方向量
    '''


    if args==None:
        raise ValueError('desire_pos_cal_asse must has args values!')

    size_leg = args[0]

    if type(corner_pos_) is not np.ndarray:
        corner_pos_ = np.array(corner_pos_)

    if np.shape(corner_pos_)!= (size_leg,3):
        raise ValueError('corner_pos_ have the wrong shape!')

    if args[1] == 'double':
        dis_xyz_asse, dis_ang_asse= [],[]
        for i in range(size_leg):
            corner_pos_one = corner_pos_[i]
            dis_xyz_,dis_ang_ = desire_pos_cal(body_now,body_deire,corner_pos_one,'double')
            dis_xyz_asse.append(dis_xyz_)
            dis_ang_asse.append(dis_ang_)
        return np.array(dis_xyz_asse),np.array(dis_ang_asse)

    elif args[1] == 'single':
        dis_xyz_asse = []
        for i in range(size_leg):
            corner_pos_one = corner_pos_[i]
            dis_xyz_ang = desire_pos_cal(body_now,body_deire,corner_pos_one,'single')
            dis_xyz_asse.append(dis_xyz_ang)
        return np.array(dis_xyz_asse)
    else :
        raise ValueError('desire_pos_cal_asse args must have double or single')
    

def force_out(body_now,body_deire,dis_xyz_asse_last,corner_pos_,mg_N,touchstate,K,D,**kwargs):
    '''
    Single Spring-Damper
    计算出脚步期望力
    输入参数为   当前身体姿态与目标身体姿态   len = 6  坐标依据身体相对世界的坐标系
                上次位置误差，在外部程序中由微分决定           
                n条腿 髋关节坐标   nX3
                K,D 为弹簧系数和阻尼系数,这里腿部通用
                是都需要保持 在 xyz坐标的位置
                身体平均到支撑腿的受力 N 及 脚部接触状态
    输出        该次矢量误差 nX3
                一个 4X3  array分别代表 LF RF LB RB 的 x y z 方向的期望力 --默认
                or 一个 2X3  array分别代表 LF RF 的 x y z 方向的期望力
                or 一个 1X3  array分别代表 Leg的 x y z 方向的期望力
    '''


    leg_num = 4
    speed_desire = 0  #期望速度
    for name, value in kwargs.items():
        if name == 'biped': leg_num = 2
        elif name == 'singled': leg_num = 1
        elif name == 'quadruped': leg_num = 4
        elif name =='speed_desire': speed_desire = value

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    dis_xyz_asse = desire_pos_cal_asse(body_now,body_deire,corner_pos_,leg_num,'single')

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

    #out_ = K * dis_xyz_asse +  D * (speed_desire - (dis_xyz_asse -  np.array(dis_xyz_asse_last))) 
    out_ = kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

    #依据接触给腿添加重力分量
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    #print(t)
    mg_N_ = mg_N / t
    for i in out_:
        i[1] += mg_N_

    return   dis_xyz_asse,out_

def force_out_2(body_now,body_deire,dis_xyz_asse_last,dis_ang_asse_last,corner_pos_,mg_N,touchstate,K,D,**kwargs):
    '''
    Double Spring-Damper
    计算出脚步期望力
    输入参数为  当前身体姿态与目标身体姿态   len = 6  坐标依据身体相对世界的坐标系
            上次位置误差，在外部程序中由微分决定  
            上次的姿态转换后的位置误差         
            n条腿 髋关节坐标   nX3
            K,D 为弹簧系数和阻尼系数,这里腿部通用  len 均为 2
            是都需要保持 在 xyz坐标的位置
            身体平均到支撑腿的受力 N 及 脚部接触状态
    输出        该次矢量误差 nX3, 上次的姿态转换后的位置误差 nX3
            1个 4X3  array分别代表 LF RF LB RB 的 x y z 方向的期望力 --默认
            or 2个 2X3  array分别代表 LF RF 的 x y z 方向的期望力
            or 2个 1X3  array分别代表 Leg的 x y z 方向的期望力
    '''

    leg_num = 4
    speed_desire = [0,0]  #期望速度
    for name, value in kwargs.items():
        if name == 'biped': leg_num = 2
        elif name == 'singled': leg_num = 1
        elif name == 'quadruped': leg_num = 4
        elif name =='speed_desire': speed_desire = value

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    dis_xyz_asse,dis_ang_asse = desire_pos_cal_asse(body_now,body_deire,corner_pos_,leg_num,'double')

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')
    if np.shape(dis_ang_asse)!= np.shape(dis_ang_asse_last):
        raise ValueError('dis_ang_asse and  dis_ang_asse_last must have the same shape!')

    out_ = kd_cal(dis_xyz_asse,speed_desire[0],dis_xyz_asse_last,K[0],D[0])+\
            kd_cal(dis_ang_asse,speed_desire[1],dis_ang_asse_last,K[1],D[1])

    #依据接触给腿添加重力分量
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    mg_N_ = mg_N / t
    for i in out_:
        i[1] += mg_N_

    return   dis_xyz_asse,dis_ang_asse,out_


def force_out_sepa(body_now,body_deire, \
            dis_xyz_asse_last,dis_ang_asse_last, \
            corner_pos_,mg_N,touchstate,K,D,**kwargs):

    leg_num = 4
    speed_desire = [0,0,0]  #期望速度
    for name, value in kwargs.items():
        if name == 'biped': leg_num = 2
        elif name == 'singled': leg_num = 1
        elif name == 'quadruped': leg_num = 4
        elif name =='speed_desire': speed_desire = value

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    dis_xyz_asse,dis_ang_asse = desire_pos_cal_asse(body_now,body_deire,corner_pos_,leg_num,'double')

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')
    if np.shape(dis_ang_asse)!= np.shape(dis_ang_asse_last):
        raise ValueError('dis_ang_asse and  dis_ang_asse_last must have the same shape!')

    K_ = np.array([[K[0][0] ,0,0] for _ in range(leg_num)])
    D_ = np.array([[D[0][0] ,0,0] for _ in range(leg_num)])
    out_xyz_ = kd_cal(dis_xyz_asse,speed_desire[0],dis_xyz_asse_last,K_,D_)
    K_ = np.array([[K[1][0] ,0,0] for _ in range(leg_num)])
    D_ = np.array([[D[1][0] ,0,0] for _ in range(leg_num)])
    out_ang_ = kd_cal(dis_ang_asse,0,dis_ang_asse_last,K_,D_)

    K_ = np.array([[0,K[0][1] ,0] for _ in range(leg_num)])
    D_ = np.array([[0,D[0][1] ,0] for _ in range(leg_num)])
    out_xyz_ += kd_cal(dis_xyz_asse,speed_desire[1],dis_xyz_asse_last,K_,D_)
    K_ = np.array([[0,K[1][1] ,0] for _ in range(leg_num)])
    D_ = np.array([[0,D[1][1],0] for _ in range(leg_num)])
    out_ang_ += kd_cal(dis_ang_asse,0,dis_ang_asse_last,K_,D_)

    K_ = np.array([[0,0,K[0][2] ] for _ in range(leg_num)])
    D_ = np.array([[0,0,D[0][2] ] for _ in range(leg_num)])
    out_xyz_ += kd_cal(dis_xyz_asse,speed_desire[2],dis_xyz_asse_last,K_,D_)
    K_ = np.array([[0,0,K[1][2]] for _ in range(leg_num)])
    D_ = np.array([[0,0,D[1][2]] for _ in range(leg_num)])
    out_ang_ += kd_cal(dis_ang_asse,0,dis_ang_asse_last,K_,D_)


    out_ = out_xyz_ + out_ang_


    #依据接触给腿添加重力分量
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    mg_N_ = mg_N / t
    for i in out_:
        i[1] += mg_N_

    return   dis_xyz_asse,dis_ang_asse,out_

if __name__ == '__main__':

    global c ,e 
    a = [0,0,0,1,0,0]
    d = [0,0,0,0,0,0]
    c = [[0 for j in range(3)] for i in range(2)]
    e = [[0 for j in range(3)] for i in range(2)]
    K_ = [[4000,4000,4000],
                 [4000,4000,4000]]
    D_ = [[40000,40000,40000],
                 [0,0,0]]
    f_= 0
    c,e, f_= force_out_sepa(a,d,c,e, np.array([[0,0.0,0.05],[0,0.0,-0.05]]),0,[1,1],K_,D_,biped = True)
    print(f_)

    a_ = np.array([[1,2,3],[2,2,2],[4,4,4]])
    # b_ = np.array([[1],[3]])
    # K_ = np.array([[a_[0][2] ,0,0] for _ in range(2)])
    # print(K_)
    # print(a_*K_)
    print(a_[:,2])
    print(type(np.eye(3)))





















