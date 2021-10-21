#   QP solver file
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 

import numpy as np


def constr_output_set(min_,max_, output_dim):
    '''一般的QP问题输出限制
    [-I,I].T  * u <= [-u_min, u_max]
    '''
    temp = np.identity(output_dim) 
    M = np.vstack((-temp,temp)) 
    gamma_ = np.vstack((-min_* np.ones((output_dim,1)) , max_* np.ones((output_dim,1))))
    return M, gamma_


def Kuhn_Tucker_condition(E,F,M,gamma,x,lambda_):
    '''Kuhn_Tucker 条件判断'''
    E_,F_,M_,gamma_ = np.mat(E),np.mat(F),np.mat(M),np.mat(gamma)
    lambda_ = np.mat(lambda_)
   
    cond1 = E_*x +F_ + M_.T*lambda_  #=0
    cond2 = M_*x- gamma_        # <= 0
    cond3 = lambda_.T*(cond2)   #=0
    cond4 = lambda_             #>=0
    return [cond1,cond2,cond3,cond4]



def HildrethQPP(E,F,M,gamma):
    '''
    Use Hildreth quadratic programming to solve primal-dual method
    J = 1/2 X'EX + X'F + Lambda '(MX-gamma)
    Lambda is the lagrange multipliers
    '''

    E_,F_,M_,gamma_ = np.mat(E),np.mat(F),np.mat(M),np.mat(gamma)
    H = M_* E_.I * M_.T
    K = gamma_ + M_* E_.I*F_
    # print(H,K)
    n1, m1 = np.shape(M_)
    eta  = - E_.I * F_
   
    kk =0
    for i in range(n1):
        if M_[i] * eta > gamma_[i] :
            kk +=1
    
    n,m =np.shape(K)
    x_init = np.zeros((n,m))
    lambda_ = np.mat(x_init, dtype=np.float64)

    if kk ==0:   
        return eta,lambda_
        
    al = 10.
    for km in range(20):
        lambda_P =  lambda_.copy()
        for i in range(n): 
            w = H[i]*lambda_ - H[i,i]* lambda_[i]
            
            w+=K[i]
            la = - w / (H[i,i] + 10e-6)  #防止除0

            if la <0:
                lambda_[i] = 0
            else :
                lambda_[i] = la

        temp =np.sum(np.abs(lambda_ -lambda_P))
    
        if temp < 0.00001:
            # print(lambda_)
            break
        eta = - E_.I * F_ - E_.I * M_.T * lambda_

    return eta,lambda_


'''
************************************************************************************************
以下为测试部分
对vmc模型，求解qp问题，分配到每条接触腿
************************************************************************************************
'''
'''
from wbc import skew_sym_matrix

temp = np.identity(3)
A_up = np.hstack((temp,temp,temp,temp))

l1 = skew_sym_matrix([0.26, 0.13,-0.4])
l2 = skew_sym_matrix([0.26, -0.13,-0.4])
l3 = skew_sym_matrix([-0.26, 0.13,-0.4])
l4 = skew_sym_matrix([-0.26, -0.13,-0.4])

A_down = np.hstack((l1,l2,l3,l4))
# print(A_down)

A = np.vstack((A_up,A_down))
F_m = np.mat([0,0,-0]).T
F_t = np.mat([0,10,0]).T
b = np.vstack((F_m,F_t))

w = np.identity(12)
E = 2*A.T*A + w
F = -2*A.T*b


M,gamma = constr_output_set(-50,50,12)


# print(gamma)
# print(np.shape(E) , np.shape(F) , np.shape(M) , np.shape(gamma))
eta, lambda_ = HildrethQPP(E,F,M,gamma)
print(eta)
cond  = Kuhn_Tucker_condition(E,F,M,gamma,eta,lambda_)
# print(cond)
'''






