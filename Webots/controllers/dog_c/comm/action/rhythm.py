#   Gait rhythm function file
#   Updated in 2020 1/12
#   Author Junwen Cui / JameScottX
#   Other: use numba to accelarate

import numpy as np

class Rhy_TriWalk(object):
    #机器人静态行走
    def __init__(self):

        self.f_dis_touch = 0.05
        self.h_dis_touch = 0.05
        self.grivtypot = np.array([0.0, 0.0])
        self.u_or_d = -1
        

        pass

    def __line_get(self, pot1, pot2):
        #求直线表达式
        A = pot2[1] - pot1[1]
        B = pot1[0] - pot2[0]
        C = pot2[0] * pot1[1] - pot1[0]*pot2[1]
        return A,B,C

    def line_dis(self, pots):
        #求支撑中线　垂直距离
        A,B,C = self.__line_get(pots[0],pots[1])
        self.u_or_d = np.sign(self.grivtypot[1] + (A * self.grivtypot[0] + C )/ B )
        d = np.fabs( A * self.grivtypot[0] + B * self.grivtypot[1] + C) / np.sqrt(A**2 + B**2)
        real_d = self.u_or_d * d
        return real_d

    def line_dis_j(self, pots, swing_id):
        pots_diag = []
        if swing_id ==3 or swing_id==0:
            pots_diag = np.array([pots[1],pots[2]])
        elif swing_id ==2 or swing_id ==1:
            pots_diag = -np.array([pots[0],pots[3]])
        
        return self.line_dis(pots_diag)



if __name__ == '__main__':

    rhy = Rhy_TriWalk()
    real_d = rhy.line_dis([[0.1, 0.2000],[-0.1,-0.1]])
    print(real_d)




