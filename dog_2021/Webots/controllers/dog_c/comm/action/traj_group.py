#   Other file
#   Updated in 2020 1/12
#   Author Junwen Cui / JameScottX
#   Other: 

import numpy as np


def traj_3pots(sta, end, T, up = 0.2):

    sta = np.array(sta)
    end = np.array(end)
    toc = end + np.array([0,0,-0.2])

    mid = (end + sta)/2  #+ (end - sta)/8
    mid[2] += up

    speed_mid = 2*(end - sta) / T
    speed_mid[2] = 0.0

    speed_sta = np.array([0,0,2*up/T])
    pots = np.vstack( (sta, mid, end, toc) )  
    speed = np.vstack( (speed_sta, speed_mid, np.zeros(3,dtype=float), np.zeros(3,dtype=float) ) )
    acc = np.zeros((4,3), dtype=float)


    return pots, speed, acc

