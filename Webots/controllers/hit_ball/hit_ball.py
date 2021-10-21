#   hit ball robots main function
#   Created in 2019 11/25
#   Vision = 1.0
#   Author Junwen Cui

import numpy as np
from controller import *


class Ball(Supervisor):

    def __init__(self):
        super(Ball, self).__init__()

        self.gps = GPS('gps')
        self.gps.enable(1)
        self.ts = TouchSensor('touch_sensor')
        self.ts.enable(1)
        self.x_ps = PositionSensor('x_ps')
        self.x_ps.enable(1)

        self.max_num = 0

        self.motor = Motor('motor')

    def get_max(self,now):
        if now >= self.max_num:
            self.max_num = now 
        return self.max_num

    def hold_pos(self,hold):

        if hold:           
            self.motor.setPosition(0.3)
        else:
            self.motor.setTorque(0)


robot = Ball()
timestep,times = int(robot.getBasicTimeStep()),0
hold = False
save = True 
while robot.step(timestep) != -1: 
    #pass
    times += 1
    max_data = robot.get_max(robot.x_ps.getValue())
    
    if(times <  (1000/timestep) * 1.5):
        robot.hold_pos(True)
        pass
    else:
        if not hold:
            robot.hold_pos(False)

        if robot.ts.getValue() > 0 or hold:
            hold = True
            robot.hold_pos(True)

            if save:
                save = False
                data_ = []
                data_.append(robot.x_ps.getValue())  #记录冲击角度
                data_.append(times *timestep/1000)                  #记录时间
                np.savetxt('ang.txt',data_,fmt='%0.5f')
                print('angle has been saved!')
        
    






