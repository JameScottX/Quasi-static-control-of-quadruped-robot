#   ROS interface
#   Updated in 2020 1/4
#   Author Junwen Cui / JameScottX
#   Other: 

import time
import roslibpy
import numpy as np 

# talker = roslibpy.Topic(client, '/nininin', 'std_msgs/String')

# while client.is_connected:
#     talker.publish(roslibpy.Message({'data': 'Hello World!'}))
#     print('Sending message...')
#     time.sleep(1)

# talker.unadvertise()

# client.terminate()



class MSG2ROS(object):

    def __init__(self):

        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()
        self.joint_talker = roslibpy.Topic(self.client, '/joint_info', 'std_msgs/Float64MultiArray')
        self.body_talker = roslibpy.Topic(self.client, '/body_info', 'std_msgs/Float64MultiArray')
        self.glb_foot_talker = roslibpy.Topic(self.client, '/glb_foot_info', 'std_msgs/Float64MultiArray')
        self.qp_force_talker = roslibpy.Topic(self.client, '/qp_force_info', 'std_msgs/Float64MultiArray')
        self.traj_talker = roslibpy.Topic(self.client, '/traj_info', 'std_msgs/Float64MultiArray')
        
        self.temp_talker = roslibpy.Topic(self.client,'/temp_info', 'std_msgs/Float64MultiArray')

        self.count = 0
        self.peroid = 10

        pass    
    
    def count_rec(self):
        # 控制频率功能
        self.count +=1
        if self.count % self.peroid ==0:
            self.count = 0
        return self.count

    def send_joint(self, q_leg):
        # 发送关节角度
        q_leg_ = list(q_leg)
        if len(q_leg_)!=12:
            print("Error about send_joint q_leg ")
            return
        self.joint_talker.publish(roslibpy.Message({'data':  q_leg_}))
        pass
    
    def send_body(self, q_body):
        # 发送body 位置和姿态 
        q_body_ = list(q_body)
        if len(q_body_)!=7:
            print("Error about send_body q_body ")
            return
        self.body_talker.publish(roslibpy.Message({'data':  q_body_}))

    def send_foot(self, glb_foot):
        # 发送全局框架下 脚末端位置
        glb_foot_ = np.array(glb_foot).flatten()
        glb_foot_ = list(glb_foot_)
        if len(glb_foot_)!=12:
            print("Error about send_foot glb_foot ")
            return
        self.glb_foot_talker.publish(roslibpy.Message({'data':  glb_foot_}))


    def send_qp_force(self, foot_glb, f_qp, ratio = 0.005):

        # 发送 qp 计算的分配力
        f_qp_ =  np.array(f_qp).flatten()
        foot_glb_ = np.array(foot_glb).flatten()
        f_qp_ = list(ratio * f_qp_)
        foot_glb_ = list(foot_glb_)
        foot_glb_.extend(f_qp_)
      
        if len(foot_glb_)!=24:
            print("Error about send_qp_force foot_glb &  f_qp ")
            return
        self.qp_force_talker.publish(roslibpy.Message({'data':  foot_glb_}))

    def send_traj(self, traj_pots):
        # 发送多条三维轨迹 
        traj_pots_ = np.array(traj_pots).flatten() 
        #第一个量为 轨迹数量
        traj_pots_ = np.insert(traj_pots_,0, int((traj_pots_.shape[0])/3) ,axis = 0)
        traj_pots_ = list(traj_pots_)
        # print(" traj_pots_  ", len(traj_pots_) )
        if (len(traj_pots_)-1)%3 !=0:
            print("Error about send_traj traj_pots ")
            return
        self.traj_talker.publish(roslibpy.Message({'data':  traj_pots_}))

    def send_temp(self, temp):
        temp_ = np.array(temp).flatten()
        temp_ = list(temp_)
        self.temp_talker.publish(roslibpy.Message({'data':  temp_}))



# msg2ros = MSG2ROS()
# while msg2ros.client.is_connected:
#     msg2ros.send_joint(1)
#     print('Sending message...')
#     time.sleep(1)

# msg2ros.joint_talker.unadvertise()
# msg2ros.client.terminate()