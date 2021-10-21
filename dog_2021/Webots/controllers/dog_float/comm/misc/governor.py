#   Debug interface based on PyQT
#   Update in 2019 12/19
#   Vision = 1
#   Author Junwen Cui

'''
仿真调试界面
通过命令给参数
'''

import time
import random
#from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow,QWidget,QHBoxLayout,QVBoxLayout,QLabel,\
QApplication,QTextEdit,QPushButton,QSizePolicy

from PyQt5.QtGui import QFont,QPalette,QColor
from PyQt5.QtCore import Qt,QEvent,QTimer
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


from comm.action.new_vmc import *

class Robot():

    def __init__(self):

        self.width = 0.13
        self.length = 0
        self.height = 0.1

        self.stand_height = 0.2

        self.virtual_height = 0.05
        self.leg_num = 2


        self.pos_now = [0,0,0,0,0,0]
        self.pos_des = [0,0,0,0,0,0]
        self.leg_loc = np.array([[0,self.virtual_height,self.width/2],[0,self.virtual_height,-self.width/2]])

        self.body_frame = []


        pass




'''
3D窗口部分
'''
class MyDynamicMplCanvas(FigureCanvas):

    def __init__(self, width=10, height=4, dpi=100,show_size = 0.5):
        
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111, projection='3d')

        super(MyDynamicMplCanvas, self).__init__(fig)

        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

        self.figure.tight_layout()  # uses most available frame space
        self.axes.mouse_init(rotate_btn=1, zoom_btn=3)  # enables mouse interactions
        

        self.compute_initial_figure()
        
        timer = QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(100)

        self.show_size = show_size


        '''
        以下为机器人绘图部分
        '''
        self.biped = Robot()


        



    def compute_initial_figure(self):
        pass



    def trans_pos(self,points):

        points = np.array(points)
        trans_ = np.mat([[1 ,0, 0],[0,0,-1],[0,1,0]])   #转置矩阵
        frame = []
        num_ =np.shape(points)[0]
        #print(np.shape(points))
        for i in range(num_):                
            temp = np.reshape(points[i],(3,1))
            frame.append(trans_ * np.mat(temp))

        frame = np.squeeze(np.array(frame)).T

        return frame

    def update_figure(self):

        self.axes.cla()
        #n_= 10*np.random.rand()
        #n_2= 10*np.random.rand()
        #n_3= 10*np.random.rand()
        
        #x,y,z = [0,n_,2],[0,n_2,2],[0,1,0]
        #self.axes.plot(x, y, z, c='r')
      
        #print(np.shape(self.body_frame))
        #print(np.shape(self.body_frame[0].T))


        self.vir_h_r = np.array([[0,0,-self.biped.width/2],[0,self.biped.virtual_height,-self.biped.width/2]])
        self.vir_h_r +=np.array([[0,self.biped.stand_height,0] for i in range(2)])

        frame = self.trans_pos(self.vir_h_r)
        x= frame[0]
        y= frame[1]
        z= frame[2]
        self.axes.plot(x, y, z, c='b',linewidth=4)

        self.vir_h_l = np.array([[0,0,self.biped.width/2],[0,self.biped.virtual_height,self.biped.width/2]])
        self.vir_h_l +=np.array([[0,self.biped.stand_height,0] for i in range(2)])

        frame = self.trans_pos(self.vir_h_l)
        x= frame[0]
        y= frame[1]
        z= frame[2]
        self.axes.plot(x, y, z, c='b',linewidth=4)

       
        self.biped.pos_now = [0,self.biped.stand_height +self.biped.height/2 ,0,0.,0.,0.4]
        self.biped.pos_des= [0,self.biped.stand_height +self.biped.height/2,0,0,0.,0]
        
        error_last = [[0 for j in range(3)] for i in range(2)]

        error_last,force =  force_out(self.biped.pos_now , self.biped.pos_des,error_last ,self.biped.leg_loc ,0 ,
                                      [1,1],10, 0,biped = True)


        force = 0.5*force
        

        force_r = [self.vir_h_r[1]]
        force_ = force[1] + self.vir_h_r[1]
        force_r = np.concatenate((force_r,[force_]),axis=0)
        #print(force_l)
        frame = self.trans_pos(force_r)
        x= frame[0]
        y= frame[1]
        z= frame[2]
        self.axes.plot(x, y, z, c='y',linewidth=3)

        force_l = [self.vir_h_l[1]]
        force_ = force[0] + self.vir_h_l[1]
        force_l = np.concatenate((force_l,[force_]),axis=0)
        #print(force_l)
        frame = self.trans_pos(force_l)
        x= frame[0]
        y= frame[1]
        z= frame[2]
        self.axes.plot(x, y, z, c='y',linewidth=3)



        self.body_frame = np.array([[0,self.biped.height,-self.biped.width/2],
                           [0,self.biped.height,self.biped.width/2],
                           [0,0,self.biped.width/2],
                           [0,0,-self.biped.width/2],
                           [0,self.biped.height,-self.biped.width/2]])
        self.body_frame +=np.array([[0,self.biped.stand_height,0] for i in range(5)])

        frame = self.trans_pos(self.body_frame)
        x= frame[0]
        y= frame[1]
        z= frame[2]
        self.axes.plot(x, y, z, c='r')



        self.axes.set_xlabel('X')
        self.axes.set_ylabel('Y')
        self.axes.set_zlabel('Z')
        self.axes.set_xlim(-self.show_size,self.show_size)
        self.axes.set_ylim(-self.show_size, self.show_size)
        self.axes.set_zlim(0,self.show_size)
        self.draw()


'''
控制界面
'''
class ApplicationWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        
        self.gui_init()
        dsads = {'asd':1,'adasda':2}
        self.label_show(**dsads)

        
    def label_show(self,**kwargs):

        text_ = ''
        for name, value in kwargs.items():
            text_ += (name +': '+str(value)+'\n')
  
        self.info_lab.setText(text_)


    def command(self):

        data = self.shell.toPlainText()
        data = data[-100:]
        command_ = data.split('#')[-1]
        print(command_)
        
    def _shellclear(self):

        self.shell.clear()
        self.shell.append('type your command to control')
        self.shell.append('#')

    def gui_init(self):

        self.setWindowTitle('Robot debugger with Webots 2020a')
        self.resize(1200,800)
        self.setFixedSize(1200,800)

        self._main = QWidget()
        self.setCentralWidget(self._main)

        h_lay = QHBoxLayout(self._main)   #水平layout
        v_lay_l = QVBoxLayout() #左边竖直
        v_lay_r = QVBoxLayout() #右边竖直

        h_lay.addLayout(v_lay_l)
        h_lay.addLayout(v_lay_r)

        #右上试图部分
        self.info_lab = QLabel()
        v_lay_r.addWidget(self.info_lab)
        self.info_lab.resize(200,200)
        self.info_lab.setFixedSize(200,200)
        self.info_lab.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.info_lab.setFont(QFont("Roman times",10,QFont.Bold))
        pe = QPalette()
        pe.setColor(QPalette.WindowText,Qt.red)
        self.info_lab.setPalette(pe)


        #右下视图
        shell_name = QLabel()
        shell_name.setText('Command Shell')
        shell_name.setFont(QFont("Roman times",18,QFont.Bold))
        v_lay_r.addWidget(shell_name)

        self.shell = QTextEdit()
        v_lay_r.addWidget(self.shell)
        self.shell.setFixedWidth(300)
        self.shell.setFont(QFont("Roman times",10))
        self.shell.append('type your command to control')
        self.shell.append('#')
        self.shell.installEventFilter(self) 

        self.clear_shell_clear = QPushButton('CLEAR')
        self.clear_shell_command = QPushButton('COMMAND')
        self.clear_shell_clear.setFont(QFont("Roman times",14,QFont.Bold))
        self.clear_shell_command.setFont(QFont("Roman times",14,QFont.Bold))
        self.clear_shell_clear.released.connect(self._shellclear)  #清除信号槽
        self.clear_shell_command.released.connect(self.command)

        h_lay__ = QHBoxLayout(self._main)
        h_lay__.addWidget(self.clear_shell_clear)
        h_lay__.addWidget(self.clear_shell_command)
        v_lay_r.addLayout(h_lay__)


        #matplotlib 部分   左上部分
        self.canvas = MyDynamicMplCanvas()
        self.addToolBar(Qt.BottomToolBarArea,
                        NavigationToolbar(self.canvas, self))

        v_lay_l.addWidget(self.canvas)
        

    def  eventFilter(self,obj, event):

        if obj == self.shell:

            if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Return:
              
                    self.command()
                    self.shell.append('#')
                    return True

        return QWidget.eventFilter(self,obj,event)



