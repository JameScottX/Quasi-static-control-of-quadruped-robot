#   Data drawing methods
#   Update in 2019 12/18
#   Vision = 1.3
#   Author Junwen Cui

'''
matplotlib 具体参考网址
https://matplotlib.org/3.1.1/gallery/index.html
'''

#coding:utf-8
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from   matplotlib.font_manager import FontProperties
font_set = FontProperties(fname=r"c:\windows\fonts\simsun.ttc", size=12)


#定义数据记录结构体
#可以在绘图文件中定义，获取数据
class data_record():

    def __init__(self, *args, **kwargs):
              
        self.data_x = []      ##此处需要进行转置
        self.data_y = []      ##此处需要进行转置

        if 'data_name' not in kwargs.keys():
            raise ValueError('data_name can not been none!')
        self.data_name = kwargs['data_name']
        self.len_ = len(self.data_name)
        self.is_saved = False

    def save_data_times(self,x_data,y_data):

        data_x_ = [x_data for i in range(self.len_)]        
        self.data_x.append(data_x_)##此处需要进行转置
        self.data_y.append(y_data)##此处需要进行转置




#数据保存文件 格式 .txt or .mat
#输入参数  文件名.txt or .mat
#          数据集
#          是否以y的index作为 X 
#          小数位数
#返回      文件存储路径
def data_to_save(file,data_y,index_x = None ,format = '.txt',dot_len = 5):

    data_y = np.array(data_y)
    if data_y.ndim !=2:
        raise ValueError('data_y ndim must be 2')

    x_ = []
    num_data = np.shape(data_y)[0]

    if index_x.all()!=None:   # x的数据已经指定
        index_x = np.array(index_x)
        if np.shape(index_x) != np.shape(data_y):
            
            raise ValueError('index_x and data_y must be same shape!!!')

        for i in range(num_data):
            x__ = index_x[i]
            x_.append(x__)
        x_ = np.array(x_)
    else :                  # x的数据未指定使用默认序号
        for i in range(num_data):
            x__ = [ j for j in range(len(data_y[i]))]          
            x_.append(x__)
        x_ = np.array(x_)
        x_ = x_.astype(np.int)
        
    if format =='.txt':
        np.savetxt(file + '.txt',data_y,fmt='%0.'+str(dot_len)+'f')
        np.savetxt(file + '__x.txt',x_,fmt='%0.'+str(dot_len)+'f')
        return file + '__x.txt', file + '.txt'

    elif format == '.mat':               
        sio.savemat(file+ '.mat', {'X': x_, 'Y': data_y})
        return file+ '.mat'


#画二维图函数
#输入参数  文件名
#          标题，label x,y名称 ,lenged 名称
#          txt文件的 x 数据 
#          其他
def draw_plot(file,title,label_name_x,label_name_y,label = None,file_txt_x = None,**kwargs):

    show = True        #是否显示
    save = True        #保存图
    title_size = 20    #标题大小
    label_size = 15    #label 大小  
    fig_size = (8,8)   #调整图大小 800 *800
    sub_plot = False   #画多个子图

    for name, value in kwargs.items():
        if name =='label_size':label_size = value
        elif name =='title_size':title_size = value
        elif name =='save':save = value
        elif name =='show':show = value
        elif name =='fig_size':fig_size = value
        elif name =='sub_plot':sub_plot = value

    if file.endswith('.txt') and file_txt_x.endswith('.txt'):
        x_ ,y_ = np.loadtxt(file_txt_x,dtype=np.float64),np.loadtxt(file,dtype=np.float64)       
    elif file.endswith('.mat'):      #.mat 文件无需考虑  file_txt_x
        data = sio.loadmat(file)
        x_,y_  = data['X'],data['Y']
        
    if sub_plot==False : plt.figure(figsize = fig_size)

    plt.title(title,fontproperties='SimHei',fontsize=title_size)

    l_index = 0
    for i in range(np.shape(x_)[0]):
        l_index =  len(label)-1  if i >= len(label) else i
        plt.plot(x_[i] ,y_[i],label = label[l_index],linewidth=1)
        
    plt.legend(label,loc = 0, ncol = 2, shadow=True) 
    plt.xlabel(label_name_x, fontproperties ='SimHei',fontsize=label_size)
    plt.ylabel(label_name_y, FontProperties = 'SimHei',fontsize=label_size)

    if save and not sub_plot :plt.savefig(file + '.jpg')
    if show and not sub_plot :plt.show()

'''
例子1: 画多个plot例子

data = [[2.44848456,5.21155484,1.548611],
        [7.848456,9.21155484,5.548611]]

data_to_save('datax',data,index_x = None,format = '.mat')

plt.figure(figsize = (8,8))

for i in range(4):
    ax = plt.subplot(221+i)   
    plt.subplots_adjust(wspace =.2, hspace =.3)#调整子图间距

    draw_plot('datax.mat',title = 'draw',
    label_name_x = 'name_x',label_name_y = 'name_y',
    file_txt_x = 'datax__x.txt',label = ['1','2'],    #  'datax__x.txt' 没用
    fig_size = (1,1),sub_plot = True)

plt.show()
'''

'''
例子2: 记录webots机器人属性

dr_xyz= data_record(data_name = ['CoM-x','CoM-y','CoM-z'])
dr_xyz.save_data_times(times*timestep/1000,
                               [robot.gps_pos_now.x, robot.gps_pos_now.y, robot.gps_pos_now.z])
if times > (1000/timestep) * 8 and not dr_xyz.is_saved:
            dr_xyz.is_saved = True
            data_to_save('dr_xyz',np.array(dr_xyz.data_y).T,np.array(dr_xyz.data_x).T,'.txt')

plt.figure(figsize = (8,8))
ax = plt.subplot(311)   
plt.subplots_adjust(wspace =.2, hspace =.3)#调整子图间距

draw_plot('dr_xyz.txt','','time (s)','pos (m)',label = ['COM-x','COM-y','COM-z'],
          file_txt_x ='dr_xyz__x.txt' ,sub_plot =True)
plt.savefig('singled.jpg')
plt.show()
'''


'''
以下为各部分案例
包含有 动态图
       3D图
       热力图等
'''

from mpl_toolkits.mplot3d import axes3d 
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


#动态画图案例
def animation_():

    fig = plt.figure(figsize=(8, 6), dpi=80)
    plt.ion()

    for index in range(100):

        plt.cla()

        plt.title("动态曲线图")
        plt.grid(True)

        x = np.linspace(-np.pi + 0.1*index, np.pi+0.1*index, 256, endpoint=True)
        y_cos, y_sin = np.cos(x), np.sin(x)

        plt.xlabel("X轴")
        plt.xlim(-4 + 0.1*index, 4 + 0.1*index)
        plt.xticks(np.linspace(-4 + 0.1*index, 4+0.1*index, 9, endpoint=True))

        plt.ylabel("Y轴")
        plt.ylim(-1.0, 1.0)
        plt.yticks(np.linspace(-1, 1, 9, endpoint=True))

        plt.plot(x, y_cos, "b--", linewidth=2.0, label="cos示例")
        plt.plot(x, y_sin, "g-", linewidth=2.0, label="sin示例")
        plt.legend(loc="upper left", shadow=True)

        plt.draw()
        plt.pause(0.01)
        plt.close()

    plt.ioff()
    
    plt.show()

#3D绘图案例1
def example_3d_1():

    #初始化界面
    fig = plt.figure(figsize = (8,8))
    ax = fig.gca(projection='3d')

    #细分
    L = np.linspace(0,0.1,50)
    theta_ = np.linspace(0,np.pi/2,100)

    #分网格
    L, theta_ = np.meshgrid(L, theta_)

    #函数
    s =   L * np.sin(theta_) / np.cos(theta_/2)
    limit_f = (20000. * 1.* s * np.sin(theta_/2.) + 16.44 * 9.81 /1.)*0.8  #摩擦力
    out_f = 20000. *s * np.cos(theta_/2.)

    surf = ax.plot_surface(L, theta_,limit_f - out_f, rstride=2, cstride=2, alpha=1,cmap=cm.coolwarm,linewidth=1, antialiased=True)
    #ax.plot_wireframe(L, theta_,out_f, rstride=2, cstride=2, alpha=1)

    #映射到平面
    cset = ax.contour(L, theta_,limit_f - out_f, zdir='z', offset=-800, cmap=cm.coolwarm)
    cset = ax.contour(L, theta_,limit_f - out_f, zdir='x', offset=0, cmap=cm.coolwarm)
    cset = ax.contour(L, theta_,limit_f - out_f, zdir='y', offset=np.pi/2, cmap=cm.coolwarm)

    ax.set_zlim(-800,500)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))


    fig.colorbar(surf, shrink=0.6, aspect=2)

    ax.set_xlabel('L')
    ax.set_ylabel('theta_')
    ax.set_zlabel('dir')

    ax.set_xlim(0, 0.1)
    ax.set_ylim(0, np.pi/2)
    ax.set_zlim( -800,500)
    plt.show()


#等高线图案例
def heatmap_example():

    #建窗口
    fig = plt.figure(figsize = (8,8))
    #细分
    L = np.linspace(0,0.1,50)
    theta_ = np.linspace(0,np.pi/2,100)
    #分网格
    L,theta_ = np.meshgrid(L, theta_)
    #函数
    s =   L * np.sin(theta_) / np.cos(theta_/2)
    limit_f = (20000. * 1.* s * np.sin(theta_/2.) + 16.44 * 9.81 /1.)*0.8  #摩擦力
    out_f = 20000. *s * np.cos(theta_/2.)

    hist = limit_f - out_f

    #绘图
    plt.contourf(L,theta_, hist, 10, alpha=1) #cmap=cm.hot
    #plt.plot([0,0.1],[0,0.1],'ro--', linewidth=2,)

    plt.grid(True)
    plt.colorbar()
    plt.show()

