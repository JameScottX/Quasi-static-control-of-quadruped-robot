import numpy as np

def euler2quat(z=0, y=0, x=0):
    
    z = z/2.0
    y = y/2.0
    x = x/2.0
    cz = np.cos(z)
    sz = np.sin(z)
    cy = np.cos(y)
    sy = np.sin(y)
    cx = np.cos(x)
    sx = np.sin(x)
    return np.array([
             cx*cy*cz - sx*sy*sz,
             cx*sy*sz + cy*cz*sx,
             cx*cz*sy - sx*cy*sz,
             cx*cy*sz + sx*cz*sy])

temp = euler2quat(z= 1.5708)


# print(temp)
# print(np.sum(np.square(temp)))