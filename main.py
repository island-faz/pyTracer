from PIL import Image

import numpy as np
import math  

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def rs_intersect(R0, d, C, r):
    R0 = R0 - C
    dDotR0 = np.dot(d, R0)
    delta = (dDotR0*dDotR0 - np.dot(R0, R0) + r*r)
    if delta >= 0 :
        t = -dDotR0 - math.sqrt( delta )
        return (R0[0] + t*d[0], R0[1] + t*d[1], R0[2] + t*d[2])
    return None

frameWidth = 128
frameHeight = 128
data = np.zeros((frameWidth, frameHeight, 3), dtype=np.uint8)


d = 1
cam_o = np.array([0, 0, -2])

x = np.array([1, 0, 0])
y = np.array([0, 1, 0])

theta = math.pi / 4 # FOV HORIZONTAL
phi = math.pi / 4 # FOV VERTICAL

u_vector = (d * math.tan(theta))  * x
v_vector = (d * math.tan(phi))    * y

sphere_o = np.array([0, 0, 10])

for i in range(frameWidth):
    for j in range(frameHeight):
        alpha =     (2 * (i)) / frameWidth - 1
        beta  = 1 - (2 * (j)) / frameHeight

        VecDir = (alpha * u_vector + beta * v_vector) - cam_o # vecteur directeur du rayon

        d = rs_intersect(cam_o, normalize(VecDir), sphere_o, 5)

        if d != None :
            data[i, j] = [255, 0, 0]


im =  Image.fromarray(data) #Image.new('RGB', (512, 512))
im.save('test2.png')

