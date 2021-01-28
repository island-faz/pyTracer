from PIL import Image

import numpy as np
import math  

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def rs_intersec(r_o, r_d, s_o, s_r):
    oc = r_o - s_o
    a = np.dot(r_d, r_d)
    b = 2 * np.dot(oc, r_d)
    c = np.dot(oc, oc) - s_r * s_r
    delta = b * b - 4 * a * c
    if delta < 0. :
        return None
    t = (-b - math.sqrt(delta)) / (2 * a)
    inter_p = (r_o + t * r_d) # point d'intersection plus proche
    squared_dist = np.sum((r_o - inter_p)**2, axis=0)
    dist = np.sqrt(squared_dist)
    return dist

frameWidth = 320
frameHeight = 320

data = np.zeros((frameHeight, frameWidth, 3), dtype=np.uint8)


d = 1
cam_o = np.array([0, 0, -1])

x = np.array([-1, 0, 0])
y = np.array([0, -1, 0])

theta = math.pi / 4 # FOV HORIZONTAL
phi = math.pi / 4 # FOV VERTICAL

u_vector = (d * math.tan(theta))  * x
v_vector = (d * math.tan(phi))    * y

sphere_o = np.array([0, 0, 1.5])

for i in range(frameHeight):
    """if i%2:
        continue"""
    for j in range(frameWidth):
        """if i%2:
            continue"""
        alpha =     (2 * (i)) / frameHeight - 1
        beta  = 1 - (2 * (j)) / frameWidth

        VecDir = (alpha * u_vector + beta * v_vector) - cam_o # vecteur directeur du rayon

        d = rs_intersec(cam_o, normalize(VecDir), sphere_o, 1.5)

        if d != None :
            color = max(10, 255 - d * d * 60)
            print(color)
            data[i, j] = [color, color, color]


im =  Image.fromarray(data)
im.save('test8.png')

