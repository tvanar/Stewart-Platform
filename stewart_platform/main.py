import numpy as np
import serial
import struct
from scipy.spatial.transform import Rotation as R



def legLength(V_x):
    V_2 = 4.91
    V_1 = 0.226
    F = 30/(V_2-V_1)*(V_x-V_1) + 149
    return F
