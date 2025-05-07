import numpy as np
import serial
import struct
import kinematics

def legLength(V_x):
    V_2 = 4.91
    V_1 = 0.226
    F = 30/(V_2-V_1)*(V_x-V_1) + 149
    return F


#setup
ser = serial.Serial('COM4', 115200) 
p = kinematics.StewartPlatform()

# Initial Position
resting_position = [0,0,160,0,0,0]
position = resting_position.copy()
leg_lengths = []

while True:

    # Reading voltages from Leonardo
    voltages = struct.unpack('6f', ser.read(24))
    print(voltages)
    # Calculating Voltages
    leg_lengths = [legLength(i) for i in voltages]
    print('lengths: ', leg_lengths)
    # Calulating forward kinematics
    position = p.forward_kinematics(position,leg_lengths)
    
    print('position: ', position)
    # Sending position
    position_32 = [np.float32(i) for i in position]
    data = struct.pack('6f', *position)
    ser.write(data)
    
    
    
    
    



