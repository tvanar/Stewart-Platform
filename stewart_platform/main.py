import numpy as np
import serial
import struct
import kinematics

voltage_buffer = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]

def avg(measurements):
    global voltage_buffer
    voltage_buffer = [measurements] + voltage_buffer[:-1]
    column_sums = [sum(col) for col in zip(*voltage_buffer)]
    return [s / 6 for s in column_sums]

def legLength(V_x):
    V_2 = 4.91
    V_1 = 0.226
    F = 30/(V_1-V_2)*(V_x-V_2) + 149
    return F



#setup
<<<<<<< Updated upstream
ser = serial.Serial('COM14', 115200) 
=======
ser = serial.Serial('/dev/tty.usbmodemHIDPC1', 115200) 
>>>>>>> Stashed changes
p = kinematics.StewartPlatform()


# Initial Position
resting_position = [0,0,150,0,0,0]
position = resting_position.copy()
leg_lengths = []
old_pos = []
count = 0

while True:

    # Reading voltages from Leonardo
    voltages = struct.unpack('6f', ser.read(24))
    average = avg(list(voltages))
    # Calculating Voltages
    leg_lengths = [legLength(i) for i in average]
    # Calulating forward kinematics
    old_pos = position.copy()
    position = p.forward_kinematics(position,leg_lengths)
    if np.abs(np.linalg.norm(old_pos)-np.linalg.norm(position)) > 50:
        position = resting_position.copy()
    count = count +1
    if count == 5:
        print('lengths: ', leg_lengths)
        print('position: ', position)
        count = 0
    # Sending position
    position_32 = [np.float32(i) for i in position]
    data = struct.pack('6f', *position)
    ser.write(data) 
    
    

    
    
    
    
    



