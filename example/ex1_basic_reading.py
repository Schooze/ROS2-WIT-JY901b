import os
import sys
import math
import numpy as np
import serial
def combine_bytes_angle(DataH, DataL):
    # Convert DataH to a signed 16-bit value and shift it 8 bits to the left
    DataH = (DataH & 0xFF)  # Ensure DataH is within 8 bits (0-255)
    DataL = (DataL & 0xFF)  # Ensure DataL is within 8 bits (0-255)
    
    # Combine the high and low byte
    Data = (DataH << 8) | DataL
    
    # Ensure the value is treated as a signed 16-bit integer
    # if Data & 0x8000:  # Check if the 16th bit (sign bit) is set
    #     Data -= 0x10000  # Convert to negative range by subtracting 65536
    
    return Data
# Arrays to store acceleration values
acc_x_values = []
acc_y_values = []
acc_z_values = []
jy_sensor = serial.Serial(port="/dev/ttyUSB0", baudrate="9600", timeout=1)
print(jy_sensor.name)
while True:
    data = jy_sensor.read(size=1)
    if data == b'\x55':
        print("success!")
        jy_sensor.read(size=10)
        break
    print("trying", data)

try:
    while True:
        data = jy_sensor.read(size=11)
        if not len(data) == 11:
            print('byte error:', len(data))
            break
        #Magnetic
        # print(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7])
        if data[1] == 84:
            x = int.from_bytes(data[2:4], byteorder='little')
            y = int.from_bytes(data[4:6], byteorder='little')
            z = int.from_bytes(data[6:8], byteorder='little')
            # print(f"Magnetic output:{x:.1f}, {y:.1f}, {z:.1f} uT")
        #Angle
        elif data[1] == 83:
            roll = combine_bytes_angle(data[3], data[2])
            roll = roll/32768*180
            pitch = combine_bytes_angle(data[5], data[4])
            pitch = pitch/32768*180
            yaw = combine_bytes_angle(data[7], data[6])
            yaw = yaw/32768*180
            # x = int.from_bytes(data[2:4], byteorder='little')/32768*180
            # y = int.from_bytes(data[4:6], byteorder='little')/32768*180
            # z = int.from_bytes(data[6:8], byteorder='little')/32768*180

            print(f"Angle output:{roll:.3f}, {pitch:.3f}, {yaw:.3f} degrees")

        #Acceleration
        elif data[1] == 81:
            # ax = combine_bytes(data[3], data[2])
            # ax = ax/32768*16*9.8    
            # ay = combine_bytes(data[5], data[4])
            # ay = ay/32768*16*9.8
            # az = combine_bytes(data[7], data[6])
            # az = az/32768*16*9.8
            #ax = ((data[3] << 8)|data[2])/32768*16*9.8
            # ay =((data[5] << 8)|data[4])/32768*16*9.8
            # az = ((data[7] << 8)|data[6])/32768*16*9.8

            # Example calibration values (these should be calculated in your code based on your sensor's readings)
            acc_x_offset = -0.100 # Offset for X-axis, measured during calibration
            acc_y_offset = -0.005  # Offset for Y-axis
            acc_z_offset = 9.770 # Offset for Z-axis (gravity value should be close to 9.81)
            
            axl = data[2]
            axh = data[3]
            ayl = data[4]
            ayh = data[5]
            azl = data[6]
            azh = data[7]
            g = 9.81            # Acceleration due to gravity in m/s^2
            k_acc = 16.0 * g    # Sensitivity of the accelerometer in m/s^2

            acc_x = (axh << 8 | axl) / 32768.0 * k_acc
            acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
            acc_z = (azh << 8 | azl) / 32768.0 * k_acc
            if acc_x >= k_acc:
                acc_x -= 2 * k_acc 
            if acc_y >= k_acc:
                acc_y -= 2 * k_acc 
            if acc_z >= k_acc:
                acc_z -= 2 * k_acc
            # Corrected accelerometer values
            calibrated_acc_x = acc_x - acc_x_offset
            calibrated_acc_y = acc_y - acc_y_offset
            calibrated_acc_z = acc_z - acc_z_offset
            # print(f"Acceleration output:{calibrated_acc_x:.3f}, {calibrated_acc_y:.3f}, {calibrated_acc_z:.3f} m/s^2")
            # Append values to arrays for later calculation

            # Callibrate the accelerometer
            # acc_x_values.append(acc_x)
            # acc_y_values.append(acc_y)
            # acc_z_values.append(acc_z)
            # print("APPEND")
            # # After collecting enough samples (e.g., 100 readings), calculate offsets
            # if len(acc_x_values) > 100:
            #     # Calculate the average value for each axis
            #     avg_acc_x = np.mean(acc_x_values)
            #     avg_acc_y = np.mean(acc_y_values)
            #     avg_acc_z = np.mean(acc_z_values)

            #     # Calculate offsets
            #     acc_x_offset = avg_acc_x
            #     acc_y_offset = avg_acc_y
            #     acc_z_offset = avg_acc_z - 9.81  # Z should be near gravity, so subtract 9.81 for offset

            #     print(f"Offsets (X, Y, Z): {acc_x_offset:.3f}, {acc_y_offset:.3f}, {acc_z_offset:.3f} m/s^2")

            #     # Clear the lists for the next batch of readings
            #     acc_x_values.clear()
            #     acc_y_values.clear()
            #     acc_z_values.clear()
        elif data[1] == 86:
            p0 = data[2]
            p1 = data[3]
            p2 = data[4]
            p3 = data[5]
            h0 = data[6]
            h1 = data[7]
            h2 = data[8]
            h3 = data[9]
            
            p = (p3 << 24) | (p2 << 16) | (p1 << 8) | p0
            h = (h3 << 24) | (h2 << 16) | (h1 << 8) | h0
            # print(f"Pressure output:{p:.1f} Pa")
            # print(f"Height output:{h:.1f} cm")
        # print("----",data[0], data[1])
except KeyboardInterrupt:
    jy_sensor.close()
    print("close port")