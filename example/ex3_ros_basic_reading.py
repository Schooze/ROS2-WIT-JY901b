#!/usr/bin/env python3
"""

Author : Syahid
Date Modified : Feb 14 2025

Program yang digunakan untuk membaca data IMU dari sensor WIT-JY901b
Data yang dibaca meliputi:
- Acceleration
- Angular Velocity
- Angle
- Magnetic
- Data Output Port Status
- Quaternion

Metode : Pembacaan USB
Deskripsi : 
Cara pakai  :
- Pastikan sensor WIT-JY901b sudah terhubung dengan USB
- tulis perintah:
    ls /dev/ttyUSB*
- Ubah Config.SERIAL_PORT sesuai dengan port yang digunakan
- Jalankan program
- Program akan membaca data IMU dari sensor WIT-JY901b


"""


import rclpy
import os
import sys
import math
import numpy as np
import serial
from rclpy.node import Node
from example_interfaces.msg import String

class Config:
    ########################################
    """
    Serial port configuration
    """
    SERIAL_PORT = "/dev/ttyUSB0"    # Ubah sesuai port yang digunakan
    BAUDRATE = 9600                 # Ubah sesuai baudrate yang digunakan
    TIMEOUT = 1                     # Ubah sesuai timeout yang digunakan
    
    ########################################
    """
    PUBLISH TOPIC ENABLE
    """
    PUBLISH_ACCELERO = False
    PUBLISH_ANGULAR_VELOCITY = False
    PUBLISH_ANGLE = False
    PUBLISH_MAGNETIC = False
    PUBLISH_QUATERNION = False
    
    ########################################
    """
    Acceleration offset values
    """
    ACC_X_OFFSET = 0.0
    ACC_Y_OFFSET = 0.0
    ACC_Z_OFFSET = 0.0
    
    ########################################
    """
    Constant values
    """
    G = 9.80665                     # Gravitational acceleration (m/s^2)
    RAD_TO_DEG = 180.0 / math.pi    # Radians to degrees conversion factor
    DEG_TO_RAD = math.pi / 180.0    # Degrees to radians conversion factor
    
    ########################################
    """
    Calibration flag
    """
    ACCELERO_CALIBRATION = False    # Accelerometer calibration flag
 

class IMU_Node(Node):
    def __init__(self):
        super().__init__("WITJY901b") #Node Name
        self.subscriber_ = self.create_subscription(String, "imu", self.callback_robot_news, 10) #(tipe data, topics, proses msg, queue)
        self.get_logger().info("WIT-JY901b Node has been started")
        
        if Config.ACCELERO_CALIBRATION:
            self.acc_x_values = []
            self.acc_y_values = []
            self.acc_z_values = []
        
        self.IMU_init() #Initialize IMU
        self.read_IMU() #Read IMU

    def callback_robot_news(self, msg): #To Handle Massage
        self.get_logger().info(msg.data)
        
    def IMU_init(self):
        self.jy_sensor = serial.Serial(port=Config.SERIAL_PORT, baudrate=Config.BAUDRATE, timeout=Config.TIMEOUT)
        print(self.jy_sensor.name)
        while True:
            data = self.jy_sensor.read(size=1)
            if data == b'\x55':
                print("Packet header found!")
                self.jy_sensor.read(size=10)
                break
            print("trying", data)
 
    def read_IMU(self):
        while True:
            data = self.jy_sensor.read(size=11)
            if not len(data) == 11:
                print('byte error:', len(data))
                break
            
            # Acceleration
            if Config.PUBLISH_ACCELERO:
                if data[1] == 81:
                    axl = data[2]
                    axh = data[3]
                    ayl = data[4]
                    ayh = data[5]
                    azl = data[6]
                    azh = data[7]
                    
                    # Acceleration sensitivity (16.0 m/s^2 per 32768 LSB)
                    k_acc = 16.0
                    
                    self.acc_x = ((axh << 8) | axl) / 32768.0 * k_acc
                    self.acc_y = ((ayh << 8) | ayl) / 32768.0 * k_acc
                    self.acc_z = ((azh << 8) | azl) / 32768.0 * k_acc
                    
                    if self.acc_x >= k_acc:
                        self.acc_x -= 2 * k_acc
                    if self.acc_y >= k_acc:
                        self.acc_y -= 2 * k_acc
                    if self.acc_z >= k_acc:
                        self.acc_z -= 2 * k_acc
                        
                    if Config.ACCELERO_CALIBRATION:
                        self.calculate_accelero_offset()
                    else:
                        self.calibrated_acc_x = self.acc_x - Config.ACC_X_OFFSET
                        self.calibrated_acc_y = self.acc_y - Config.ACC_Y_OFFSET
                        self.calibrated_acc_z = self.acc_z - Config.ACC_Z_OFFSET
                        
                        print(f"Acceleration output before: {self.acc_x:.3f}, {self.acc_y:.3f}, {self.acc_z:.3f} m/s^2")
                        
                        print(f"Acceleration output after : {self.calibrated_acc_x:.3f}, {self.calibrated_acc_y:.3f}, {self.calibrated_acc_z:.3f} m/s^2\n\n")
                
            # Angular velocity output
            if Config.PUBLISH_ANGULAR_VELOCITY:
                if data[1] == 82:
                    wxL = data[2]
                    wxH = data[3]
                    wyL = data[4]
                    wyH = data[5]
                    wzL = data[6]
                    wzH = data[7]
                    
                    # Angular velocity sensitivity (2000.0 degrees per second per 32768 LSB)
                    kw = 2000.0
                    
                    self.wx = ((wxH << 8) | wxL) / 32768.0 * kw
                    self.wy = ((wyH << 8) | wyL) / 32768.0 * kw
                    self.wz = ((wzH << 8) | wzL) / 32768.0 * kw
                    
                    print(f"Angular velocity output: {self.wx:.3f}, {self.wy:.3f}, {self.wz:.3f} degrees/s")
                
            # Angle output
            if Config.PUBLISH_ANGLE:
                if data[1] == 83:
                    RollL = data[2]
                    RollH = data[3]
                    PitchL = data[4]
                    PitchH = data[5]
                    YawL = data[6]
                    YawH = data[7]
                    
                    self.roll = ((RollH << 8) | RollL) / 32768.0 * 180.0
                    self.pitch = ((PitchH << 8) | PitchL) / 32768.0 * 180.0
                    self.yaw = ((YawH << 8) | YawL) / 32768.0 * 180.0
                    
                    print(f"Angle output: {self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f} degrees")
                
            # Magnetic output
            if Config.PUBLISH_MAGNETIC:
                if data[1] == 84:
                    HxL = data[2]
                    HxH = data[3]
                    HyL = data[4]
                    HyH = data[5]
                    HzL = data[6]
                    HzH = data[7]
                    
                    self.Hx = ((HxH << 8) | HxL)
                    self.Hy = ((HyH << 8) | HyL)
                    self.Hz = ((HzH << 8) | HzL)
                    
                    print(f"Magnetic output: {self.Hx:.1f}, {self.Hy:.1f}, {self.Hz:.1f} uT")
                
            # Data output port status
            if data[1] == 85:
                d0l = data[2]
                d0h = data[3]
                d1l = data[4]
                d1h = data[5]
                d2l = data[6]
                d2h = data[7]
                d3l = data[8]
                d3h = data[9]
                
                self.d0 = ((d0h << 8) | d0l)
                self.d1 = ((d1h << 8) | d1l)
                self.d2 = ((d2h << 8) | d2l)
                self.d3 = ((d3h << 8) | d3l)
                
                Uvcc = 3.3  # Supply voltage (V)
                
                # Digital output port status
                # U = DxStatus / 1024 * Uvcc
                
                # print(f"Data output port status: {self.d0:.1f}, {self.d1:.1f}, {self.d2:.1f}, {self.d3:.1f}")
            
            # Quaternion output
            if Config.PUBLISH_QUATERNION:
                if data[1] == 89:
                    q0l = data[2]
                    q0h = data[3]
                    q1l = data[4]
                    q1h = data[5]
                    q2l = data[6]
                    q2h = data[7]
                    q3l = data[8]
                    q3h = data[9]
                    
                    self.q0 = ((q0h << 8) | q0l) / 32768.0
                    self.q1 = ((q1h << 8) | q1l) / 32768.0
                    self.q2 = ((q2h << 8) | q2l) / 32768.0
                    self.q3 = ((q3h << 8) | q3l) / 32768.0
                    
                    print(f"Quaternion output: {self.q0:.3f}, {self.q1:.3f}, {self.q2:.3f}, {self.q3:.3f}")
                
    def calculate_accelero_offset(self):
        # Gathers 100 samples from the accelerometer and computes offsets
        """
        Calculate accelerometer offsets,
        based on the average of 100 samples.
        Stop the calibration process when done.
        Print the offsets when done.
        
        Returns:
            None
        """
        self.acc_x_values.append(self.acc_x)
        self.acc_y_values.append(self.acc_y)
        self.acc_z_values.append(self.acc_z)
        print("Calibrating accelerometer... Do not move the sensor!")
        
        # Wait for 100 samples
        if len(self.acc_x_values) >= 100:
            avg_acc_x = np.mean(self.acc_x_values)
            avg_acc_y = np.mean(self.acc_y_values)
            avg_acc_z = np.mean(self.acc_z_values)
            
            # Calculate offsets
            acc_x_offset = avg_acc_x
            acc_y_offset = avg_acc_y
            acc_z_offset = avg_acc_z - Config.G
            
            print("Calibration completed!")
            print(f"Acceleration offsets: \n X: {acc_x_offset:.3f}, \n Y: {acc_y_offset:.3f}, \n Z: {acc_z_offset:.3f}")
            return  # Stop calibration
        
def main(args=None):
    rclpy.init(args=args)
    node = IMU_Node() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()