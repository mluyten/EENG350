# Group 6 | EENG350 | Mini Project RasPi Integration Code
# Purpose: Runs CV program and tells the Arduino the desired angle. 
#          Updates the desired and current position of the wheel on the LCD display.

import smbus2
import time
import time
import board
import busio
import threading
import serial
import struct
import math
import numpy as np
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from ComputerVision import *

cam = ComputerVision()
cam.startCapture()
time.sleep(2)

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)

bus = smbus2.SMBus(1)

ser = serial.Serial('/dev/ttyACM0', 115200)

def scan():
    beaconData = -1
    while not cam.arucoExist():
        pass
    return
        
def navigate():
    beaconData = -1
    beaconData = cam.getAngleAndDistance()
    if beaconData != -1:
        return beaconData
    
def getTangentCommand(data):
    radius = 14
    b = 0.658 * abs(data[0])
    a = abs(data[1])
    theta = math.atan(a/b) * data[2]
    d = np.sqrt(np.power(a, 2) + np.power(b, 2))
    if radius > d:
        return reverseAndTryAgain
    driveDistance = np.sqrt(np.power(d, 2) - np.power(radius, 2))
    direction = None
    
    if theta > 0:
        angle = (math.atan(radius/driveDistance) + theta) * 180 / 3.14159
        direction = 4
            
    elif theta < 0:
        if a < radius:
            angle = (math.atan(radius/driveDistance) - abs(theta)) * 180 / 3.14159
            direction = 4
            print("LEFT")
        elif a > radius:
            angle = (abs(theta) - math.atan(radius/driveDistance)) * 180 / 3.14159
            direction = 3
        
    print(theta * 180 / 3.14159, angle, driveDistance, a, b, direction)
            
    return [struct.pack('B', direction), struct.pack('B', np.uint8(driveDistance)),
                     struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(angle)),
                     struct.pack('B', np.uint8(angle * 256)), struct.pack('B', 0)]

#try:
radius = 12
while True:
    command = input("Action Menu:\n1 - Final Demo\n2 - Quit\n")
    if command == "1":        
        ser.write(struct.pack('B', 1))
        scan()
        ser.write(struct.pack('B', 0))
        count = 1
        
        while count < 3:
            data = navigate()
            while data == -1:
                data = navigate()
            sendArray = getTangentCommand(data)
            for byte in sendArray:
                ser.write(byte)
            ser.reset_input_buffer()
            while ser.in_waiting == 0:
                pass
            ser.reset_input_buffer()
                
            ser.write(struct.pack('B', 2))
            scan()
            ser.write(struct.pack('B', 0))
            
            count += 1
            

    elif command == "2":
        break

    else:
        print("Error: Command Not Recognized")
#except:
    #ser.close()
    #print("Done! Closed Serial")