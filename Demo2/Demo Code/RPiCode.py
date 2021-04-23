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

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#lcd.color = [0, 0, 0]
#lcd.clear()

# for RPI version 1, use “bus = smbus.SMBus(0)”
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

try:
    while True:
        command = input("Action Menu:\n1 - Demo2.1\n2 - Demo2.2\n3 - Quit\n")
        if command == "1":
            ser.write(struct.pack('B', 1))
            scan()
            ser.write(struct.pack('B', 0))
            data = navigate()
            print("Calc")
            while data == -1:
                data = navigate()
            print("Calc")
            driveDistance = np.sqrt(np.power(data[0], 2) + np.power(data[1], 2)) - 10
            print("Calc")
            if data[2] > 0:
                sendArray = [struct.pack('B', 2), struct.pack('B', np.uint8(driveDistance)),
                             struct.pack('B', np.uint8(driveDistance)), struct.pack('B', np.uint8(abs(data[2]))),
                             struct.pack('B', np.uint8(abs(data[2])*256)), struct.pack('B', 0)]
            else:
                sendArray = [struct.pack('B', 3), struct.pack('B', np.uint8(driveDistance)),
                         struct.pack('B', np.uint8(driveDistance)), struct.pack('B', np.uint8(abs(data[2]))),
                         struct.pack('B', np.uint8(abs(data[2]*256))), struct.pack('B', 0)]
            for byte in sendArray:
                ser.write(byte)
            print("Sent")

        elif command == "2":
            ser.write(struct.pack('B', 1))
            scan()
            ser.write(struct.pack('B', 0))
            data = navigate()
            while data == -1:
                data = navigate()
            opposite = 13 - abs(data[1])
            angle = abs(math.atan(abs(opposite) / abs(data[0]))) + 0.08
            driveDistance = np.sqrt(np.power(abs(data[0]), 2) + np.power(abs(opposite), 2)) - 4
            #if (data[2] > 0 and opposite > 0) or (data[2] < 0 and opposite < 0):
                #sendArray = [struct.pack('B', 4), struct.pack('B', np.uint8(driveDistance)),
                             #struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(angle))),
                             #struct.pack('B', np.uint8(abs(angle) * 256)), struct.pack('B', 0)]
            #else:
                #sendArray = [struct.pack('B', 5), struct.pack('B', np.uint8(driveDistance)),
                         #struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(angle))),
                         #struct.pack('B', np.uint8(abs(angle) * 256)), struct.pack('B', 0)]
            
            if data[2] > 0 and opposite > 0:
                sendArray = [struct.pack('B', 5), struct.pack('B', np.uint8(driveDistance)),
                             struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(angle))),
                             struct.pack('B', np.uint8(abs(angle) * 256)), struct.pack('B', 0)]
                print(angle*180/3.14159)
                print(driveDistance)
                print(1)
            
            elif data[2] > 0 and opposite < 0:
                sendArray = [struct.pack('B', 4), struct.pack('B', np.uint8(driveDistance)),
                         struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(angle))),
                         struct.pack('B', np.uint8(abs(angle) * 256)), struct.pack('B', 0)]
                print(angle*180/3.14159)
                print(driveDistance)
                print(2)
            elif data[2] < 0 and opposite > 0:
                sendArray = [struct.pack('B', 5), struct.pack('B', np.uint8(driveDistance)),
                         struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(data[2] * 2) + abs(angle))),
                         struct.pack('B', np.uint8((abs(data[2] * 2) + abs(angle)) * 256)), struct.pack('B', 0)]
                print((abs(data[2] * 2) + abs(angle))*180/3.14159)
                print(driveDistance)
                print(3)
            elif data[2] < 0 and opposite < 0:
                sendArray = [struct.pack('B', 5), struct.pack('B', np.uint8(driveDistance)),
                         struct.pack('B', np.uint8(driveDistance * 256)), struct.pack('B', np.uint8(abs(data[2] * 2) + abs(angle))),
                         struct.pack('B', np.uint8((abs(data[2] * 2) + abs(angle)) * 256)), struct.pack('B', 0)]
                print((abs(data[2] * 2) + abs(angle))*180/3.14159)
                print(driveDistance)
                print(4)
                
            if data[2] > 0:
                sendArray[5] = struct.pack('B', 1)
            else:
                sendArray[5] = struct.pack('B', 0)
                
            for byte in sendArray:
                ser.write(byte)

        elif command == "3":
            print("Error: Command Not Recognized")
except:
    ser.close()
    print("Done! Closed Serial")