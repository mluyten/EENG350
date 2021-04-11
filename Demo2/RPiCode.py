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
            print("Found")
            ser.write(struct.pack('B', 0))
            print("Nav")
            data = navigate()
            print("Nav Done")
            while data == -1:
                data = navigate()
            driveDistance = np.sqrt(np.power(data[0], 2) + np.power(data[1], 2)) - 6
            
            sendArray = [struct.pack('B', 2), struct.pack('B', np.uint8(driveDistance)),
                         struct.pack('B', np.uint8(driveDistance)), struct.pack('B', np.uint8(data[2])),
                         struct.pack('B', np.uint8(data[2]*256))]
            for byte in sendArray:
                ser.write(byte)
            while ser.in_waiting == 0:
                pass
            ser.read()

        elif command == "2":
            ser.write(struct.pack('B', 1))
            scan()
            print("Found")
            ser.write(struct.pack('B', 0))
            data = navigate()

        elif command == "3":
            print("Error: Command Not Recognized")
except:
    ser.close()
    print("Done! Closed Serial")
