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
import numpy as np
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from CVforPi import *

cam = ComputerVision()

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.color = [0, 0, 0]
lcd.clear()

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus2.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeByteArray(array):
    try:
        bus.write_i2c_block_data(address, len(array), array)
        return 0
    except:
        print("I2C Block TX Error")
        return -1
    
def readByteArray(reg):
    try:
        bus.write_byte(address, reg)
        length = bus.read_byte(address)
        array = []
        for i in range(length):
            array.append(bus.read_byte(address))
        return array
    except:
        print("I2C Block RX Error")
        return -1

def scan():
    beaconData = -1
    while True:
        beaconData = cam.getAngleAndDistance()
        if beaconData != -1:
            return beaconData

while True:
    command = input("Action Menu:\n1 - Demo2.1\n2 - Demo2.2\n3 - Quit")
    if command == "1":
        writeByteArray([1, 0, 0, 0, 0])
        data = scan()
        writeByteArray([2, np.uint8(data[0]), np.uint8(data[0]*256), np.uint8(data[1]), np.uint8(data[0]*256)])
        while bus.readByteArray(255) == 0:
            time.sleep(0.01)
        data = scan()
        writeByteArray([3, np.uint8(data[0]), np.uint8(data[0] * 256), np.uint8(data[1]), np.uint8(data[0] * 256)])
        while bus.readByteArray(255) == 0:
            time.sleep(0.01)

    elif command == "2":
        writeByteArray(1)
        data = scan()

    elif command == "3":
        print("Error: Command Not Recognized")