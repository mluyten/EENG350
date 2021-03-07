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
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from CVforPi import *

cam = computer_vision()

quadDict = {'NW': 1, 'NE': 2, 'SW': 3, 'SE': 4, 'NA': 0}

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

def readPosition():
    quad = 'NA'
    while(runThread):
        posArray = readByteArray(255)
        pos = posArray[0] + 256 * posArray[1]
        lcd.color = [0,0,100]
        newQuad = cam.getArucoQuadrant()
        if newQuad != quad:
            quad = newQuad
            writeByteArray([quadDict[quad]])
        lcd.message = "Set: " + str(quadDict[quad]) + "      \nPosition: " + str(pos) + "       "
        time.sleep(0.01)
        
runThread = False

while True:
    print("Reading Wheel Position...")
    lcd.clear()
    runThread = True
    thread = threading.Thread(target=readPosition)
    thread.start()
    x = input("Press Any Key to Stop")
    print("Done!")
    runThread = False
    break
