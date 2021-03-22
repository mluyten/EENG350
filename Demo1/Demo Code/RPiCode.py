# Group 6 | EENG350 | Demo 1 RasPi Integration Code
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
from FetchAngleforPi import *

cam = ComputerVision()

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Clears the LCD
lcd.color = [0, 0, 0]
lcd.clear()

# This is a angle-reading thread. It runs until runThread is set to false.
# This function reads the angle of the center of an aruco marker to the center plane of the robot and displays it on the LCD. 
# If no aruco is detected, "No Waypoint Detected"
def readPosition():
    angle = -1;
    while(runThread):
        lcd.color = [0,0,100] # Turn LCD on
        angle = -1 * cam.getArucoAngle() # Get angle from CV code
        if angle == -1:
            lcd.message = "No Waypoint     \nDetected        "
        else:
            rad = angle * 3.14159 / 180
            lcd.message = "Deg: " + str(angle) + "         \nRad: " + str(rad) + "       "
        time.sleep(0.01)
        
runThread = False

# Initializes and starts the thread.
print("Searching for Waypoint...") 
lcd.clear()
runThread = True
thread = threading.Thread(target=readPosition)
thread.start()
x = input("Press Any Key to Stop") # Waits for keystoke to stop thread.
print("Done!")
runThread = False

