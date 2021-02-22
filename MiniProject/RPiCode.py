import smbus2
import time
import time
import board
import busio
import threading
import serial
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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
    while(runThread):
        posArray = readByteArray(1)
        pos = posArray[0] + 256 * posArray[1]
        lcd.color = [0,0,100]
        lcd.message = "Counts: " + str(pos) + "       "
        time.sleep(0.001)
        
runThread = False

while True:
    print("Reading Wheel Position...")
    runThread = True
    thread = threading.Thread(target=readPosition)
    thread.start()
    x = input("Press Any Key to Stop")
    print("Done!")
    runThread = False
    break
