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

def writeByte(value, reg):
    try:
        bus.write_byte_data(address, reg, value)
        return 0
    except:
        print("I2C Byte TX Error")
    return -1

def readByte(reg):
    try:
        bus.write_byte(address, reg)
        time.sleep(0.1)
        number = bus.read_byte(address)
        return number
    except:
        print("I2C Byte RX Error")
        return -1

def writeByteArray(array):
    try:
        bus.write_i2c_block_data(address, len(array), array)
        return 0
    except:
        print("I2C Block TX Error")
        return -1
    
def readByteArray():
    try:
        length = bus.read_byte(address)
        array = []
        for i in range(length):
            array.append(bus.read_byte(address))
        return array
    except:
        print("I2C Block RX Error")
        return -1

def readPot():
    while(runThread):
        voltage = readByte(255) * 4
        voltage = voltage / 1024 * 5
        lcd.color = [0,0,100]
        lcd.message = "Voltage: " + str(voltage) + "       "
        time.sleep(0.001)
        
def writeByteSerial(ser, value):
    try:
        ser.write(bytes([int(value)]))
        return 0
    except:
        print("Serial TX Error")
        return -1
        
def readByteSerial(ser):
    try:
        if ser.inWaiting() > 0:
            num = ser.read()
            return int.from_bytes(num, "big")
        else:
            print("Nothing in buffer")
    except:
        print("Serial RX Error")
        return -1
    
runThread = False


while True:
    print("1 - Byte->Arduino->Byte+5")
    print("2 - ^ + User Defines Arduino Addition")
    print("3 - ^ + LCD Screen")
    print("4 - String->Arduino->String")
    print("5 - Dsplay Potentiometer Voltage on LCD")
    print("6 - 1 Over Serial")
    select = input("Select a Test: ")
    lcd.color = [0, 0, 0]
    lcd.clear()
    
    if int(select) == 1:
        var = input("Enter 1 – 9: ")
        writeByte(int(var), 0)
        print("Sent:", var)
        time.sleep(0.25)
        num = readByte(0)
        print("Received:", num)
        
    elif int(select) == 2:
        var = input("Enter 1 – 9: ")
        reg = input("Input a Register Address: ")
        writeByte(int(var), int(reg))
        print("Sent:", var)
        time.sleep(0.25)
        num = readByte(int(reg))
        print("Received:", num)
        
    elif int(select) == 3:
        var = input("Enter 1 – 9: ")
        reg = input("Input a Register Address: ")
        lcd.color = [0,0,100]
        writeByte(int(var), int(reg))
        print("Sent:", var)
        time.sleep(0.25)
        num = readByte(int(reg))
        print("Received:", num)
        lcd.message = "sent: " + str(var) + "\ngot: " + str(num)

    elif int(select) == 4:
        var = input("Input a String to Send (Max 32 Characters): ")
        sendArray = []
        lcd.color = [0,0,100]
        for letter in var:
            sendArray.append(ord(letter))
        writeByteArray(sendArray)
        print("Sent:", var)
        time.sleep(0.25)
        string = "".join([chr(value) for value in readByteArray()])
        print("Received:", string)
        lcd.message = "sent: " + var + "\ngot: " + string
        
    elif int(select) == 5:
        runThread = True
        potThread = threading.Thread(target=readPot)
        potThread.start()
        var = input("Press Enter to Stop")
        runThread = False
        
    elif int(select) == 6:
        with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
            ser.flushInput()
            var = input("Enter 1 – 9: ")
            writeByteSerial(ser, int(var))
            print("Sent:", var)
            time.sleep(0.25)
            num = readByteSerial(ser)
            print("Received:", num)
            ser.close()
        
    else:
        print("Invalid Input: Enter 1-6")
        
    print("")
