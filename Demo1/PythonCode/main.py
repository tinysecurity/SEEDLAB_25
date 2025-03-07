#------------------------------
#Julie Treesh, Sing Piper, Nolan Pratt
#03/06/2025
#Angle and Distance Detection Code for the Raspberry Pi
#This code should detect a marker and then determine how far away the marker is from the camera
#and at what angle the marker is at.
#It will then send that information over to the Arduino
#-----------------------------

from Camera import Camera
import cv2
from cv2 import aruco
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus
import queue
import threading

#starting and reset values
angle = 10000
lengthOf = 10000
markers = 'start'
angleAndLength = [10000,10000]
tempAngleAndLength = [10000,10000]

#calling Camera class
cam = Camera(0)

#initializing queue
q = queue.Queue()

#I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

#assign i2c to the board's ports
i2c = board.I2C()

#Initialize SMBus library with I2C bus 1 for connection to Arduino
i2cArduino = SMBus(1)

#setting LCD dimensions
lcd_columns = 16
lcd_rows = 2

#setting offset
offset = int(2)

def handleLCD():
    #initialize the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

    while(True):
        if not q.empty():
            lcd.clear()
            gotSomething = q.get() #get next item in queue

            #write data to the LCD
            lcd.clear()
            lcd.color = [0,100,0]
            if gotSomething == "No markers found":
                lcd.message = gotSomething #printing "No markers found" on LCD
            else:
                lcd.message = "Angle: " + str(gotSomething) #printing angle on LCD
            sleep(0.0005)

myThread = threading.Thread(target=handleLCD,args=())
myThread.start()

while True:
    cam.update()
    cam.show()

    if len(cam.arucoDict) != 0: #if a marker is detected
        markers = "Markers Found"

        if ((angle + 0.5 <= cam.arucoDict[0]["angle"]) or (angle - 0.5 >= cam.arucoDict[0]["angle"])):
            angle = round(cam.arucoDict[0]["angle"],2) #updates the angle variable if there was a change in angle
            q.put(angle) #puts angle into queue to be displayed on LCD
            markers = "Markers found"
            tempAngleAndLength[0] = angle

        if ((lengthOf + 0.5 <= cam.arucoDict[0]["distance"]) or (lengthOf - 0.5 >= cam.arucoDict[0]["distance"])):
            lengthOf = round(cam.arucoDict[0]["distance"],2) #updates the distance variable if there was a change in distance
            markers = "Markers found"
            tempAngleAndLength[1] = lengthOf

        if (tempAngleAndLength[0] != angleAndLength[0]) or (tempAngleAndLength[1] != angleAndLength[1]):
            angleAndLength[0] = tempAngleAndLength[0]
            angleAndLength[1] = tempAngleAndLength[1]
            print(angleAndLength)
            #i2cArduino.write_block_data(ARD_ADDR,offset,angleAndLength) #sends angle and distance to Arduino

    if len(cam.arucoDict) == 0: #if no markers are detected
        if markers != "No markers found":
            markers = "No markers found"
            print(markers)
            q.put(markers) #put no marker found message into queue

            #reset values
            angle = 10000
            lengthOf = 10000
            angleAndLength = [10000,10000]
        
