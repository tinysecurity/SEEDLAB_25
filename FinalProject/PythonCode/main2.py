#------------------------------
#Julie Treesh, Sing Piper, Nolan Pratt
#03/06/2025
#Angle and Distance Detection Code for the Raspberry Pi
#This code should detect a marker and then determine how far away the marker is from the camera
#and at what angle the marker is at.
#It will then send that information over to the Arduino
#-----------------------------
#ColorCam is the name of the copy of the Camera.py script 
#Its the same essential thing, but that way I can make changes without issue

from ColorCamRefinedWithThreading import Camera
import cv2
from cv2 import aruco
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus
import queue
import threading
import CustomI2C

#starting and reset values
angle = 0
lengthOf = 18
markers = 'start'

I2CArray = [10000,10000,10000,10000] #[markerFound, arrowColor, angle, length]
tempI2CArray = [10000,10000,10000,10000] #[markerFound, arrowColor, angle, length]

#calling Camera class
cam = Camera(0)

#initializing queue for LCD screen
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

#def handleCam():
    #while(True):
        #if not q2.empty():
            #gotSomething2 = q2.get() #get next item in queue
            #cam.update()
            #cam.show()

myThread = threading.Thread(target=handleLCD,args=())
myThread.start()

while True:
    cam.update()
    #cam.show()
    #q2.put(1)
    
    if len(cam.closestDict) != 0: #if a marker is detected
        markers = "Markers Found"
        #print("Marker found")
        I2CArray[0] = 1 #set marker found to true
        tempI2CArray[1] = cam.closestDict["arrowColor"]

        if ((angle + 0.5 <= cam.closestDict["angle"]) or (angle - 0.5 >= cam.closestDict["angle"])):
            angle = round(cam.closestDict["angle"],2) #updates the angle variable if there was a change in angle
            q.put(angle) #puts angle into queue to be displayed on LCD
            markers = "Markers found"
            tempI2CArray[2] = angle

        if ((lengthOf + 0.5 <= cam.closestDict["distance"]) or (lengthOf - 0.5 >= cam.closestDict["distance"])):
            lengthOf = round(cam.closestDict["distance"],2) #updates the distance variable if there was a change in distance
            markers = "Markers found"
            tempI2CArray[3] = lengthOf

        if (tempI2CArray[2] != I2CArray[2]) or (tempI2CArray[3] != I2CArray[3]) or (tempI2CArray[1] != I2CArray[1]):
            I2CArray[2] = tempI2CArray[2]
            I2CArray[3] = tempI2CArray[3]
            I2CArray[1] = tempI2CArray[1]
            print(I2CArray)
            #i2cArduino.write_block_data(ARD_ADDR,offset,I2CArray) #sends angle and distance to Arduino
            CustomI2C.sendMessage(cam.closestDict, 8, 4, 1)

    if len(cam.closestDict) == 0: #if no markers are detected
        if markers != "No markers found":
            markers = "No markers found"
            q.put(markers) #put no marker found message into queue

            #reset values
            angle = 0
            lengthOf = 18
            I2CArray = [0,1,0,18]
            print(I2CArray)
            #i2cArduino.write_block_data(ARD_ADDR,offset,I2CArray) #sends angle and distance to Arduino
            CustomI2C.sendMessage(cam.closestDict, 8, 4, 1)
