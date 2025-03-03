#q---------------------
#Julie Treesh, Sing Piper
#02/16/2025
#Quadrant  Detection Code for the Raspberry Pi
#This code should detect a marker that is in front of a camera.
#It will then determine which quadrant this marker is in and send that information over to the Arduino
#---------------------

from Camera import Camera
#import time
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import queue
import threading
import yaml
import os

angle = 0

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #dictionary for markers
#camera = cv2.VideoCapture(0) #setting up camera
cam = Camera(0)
sleep(0.5)

q = queue.Queue()

#I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

#assign i2c to the board's ports
i2c = board.I2C()

#Initialize SMBus library with I2C bus 1 for connection to Arduino
i2cArduino = SMBus(1)

#val = "No Markers Found"
quadrant = "No quadrant"

#setting LCD dimensions
lcd_columns = 16
lcd_rows = 2

#setting offset
offset = int(2)

def handleLCD():
    #initialize the LCD class
    #lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns,lcd_rows)

    while (True):
        if not q.empty():
            #lcd.clear()
            gotSomething = q.get()

            #write data to the LCD
            #lcd.clear()
            #lcd.color = [0,100,0]
            if gotSomething == "No markers found":
                lcd.message = gotSomething
            else:
                lcd.message = "Goal Position:" + gotSomething
            sleep(0.0005)
                

myThread = threading.Thread(target=handleLCD,args=())
myThread.start()

while (True):
    
    #ret, image = cam.read() #get images from camera
    cam.updateCoords()
    cam.updateClosestCoords()
    cam.getCoords()
    
    if len(cam.getCoords()) !=0: #if a marker is detected
        centerX = int((cam.getCoords()[0,0] + cam.getCoords()[1,0] + cam.getCoords()[2,0] + cam.getCoords()[3,0])/4) #find average coord for X
        centerY = int((cam.getCoords()[0,1] + cam.getCoords()[1,1] + cam.getCoords()[2,1] + cam.getCoords()[3,1])/4) #find average coord for Y


        #angle detection
        #3D ray projecting center point into 3D space
        realMarkerVec = (np.linalg.inv(cam.cameraMatrix)).dot([centerX,centerY,1.0])
        cameraVec = (np.linalg.inv(cam.cameraMatrix)).dot([320,240,1.0]) #check this
        #A.B = |A||B|cos(angle) --> angle = arccos[(A.B)/(|A||B|)]
        tempAngle = np.rad2deg(np.arccos(realMarkerVec.dot(cameraVec)/(np.linalg.norm(realMarkerVec)*(np.linalg.norm(cameraVec)))))

        if centerX > 320:
            tempAngle = -1*tempAngle

        
        if tempAngle != angle:
            angle = tempAngle
            print(angle)

        
        #figuring out which quadrant the marker is in:
        #checking that center point is in NW
        if centerX <= 320 and centerY <= 240:
            tempQuadrant = "01"
            #write byte to the i2c bus
            #i2cArduino.write_byte_data(ARD_ADDR,offset,1)
        #checking that center point is in NE
        if centerX > 320 and centerY < 240:
            tempQuadrant = "00"
            #write byte to the i2c bus
            #i2cArduino.write_byte_data(ARD_ADDR,offset,0)
        #checking that center point is in SW
        if centerX < 320 and centerY > 240:
            tempQuadrant = "11"
            #write byte to the i2c bus
            #i2cArduino.write_byte_data(ARD_ADDR,offset,3)
        #checking that center point is in SE
        if centerX > 320 and centerY > 240:
            tempQuadrant = "10"
            #write byte to the i2c bus
            #i2cArduino.write_byte_data(ARD_ADDR,offset,2)
            
        #send it to the thread if aruco marker moved:
        if tempQuadrant is not quadrant:
            quadrant = tempQuadrant
            #q.put(quadrant)
            if quadrant == "01":
                #write byte to the i2c bus
                #i2cArduino.write_byte_data(ARD_ADDR,offset,1)
                print(quadrant)
            elif quadrant == "00":
                #write byte to the i2c bus
                #i2cArduino.write_byte_data(ARD_ADDR,offset,0)
                print(quadrant)
            elif quadrant == "11":
                #write byte to the i2c bus
                #i2cArduino.write_byte_data(ARD_ADDR,offset,3)
                print(quadrant)
            elif quadrant == "10":
                #write byte to the i2c bus
                #i2cArduino.write_byte_data(ARD_ADDR,offset,2)
                print(quadrant)

    if len(cam.getCoords()) == 0:
        tempQuadrant = "No markers found"
        if tempQuadrant != quadrant:
            quadrant = tempQuadrant
            print(quadrant)

    if cv2.waitKey(33) == ord('q'): #allows us to close window
        break

cam.camera.release()
cv2.destroyAllWindows()
    
