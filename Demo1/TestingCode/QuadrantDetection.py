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

angle = 10000
lengthOf = 10000
noMarkers = 'start'
angleAndLength = [10000,10000]
tempAngleAndLength = [10000,10000]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #dictionary for markers
cam = Camera(0)
sleep(0.5)

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
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns,lcd_rows)

    while (True):
        if not q.empty():
            lcd.clear()
            gotSomething = q.get()

            #write data to the LCD
            lcd.clear()
            lcd.color = [0,100,0]
            if gotSomething == "No markers found":
                lcd.message = gotSomething
            else:
                lcd.message = "Angle:" + str(gotSomething)
            sleep(0.0005)
                

myThread = threading.Thread(target=handleLCD,args=())
myThread.start()

while (True):
    
    #ret, image = cam.read() #get images from camera
    cam.updateCoords()
    cam.updateClosestCoords()
    cam.getCoords()
    
    if len(cam.getCoords()) !=0: #if a marker is detected
        noMarkers = "Markers Found"
        centerX = int((cam.getCoords()[0,0] + cam.getCoords()[1,0] + cam.getCoords()[2,0] + cam.getCoords()[3,0])/4) #find average coord for X
        centerY = int((cam.getCoords()[0,1] + cam.getCoords()[1,1] + cam.getCoords()[2,1] + cam.getCoords()[3,1])/4) #find average coord for Y
        a = 5/(2.54*2)
        objectPoints = np.array([[-a,a,0],[a, a, 0],[a, -a,0],[-a,-a,0]])
        
        ret, rvec2, tvec2 = cv2.solvePnP(objectPoints, np.asarray(cam.getCoords()),cam.cameraMatrix, cam.distCoeff)
        #print(f'rvec: {rvec2}')
        #print(f'tvec: {tvec2}')
        
	#angle and distance detection
        #3D ray projecting center point into 3D space
        tempLengthOf = round(np.linalg.norm(tvec2),2)
        #print(f'Length: {lengthOf}')
        realMarkerVec = (np.linalg.inv(cam.cameraMatrix)).dot([centerX,centerY,1.0])
        cameraVec = (np.linalg.inv(cam.cameraMatrix)).dot([320,240,1.0])

        #A.B = |A||B|cos(angle) --> angle = arccos[(A.B)/(|A||B|)]
        tempAngle = round(np.rad2deg(np.arccos(realMarkerVec.dot(cameraVec)/(np.linalg.norm(realMarkerVec)*(np.linalg.norm(cameraVec))))),2)

        if centerX > 320:
            tempAngle = -1*tempAngle

        if ((angle+0.5 <= tempAngle) or (angle-0.5 >= tempAngle)):
            angle = tempAngle
            #print(f'Angle:{angle}')
            q.put(angle)
            noMarkers = "Markers found"
            tempAngleAndLength[0] = angle
            #i2cArduino.write_byte_data(ARD_ADDR,offset,angle)
        
        if ((tempLengthOf >= lengthOf+0.5) or (tempLengthOf <= lengthOf-0.5)):
            lengthOf = tempLengthOf
            #print(f'Length:{lengthOf}')
            noMarkers = "Markers found"
            tempAngleAndLength[1] = lengthOf
            #i2cArduino.write_byte_data(ARD_ADDR,offset,lengthOf)

        if (tempAngleAndLength[0] != angleAndLength[0]) or (tempAngleAndLength[1] != angleAndLength[1]):
            angleAndLength[0] = tempAngleAndLength[0]
            angleAndLength[1] = tempAngleAndLength[1]
            print(angleAndLength)
        
    if len(cam.getCoords()) == 0:
        #print(noMarkers == "No markers found")
        if noMarkers != "No markers found":
            noMarkers = "No markers found"
            print(noMarkers)
            q.put(noMarkers)
            angle = 10000
            lengthOf = 10000
            angleAndLength = [10000,10000]
        

    if cv2.waitKey(33) == ord('q'): #allows us to close window
        break

cam.camera.release()
cv2.destroyAllWindows()
    
