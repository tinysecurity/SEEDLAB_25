#---------------------
#Julie Treesh, Sing Piper
#02/16/2025
#Quadrant  Detection Code for the Raspberry Pi
#This code should detect a marker that is in front of a camera.
#It will then determine which quadrant this marker is in and send that information over to the Arduino
#---------------------

import time
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import queue
import threading

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #dictionary for markers
camera = cv2.VideoCapture(0) #setting up camera
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
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns,lcd_rows)

    while (True):
        if not q.empty():
            lcd.clear()
            gotSomething = q.get()
            #print("I got: {}".format(gotSomething))

            #write data to the LCD
            lcd.clear()
            lcd.color = [0,100,0]
            if gotSomething == "No markers found":
                lcd.message = gotSomething
            else:
                lcd.message = "Goal Position:" + gotSomething
            sleep(0.0005)
                

myThread = threading.Thread(target=handleLCD,args=())
myThread.start()

while (True):
    
    ret, image = camera.read() #get images from camera
    if not ret:
        print("Cannot receive Frame. Exiting...")
        quit() #quitting program if we can not find image
        break
    greyimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #change from color to greyscale
    cv2.imshow("overlay", greyimage) #showing grey image

    #---------marker detection-------------
    corners,ids,rejected = aruco.detectMarkers(greyimage,aruco_dict)
    overlay = cv2.cvtColor(greyimage,cv2.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4) #outline on GUI
    if ids is not None:
        ids = ids.flatten()
        for (outline,id) in zip(corners,ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0),2)

            centerX = int((markerCorners[0,0] + markerCorners[1,0] + markerCorners[2,0] + markerCorners[3,0])/4) #find average coord for X
            centerY = int((markerCorners[0,1] + markerCorners[1,1] + markerCorners[2,1] + markerCorners[3,1])/4) #find average coord for Y

            #figuring out which quadrant the marker is in:
	    #checking that center point is in NW
            if centerX <= 320 and centerY <= 240:
                tempQuadrant = "01"
                #write byte to the i2c bus
                i2cArduino.write_byte_data(ARD_ADDR,offset,1)
	    #checking that center point is in NE
            if centerX > 320 and centerY < 240:
                tempQuadrant = "00"
		#write byte to the i2c bus
                i2cArduino.write_byte_data(ARD_ADDR,offset,0)
	    #checking that center point is in SW
            if centerX < 320 and centerY > 240:
                tempQuadrant = "11"
		#write byte to the i2c bus
                i2cArduino.write_byte_data(ARD_ADDR,offset,3)
	    #checking that center point is in SE
            if centerX > 320 and centerY > 240:
                tempQuadrant = "10"
		#write byte to the i2c bus
                i2cArduino.write_byte_data(ARD_ADDR,offset,2)
            
        #send it to the thread if aruco marker moved:
        if tempQuadrant is not quadrant:
            quadrant = tempQuadrant
            q.put(quadrant)

        cv2.imshow("overlay",overlay)
        time.sleep(.0005)
    else:
        cv2.imshow("overlay",overlay)
        if quadrant != "No markers found":
            quadrant = "No markers found"
            q.put(quadrant)
        time.sleep(.0005)

    if cv2.waitKey(33) == ord('q'): #allows us to close window
        break

camera.release()
cv2.destroyAllWindows()
    
