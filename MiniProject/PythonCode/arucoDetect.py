#import require depenencies
#Time is for how long to do stuff
#board allows us to directly interface with Pi pins
#character LCD lets us control the screen
#numpy is useful for array operations
#cv2 and its dependencies allow us to use the camera
import time
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
lcd_columns = 16
lcd_rows = 2

#assign i2c to the board's ports
i2c = board.I2C()

lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcd_columns,lcd_rows)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
camera = cv2.VideoCapture(0)
#start the camera and declare which aruco detection dictionary we will use

sleep(0.5)
#this is essentially the "run loop" of the python code where we can run
#the detection algo
while(True):
	lcd.clear()
	lcd.color = [0,100,0]
	#fun colors
	ret, image = camera.read()
	#get images from camera
	greyimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#change from color to greyscale
	cv2.imshow("overlay", greyimage)
	#this is where we are doing the marker detection VVVVVV
	corners,ids,rejected = aruco.detectMarkers(greyimage,aruco_dict)
	overlay = cv2.cvtColor(greyimage,cv2.COLOR_GRAY2RGB)
	#here is where we outline it on the GUI so we can see where
	overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
	if ids is not None:
		ids = ids.flatten()
		for (outline, id) in zip(corners,ids):
			markerCorners = outline.reshape((4,2))
			overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
		val = str(id)
		cv2.imshow("overlay",overlay)
		lcd.message = "ID is "+val
		time.sleep(.25)
		#this code essentially shows the value from the detection
	else:
		cv2.imshow("overlay",overlay)
		lcd.message = "No Markers found"
		time.sleep(.25)
	if cv2.waitKey(33) == ord('q'):
		break
camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()
#^^^^^ turns off LCD panel and clears any content
#fileName = input("File name")
#if not ret:
#	print("Could not capture image from camera!")
#	quit()

#else:
#	print("Saving image"+ fileName)
#	try:
#		cv2.imwrite(fileName,greyimage)
#	except:
#		print("Could not save "+fileName)
#		pass

