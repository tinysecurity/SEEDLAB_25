import time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
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
from multiprocessing import Process
import queue
val = 0
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
camera = cv2.VideoCapture(0)
sleep(0.5)
q = queue.Queue()
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns,lcd_rows)
def showLCD():
	while(True):
		if not q.empty():
			val = q.get()
			print(val)
			lcd.message = val
			sleep(0.25)
			lcd.clear()

if __name__=='__main__':

	
	while(True):
		lcd.clear()
        	lcd.color = [0,100,0]
		ret, image = camera.read()
		if not ret:
			print("Cannot receive Frame. Exiting...")
		greyimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cv2.imshow("overlay", greyimage)
		corners,ids,rejected = aruco.detectMarkers(greyimage,aruco_dict)
		overlay = cv2.cvtColor(greyimage,cv2.COLOR_GRAY2RGB)
		overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
		if ids is not None:
			ids = ids.flatten()
			for (outline, id) in zip(corners,ids):
				markerCorners = outline.reshape((4,2))
			overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0),2)

			val =  str(id)
			q.put(val)
			cv2.imshow("overlay",overlay)
			print(val)
			time.sleep(.25)
		else:
			cv2.imshow("overlay",overlay)
			print("No Markers found")
			q.put(-1)
			time.sleep(.25)

		p = Process(target=showLCD)
		p.start()
		p.join()
		if cv2.waitKey(33) == ord('q'):
			break

camera.release()
cv2.destroyAllWindows()
lcd.color = [0,0,0]
lcd.clear()
