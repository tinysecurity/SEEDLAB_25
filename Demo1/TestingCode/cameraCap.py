import cv2
import numpy as np
import glob
from time import sleep
import os
camera = cv2.VideoCapture(0)
os.chdir('/home/seedlab/SEEDLAB_25/Demo1/PythonCode/calibrationImg')
a = 0
print(os.getcwd())

while(True):
	ret, image = camera.read()
	grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	cv2.imshow("Viewport",grey)
	if cv2.waitKey(1) == ord('s'):
		filename = f'img{a}.jpg'
		a=a+1
		cv2.imwrite(filename,grey)
		print("saved!")
	if cv2.waitKey(33) == ord('q'):
		break
	
camera.release()
cv2.destroyAllWindows()
