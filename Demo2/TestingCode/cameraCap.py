import cv2
import numpy as np
import glob
from time import sleep
from time import time
import os
camera = cv2.VideoCapture(0)
os.chdir('/home/seedlab/SEEDLAB_25/Demo2/PythonCode/calibrationImg')
a = 127
print(os.getcwd())
previous = time()
d = 0

while(True):
	current = time()
	d+=current-previous
	previous = current
	ret, image = camera.read()
	#grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	cv2.imshow("Viewport",image)
	if d > 3:
		filename = f'img{a}.jpg'
		a=a+1
		cv2.imwrite(filename,image)
		print("saved!")
		d = 0
	ret2, img = camera.read()
	cv2.imshow("capture",img)
	if cv2.waitKey(33) == ord('q'):
		break
	
camera.release()
cv2.destroyAllWindows()
