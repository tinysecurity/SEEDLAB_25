# CameraTest.py
from Camera import Camera

import cv2
from cv2 import aruco
from time import sleep
import numpy as np

mycam = Camera(0)
#camera = cv2.VideoCapture(0)
while True:
    print(mycam.everything())
    if cv2.waitKey(33) == ord('q'):
        break

mycam.camera.release()
cv2.destroyAllWindows()

