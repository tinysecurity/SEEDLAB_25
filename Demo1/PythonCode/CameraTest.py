# CameraTest.py
from Camera import Camera

import cv2
from cv2 import aruco
from time import sleep
import numpy as np

mycam = Camera(0)
# camera = cv2.VideoCapture(0)
while True:
    mycam.updateCoords()
    mycam.updateClosestCoords()
    # mycam.getCoords()
    print(mycam.getCoords())

