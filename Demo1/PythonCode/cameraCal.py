#Sing Piper
#This is the beginning of the camera calibration work
# 1st I learn how to cal the camera, pose data, then integrate code
import numpy as np
import cv2 as cv
from cv2 import aruco
import glob

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30,0.001)
camera = cv.VideoCapture(0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
objpoints = [] # real world points 3d
imgpoints = [] # image plane 2d


while(True):
	ret, image = camera.read()
	gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
	cv.imshow("output",gray)
	ret2, corners = cv.findChessboardCorners(gray,(8,8),None)
	if ret2 == True:
		objpoints.append(objp)
		corners2 = cv.cornerSubPix(gray,corners, (11,11),(-1,-1),criteria)

		imgpoints.append(corners2)
		cv.drawChessboardCorners(gray, (8,8), corners2, ret2)
		cv.imshow('img',gray)
	if cv.waitKey(33) == ord('q'): #allows us to close window
		break
cv.destroyAllWindows()
