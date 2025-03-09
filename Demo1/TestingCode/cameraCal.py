#Sing Piper
#This is the beginning of the camera calibration work
# 1st I learn how to cal the camera, pose data, then integrate code
import numpy as np
import cv2
from cv2 import aruco
import glob
import os
from time import sleep
import yaml

CHECKERBOARD = (9,7)
MIN_POINTS= 50
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30,0.001)
a = 8
b = 6
objp = np.zeros((b*a,3), np.float32)
objp[0:,:2] = np.mgrid[0:a,0:b].T.reshape(-1,2)
camera = cv2.VideoCapture(0)
objpoints = [] # real world points 3d
imgpoints = [] # image plane 2d
print(os.getcwd())

while(True):
	ret, image = camera.read()
	grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	ret, corners = cv2.findChessboardCorners(grey, (8,6), None)
	cv2.imshow('Viewport',grey)
	
	if ret == True:
		print("True")
		objpoints.append(objp)
		print(objpoints)
		corners2 = cv2.cornerSubPix(grey,corners, (11,11), (-1,-1), criteria)
		imgpoints.append(corners2)
		cv2.drawChessboardCorners(image,(8,6), corners2, ret)
		cv2.imshow('viewport2',image)
		print("Proc")
		cv2.waitKey(1)
	if cv2.waitKey(33) == ord('q'): #allows us to close window
		camera.release()
		break
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1],None,None)
os.chdir('/home/seedlab/SEEDLAB_25/Demo1/TestingCode/calibrationImg')

data = {'camera_matrix': np.asarray(mtx).tolist(),
        'dist_coeff': np.asarray(dist).tolist(), 'rvecs':np.asarray(rvecs).tolist(), 'tvecs':np.asarray(tvecs).tolist()}

# and save it to a file
with open("JULIE2_calibration_matrix.yaml", "w") as f:
    yaml.dump(data, f)
print("Finished making database")

img = cv2.imread('img9.jpg')
cv2.imshow('test',img)
sleep(10)
h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

print("Processing...")
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult2.jpg', dst)
cv2.imshow('unfuck',dst)

sleep(10)
testing = cv2.imread('img8.jpg')
cv2.imshow('random', testing)
print("Successfully unfucked")




