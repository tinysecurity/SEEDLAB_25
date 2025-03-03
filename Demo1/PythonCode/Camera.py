#Camera.py
import cv2
from cv2 import aruco
from time import sleep
import os
import yaml
import numpy as np

os.chdir('/home/seedlab/SEEDLAB_25/Demo1/PythonCode/calibrationImg') #set OS path to calibrationImg folder
#read and open YAML file from calibrationImg folder
with open("calibration_matrix.yaml", "r") as f:
    data = yaml.safe_load(f)
    
#set variables from YAML file
cameraMatrix = np.asarray(data['camera_matrix'])
distCoeff = np.asarray(data['dist_coeff'])
rvecs = np.asarray(data['rvecs'])
tvecs = np.asarray(data['tvecs'])

class Camera:
    # class variable initalization
    DOWNSAMPLE = False
    SCALE = 0.5
    SHOW_IMAGE = True
    FLIP_IMAGE = True


    def __init__(self, cameraIndex):
        # camera startup
        self.camera = cv2.VideoCapture(cameraIndex)
        sleep(0.5)
        # initalize variables
        self._corners = []
        self._closestCorners = []
        # get starting vaues
        self.updateCoords()
        self.updateClosestCoords()


    def read(self):
        return self.camera.read()


    def updateCoords(self):
        # read an image from the camera
        returned, image = self.read()
        if not returned: # quitting program if we can not find image
            return

        # search for and return any aruco markers
        lookFor = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # looking for 6 by 6
        self._corners = aruco.detectMarkers(image, lookFor)[0]

        if self._corners: #if marker is detected
            #calibrate camera
            h, w = image.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeff,(w,h),1,(w,h))
            #undistort image
            dst = cv2.undistort(image,cameraMatrix,distCoeff,None,newcameramtx)


        # image processing
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #change from color to greyscale
        if self.DOWNSAMPLE:
            SCALE = 0.5
            image = cv2.resize(image, (0,0), fx=factor, fy=factor)

        # showing image w/ user modifications to frame
        if self.SHOW_IMAGE:
            shownImage = image
            if self.FLIP_IMAGE:
                shownImage = cv2.flip(shownImage, 1)
            #adding grid:
            shownImage = cv2.line(shownImage,[320,0],[320,480], (255,0,0),4)
            shownImage = cv2.line(shownImage,[0,240],[640,240], (0,255,0),4)
            cv2.imshow("camera live capture", shownImage)
            cv2.waitKey(1)


    def updateClosestCoords(self):
        # checking if instance variables are valid
        if not self._corners: #if marker is not detected
            self._closestCorners = [] #clear marker coords
            return
        if self._corners == [0]:
            return

        # initalize local comparator variables
        deltaX = 0
        deltaY = 0

        # solve for closest and set to instance variable
        for marker in self._corners:
            marker = marker[0] # remove extra vector layer

            localDeltaX = abs(marker[2].item(0)-marker[0].item(0))
            if abs(marker[1].item(0)-marker[3].item(0)) > localDeltaX:
                localDeltaX = abs(marker[1].item(0)-marker[3].item(0))

            localDeltaY = abs(marker[2].item(1)-marker[0].item(1))
            if abs(marker[3].item(1)-marker[1].item(1)) > localDeltaY:
                localDeltaY = abs(marker[3].item(1)-marker[1].item(1))

            if localDeltaX > deltaX or localDeltaY > deltaY:
                deltaX = localDeltaX
                deltaY = localDeltaY
                self._closestCorners = marker


    def getCoords(self):
        return self._closestCorners


    def everything(self):
        self.updateCoords()
        self.updateClosestCoords()
        return self.getCoords()
