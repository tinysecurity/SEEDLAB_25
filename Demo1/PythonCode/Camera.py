#Camera.py
import cv2
from cv2 import aruco
from time import sleep

class Camera:
    def __init__(self, cameraIndex):
        self.camera = cv2.VideoCapture(cameraIndex)
        sleep(0.5)
        self.corners = []
        self.closestCorners = []
        DOWNSAMPLE = False
        SHOW_IMAGE = True
        self.updateCoords()
        self.updateClosestCoords()

    def read(self):
        return self.camera.read()

    def updateCoords(self):
        returned, image = self.read()
        if not returned: #quitting program if we can not find image
            return
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #change from color to greyscale
        #adding grid:
        image = cv2.line(image,[320,0],[320,480], (255,0,0),4)
        image = cv2.line(image,[0,240],[640,240], (0,255,0),4)
        cv2.imshow("Camera Class", image)
        cv2.waitKey(1)
        lookFor = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # looking for 6 by 6
        self.corners = aruco.detectMarkers(image, lookFor)[0]
        #print(self.corners)

    def updateClosestCoords(self):
        deltaX = 0
        deltaY = 0
        if not self.corners: #if marker is not detected
            self.closestCorners = [] #clear marker coords
            return
        if self.corners == [0]:
            return
        # print(self.corners)
        for marker in self.corners:
            marker = marker[0] # extra vector layer
            # print(marker)
            localDeltaX = abs(marker[2].item(0)-marker[0].item(0))
            if abs(marker[1].item(0)-marker[3].item(0)) > localDeltaX:
                localDeltaX = abs(marker[1].item(0)-marker[3].item(0))

            localDeltaY = abs(marker[2].item(1)-marker[0].item(1))
            if abs(marker[3].item(1)-marker[1].item(1)) > localDeltaY:
                localDeltaY = abs(marker[3].item(1)-marker[1].item(1))

            if localDeltaX > deltaX or localDeltaY > deltaY:
                deltaX = localDeltaX
                deltaY = localDeltaY
                self.closestCorners = marker

    def getCoords(self):
        return self.closestCorners

