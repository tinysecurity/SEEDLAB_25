# Camera.py
import cv2
from cv2 import aruco

class Camera:
    def __init__(self, cameraIndex):
        camera = cv2.VideoCapture(cameraIndex)
        sleep(0.5)
        corners = []
        closestCorners = []
        DOWNSAMPLE = False
        SHOW_IMAGE = True
        self.updateCoords()

    def read():
        return camera.read()

    def updateCoords:
        returned, image = self.read()
        if not returned:
            return
        lookFor = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # looking for 6 by 6
        corners, ids = aruco.detectMarkers(image, lookFor)[1:2]

    def updateClosestCoords:
        deltaX = 0
        deltaY = 0
        for marker in corners:
            localDeltaX = abs(marker[3].X-marker[1].X)
            if abs(marker[2].X-marker[4].X) > localDeltaX:
                localDeltaX = abs(marker[2].X-marker[4].X)

            localDeltaY = abs(marker[3].Y-marker[1].Y)
            if abs(marker[4].Y-marker[2].Y) > localDeltaY:
                localDeltaY = abs(marker[4].Y-marker[2].Y)

            if localDeltaX > deltaX or localDeltaY > deltaY:
                deltaX = localDeltaX
                deltaY = localDeltaY
                closestCorners = marker

