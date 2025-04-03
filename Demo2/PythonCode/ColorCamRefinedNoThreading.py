#Camera
import cv2
from cv2 import aruco
from time import sleep
import os
import yaml
import numpy as np
import math

#read and open YAML file
data = yaml.safe_load(open("JULIE4_calibration_matrix.yaml", "r"))

#set variables from YAML file
cameraMatrix = np.asarray(data['camera_matrix'])
distCoeff = np.asarray(data['dist_coeff'])
rvecs = np.asarray(data['rvecs'])
tvecs = np.asarray(data['tvecs'])



class Camera:
    # class variable initalization
    DOWNSAMPLE = False
    GREYSCALE = False
    SCALE = 0.5
    FLIP_IMAGE = True
    ALPHA = 1
    _DET_PARAMS = aruco.DetectorParameters()
    _DET_PARAMS.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    #_DET_PARAMS.useAruco3Detection = True
    _REF_PARAMS = aruco.RefineParameters()
    DETECTOR = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_6X6_50),_DET_PARAMS,_REF_PARAMS)


    def __init__(self, cameraIndex):
        # camera startup
        self.camera = cv2.VideoCapture(cameraIndex)
        sleep(0.5)
        # initalize variables
        self.arucoDict = []
        self.closestDict = dict()
        #set variables from YAML file
        self.cameraMatrix = np.asarray(data['camera_matrix'])
        self.distCoeff = np.asarray(data['dist_coeff'])
        self.rvecs = np.asarray(data['rvecs'])
        self.tvecs = np.asarray(data['tvecs'])
        # get starting image, find more accurate camera matrix
        self.image = self.camera.read()[1]
        h, w = self.image.shape[:2]
        self.newCameraMatrix = np.asarray(cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeff,(w,h),self.ALPHA,(w,h))[0])
        # get starting vaues
        self.update()


    def read(self):
        # read an image from the camera
        returned, temp_image = self.camera.read()
        # make sure to only update if an image is captured
        if not returned:
            return

        # undistort image
        temp_image = cv2.undistort(temp_image,self.cameraMatrix,self.distCoeff,None,self.newCameraMatrix)

        # image processing
        if self.GREYSCALE:
            temp_image = cv2.cvtColor(temp_image, cv2.COLOR_BGR2GRAY) #change from color to greyscale
        if self.DOWNSAMPLE:
            SCALE = 0.5
            temp_image = cv2.resize(temp_image, (0,0), fx=factor, fy=factor)

        self.image = temp_image
        return temp_image


    def update(self):
        # read and postprocess frame
        temp_image = self.read()
        
        # search for and return any aruco markers
        corners = self.DETECTOR.detectMarkers(temp_image)[0]

        # update all information
        self.arucoDict.clear()
        self.closestDict.clear()
        shortestDistance = -1
        if corners:
            for marker in corners:
                marker = marker[0] # remove extra vector layer
                centerX = int((marker[0].item(0) + marker[1].item(0) + marker[2].item(0) + marker[3].item(0))/4) #find average coord for X
                centerY = int((marker[0].item(1) + marker[1].item(1) + marker[2].item(1) + marker[3].item(1))/4) #find average coord for Y
                a = 5/(2.54*2) # finding the side lengths of aruco marker with origin at center of marker
                objectPoints = np.array([[-a,a,0],[a, a, 0],[a, -a,0],[-a,-a,0]])
                # finding tvec and rvec for each marker
                ret, rvec, tvec = cv2.solvePnP(objectPoints, np.asarray(marker),self.newCameraMatrix, self.distCoeff)
                distance = np.linalg.norm(tvec)
                realMarkerVec = (np.linalg.inv(self.cameraMatrix)).dot([centerX,centerY,1.0])
                cameraVec = (np.linalg.inv(self.cameraMatrix)).dot([320,240,1.0])
                #angle = np.rad2deg(np.arccos(realMarkerVec.dot(cameraVec)/(np.linalg.norm(realMarkerVec)*(np.linalg.norm(cameraVec)))))
                #R,_ = cv2.Rodrigues(rvec)
                #angle = np.rad2deg(math.atan2(-1*R[2][0],math.sqrt(math.pow(R[2][1],2)+math.pow(R[2][2],2))))
                
                #angle = atan((centerX - cx)/fx)
                # using trigonometry to calculate the angle to aruco marker, given x and z components of tvec
                angle = abs(np.rad2deg(math.atan((centerX-self.cameraMatrix[0][2])/cameraMatrix[0][0])))
                
                
                if centerX > 320: # make sure left of camera is positive
                    angle = -1*angle
              
                # define dictionary values
                # corners: 4x2 array of marker corners, i.e. [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
                # distance: "unsigned" scalar distance value
                # angle: signed scalar angle value
                # tvec and rvec: raw values, as returned from solvePnP
                # arrowColor: the color of the arrow that is closest 
                thisDict = {
                    "corners":marker,
                    "distance":distance,
                    "angle":angle,
                    "rvec":rvec,
                    "tvec":tvec,
                    "arrowColor":arrowColor}
                self.arucoDict.append(thisDict)
                # set closestDict based on the closest marker
                if distance < shortestDistance or shortestDistance == -1:
                    shortestDistance = distance
                    self.closestDict = thisDict

            #crop image based on marker location
            cropImage = temp_image[ self.closestDict[0]["corners"][0][1] - 100:self.closestDict[0]["corners"][2][1] + 100, self.closestDict[0]["corners"][0][0] - 100:self.closestDict[0]["corners"][2][0] + 100 ] #coords for the top left corner and bottom right corner
        
            imgHSV = cv2.cvtColor(cropImage,cv2.COLOR_BGR2HSV)
        
            upperGreen = np.array([80, 211, 81])
            lowerGreen = np.array([55, 89, 39])

            mask = cv2.inRange(imgHSV,lowerGreen,upperGreen)
            kernel = np.ones((5,5),np.uint8)
            closing = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
            contoursGreen,_ = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(self.image,contoursGreen,-1,(255,0,0),3)

            greenArrowCenters = np.empty((0,2))
            greenArrowAreas = np.empty((0))

            for index, cnt in enumerate(contoursGreen):
                contour_area = cv2.contourArea(cnt)
                if contour_area > 150:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center = int(x+w/2),int(y+h/2)
                    greenArrowAreas = np.append(greenArrowAreas,contour_area)
                    greenArrowCenters = np.vstack((greenArrowCenters,center))
                    cv2.rectangle(self.image, (x,y), (x+w,y+h), (0,255,0),2)
                    #cv2.putText(self.image, 'Green',(x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                    cv2.putText(self.image, '+', center, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                    
        
            upperRed = np.array([179, 224, 193])
            lowerRed = np.array([140, 180, 75])
            mask2 = cv2.inRange(imgHSV,lowerRed,upperRed)
            kernel2 = np.ones((5,5),np.uint8)
            closing2 = cv2.morphologyEx(mask2,cv2.MORPH_CLOSE,kernel2) 
            contoursRed,_ = cv2.findContours(closing2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #cv2.drawContours(self.image,contoursRed,-1,(255,0,0),3)

            redArrowCenters = np.empty((0,2))
            redArrowAreas = np.empty((0))

            for index, cnt in enumerate(contoursRed):
                contour_area = cv2.contourArea(cnt)
                if contour_area > 150:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center = int(x+w/2),int(y+h/2)
                    redArrowAreas = np.append(redArrowAreas,contour_area)
                    redArrowCenters = np.vstack((redArrowCenters,center))
                    cv2.rectangle(self.image, (x,y), (x+w,y+h), (0,0,255),2)
                    #cv2.putText(self.image, 'Green',(x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                    cv2.putText(self.image, '+', center, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)

            #find the arrow that is associated with the right marker
            if len(greenArrowAreas) == 0 and len(redArrowAreas) == 0:
                #print("No arrows found")
                arrowColor = 1
            elif len(redArrowAreas) == 0:
                #print("Green Arrow Found, No red")
                arrowColor = 2
            elif len(greenArrowAreas) == 0:
                #print("Red Arrow Found, No green")
                arrowColor = 3
            elif np.max(greenArrowAreas) >= np.max(redArrowAreas):
                #print("Green Arrow Closest")
                arrowColor = 2
            elif np.max(greenArrowAreas) < np.max(redArrowAreas):
                #print("Red Arrow Closest")
                arrowColor = 3
            
            #assign arrow color based on detection
            self.closestDict[0]["arrowColor"] = arrowColor

    def show(self):
        # showing image w/ user modifications to frame
        # flip image
        temp_image = self.image
        if self.FLIP_IMAGE:
            temp_image = cv2.flip(temp_image, 1)
        # show image with informative title
        cv2.imshow("liveFrameView", temp_image)
        if self.arucoDict:
            cv2.setWindowTitle('liveFrameView', 'Aruco marker found!')
        else:
            cv2.setWindowTitle('liveFrameView', 'No marker in frame.')
        # wait for q to quit python instance
        if cv2.waitKey(33) == ord('q'):
            quit()
        
