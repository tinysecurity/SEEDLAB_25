from time import sleep
import numpy as np
import cv2

#------takes a picture------
#initialize the camera.
camera = cv2.VideoCapture(0)

#Let the camera warmup
sleep(1)

ret, image = camera.read() #gets an image from the camera stream

if not ret:
    print("Could not capture image from camera!")
    quit()
else:
    print("Origianl Image Saved as 'OriginalImage.png'.")
    try:
        cv2.imwrite('OriginalImage.png',image)
    except:
        print("Could not save original image.")
        pass
        quit()

#Loading in the image that has been saved:
img = cv2.imread('OriginalImage.png')

#------detects the red shape------
#Converting the image to HSV
imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#creating a range of red that we might see
upperRed = np.array([179, 224, 193])
lowerRed = np.array([140, 180, 75])
#upperRed = np.array([255,160,160])
#lowerRed = np.array([150,200,200])
#[84, 255, 255]
#[60, 160, 90]

#apply mask to a new array
mask = cv2.inRange(imgHSV,lowerRed,upperRed)
#result = cv2.bitwise_and(img,img,mask=mask)


#------clean up the mask with morphological transformations------
kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel) #dilation followed by erosion to clean up shape

#------display the detected shape on the original image using a contour------
contourRedVisualize = img.copy()
contoursRed,_ = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(contourRedVisualize,contoursRed,-1,(255,0,0),3)
cv2.imshow("Contours",contourRedVisualize)
cv2.waitKey(0)
cv2.destroyAllWindows()
