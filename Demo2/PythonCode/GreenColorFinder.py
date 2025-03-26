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

#------detects the green shape------
#Converting the image to HSV
imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#creating a range of green that we might see
upperGreen = np.array([86, 211, 81])
lowerGreen = np.array([60, 89, 39])

#[96, 211, 81]
#[60, 89, 39]

#apply mask to a new array
mask = cv2.inRange(imgHSV,lowerGreen,upperGreen)
#result = cv2.bitwise_and(img,img,mask=mask)


#------clean up the mask with morphological transformations------
kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel) #dilation followed by erosion to clean up shape

#------display the detected shape on the original image using a contour------
contourGreenVisualize = img.copy()
contoursGreen,_ = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(contourGreenVisualize,contoursGreen,-1,(255,0,0),3)
cv2.imshow("Contours",contourGreenVisualize)
cv2.waitKey(0)
cv2.destroyAllWindows()
