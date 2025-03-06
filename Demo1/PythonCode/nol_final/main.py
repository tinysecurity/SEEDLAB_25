from Camera import Camera
import cv2

cam = Camera(0)
while True:
    cam.update()
    cam.show()
