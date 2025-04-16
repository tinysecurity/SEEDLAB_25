from Camera import Camera
import CustomI2C

cam = Camera(0)

while(True):
    cam.update()
    cam.show()
    # 16 is address, 4 is byte precision of floats, 1 is bus index
    CustomI2C.sendMessage(cam.closestDict, 8, 4, 1)
