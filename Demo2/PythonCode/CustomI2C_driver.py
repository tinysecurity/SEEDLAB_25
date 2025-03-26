from Camera import Camera
import CustomI2C

cam = Camera(1)

while(True):
    cam.update()
    cam.show()
    # 16 is address, 4 is byte precision of floats, 1 is i2c address
    CustomI2C.sendMessage(cam.closestDict, 16, 4, 1)
