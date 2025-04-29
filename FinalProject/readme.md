FINAL PROJECT README
_________________________________________
This folder contains all code and files required for the final demo on April 28, 2025.

*PythonCode is the folder that contains the final computer vision algorithm that powers our ability to perceive and understand the environment
    - CustomI2C.py is the underlying code that handles the transmission protocol from I2C between Pi and Arduino. It applies our special encoding mechanism that reduces 
      transmission overhead
    - CustomI2C_driver.py is the driver code that uses the CustomI2C.py code to execute the actual transmission
    - ColorCamRefinedWithThreading.py is our Camera class that encapsulates the operating functionality of the camera. It combines opening a channel, detection of colors, 
      ArucoMarkers, distances and angles
    = Main2.py is the program that utilizes an instance of the Camera class, acquires infromationa and uses our spcial encoding to send it over to the Arduino\
    - The JULIE4_calibration_matrix.yaml is the file that contains camera characteristics and information from our calibration process
