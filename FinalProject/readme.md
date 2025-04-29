FINAL PROJECT README
_________________________________________
This folder contains all code and files required for the final demo on April 28, 2025.

Computer Vision

* PythonCode is the folder that contains the final computer vision algorithm that powers our ability to perceive and understand the environment
* CustomI2C.py is the underlying code that handles the transmission protocol from I2C between Pi and Arduino. It applies our special encoding mechanism that reduces transmission overhead
* CustomI2C_driver.py is the driver code that uses the CustomI2C.py code to execute the actual transmission
* ColorCamRefinedWithThreading.py is our Camera class that encapsulates the operating functionality of the camera. It combines opening a channel, detection of colors, 
ArucoMarkers, distances and angles
* Main2.py is the program that utilizes an instance of the Camera class, acquires infromationa and uses our spcial encoding to send it over to the Arduino\
* The JULIE4_calibration_matrix.yaml is the file that contains camera characteristics and information from our calibration process

Control Systems
* The 2 main folders you will see in here is arduinoTestsAndExperiments/bingusFreeTime and finalProjectArduino
* bingusFreeTime consists of bingusFreeTime.ino, control_rho_phi.hpp and encoder.hpp
*     The purpose of Bingus Free Time is to reward Bingus for being a good robot and allow him time to roam the space
*     bingusFreeTime.ino takes advantage of the control system we generated for our robot. Its purpose is to choose a random distance and direction and have Bingus just go in that direction. These values are bounded between 2 and 5 feet and 0 and I believe 20 degrees.
*     Its very silly, but we like as a test of our sustem and to just play around becuse we made a silly robot and its fun
* finalProhectArduino is the full and final implementation of the control system. It was written to utilize our own encoder library and the written control system to have smooth and responsive action
*     The finalProjectArduino.ino implements the finite state machine that uses system inputs, outputs and sensor data (aka comptuer vision information piped over via I2C) to allow Bingus to autonomously decide how to react to his environment. 
