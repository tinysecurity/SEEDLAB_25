Demo 2 README

PythonCode
	This is the folder where all code for the machine learning team goes for this demo.
	We start with a base provided by the prior Demo1 and build on it by addding color and shape detection to detect the arrows

	*Camera.py is the original object class developed to encapsulate the camera and its main objectives
 		Detect Aruco Marker
   		Detect Color
     		Format data for transmission over i2c
       		Create data array contain distance, angle, color and arrow presence
       *CustomI2C.py is a functional class with the purpose of creating an encoding mechanism to minimze bytes necessary for transmission over I2C
                Fundamentally, this subprocess masks the values given, changes from floats to unsigned integers, which can be encoded in such a way as to maintain ful precision in the final measurement before transmission and decodes the data after transmission
	*CustomI2C.py is the implementation or application of the custom transmission protocol object
 	*main.py - main is the primary driver code that implements both the Camera object and the CustomI2C encoding. 
  	We implemented threading to optimize for speed and responsiveness
   

arduinoCode
	This is the folder where all code for the functional operation of the arduino will be stored.
	As files are added and modified, we will add detail here as to what is updated.

	ADD README DETAILS
	
	ADD README DETAILS
