Demo 2 README

PythonCode
	This is the folder where all code for the machine learning team goes for this demo.
	We start with a base provided by the prior Demo1 and build on it by addding color and shape detection to detect the arrows

	*ColorCamRefinedWithThreading.py is the original object class developed to encapsulate the camera and its main objectives
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

    demo2_arduino folder:
	    This folder contains test code for our I2C communication with the Pi
    
    arduino_tests_and_experiments folder:
        This folder contains the different iteration of our main Arduino code.
        
        
	bignus_free_time: 
	    This folder contains code that we wrote for fun to test our control system for the robot. This code drives the robot in a square so we can test how accurate the driving is after a prolonged period of time. 
	    
	stateMachine:
	    This folder contains the Arduino code for our first iteration of the state machine for our control side of the project.
	
	stateMachineV2:
	    This folder is the second and final iteration of our Arduino code. Our code utilizes a state machine to navigate around its environment. We receive bytes from the Pi and translate these into different states that our code can understand. 
