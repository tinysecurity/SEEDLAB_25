Mini Project


TestingCode folder:

This folder is used to store code we used for testing.


MATLAB folder:

This folder contains all of our MATLAB files. RunPIDSimulink runs the simulink model for our system, which we used to create the PI controller for our robot.


mp_arduino folder:

This folder contains all of our Arduino code for the robot. These files are broken up into hpp header files that we can load into our main mp_arduino.ino file. This runs all of the main Arduino code that will take input from the Pi and convert that into movement on our robot.   


PythonCode folder:

This folder contains our Python code that we run on the Pi. QuadrantDetection.py detects which quadrant a ArUco marker is in, and then sends this information over to the Arduino.
