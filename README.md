# SEED Lab Repository, Team 13

This is our central repository for our code for EENG 350, Section B.

### Group members:
Jake Stanley, [jtstanley@mines.edu](mailto:jtstanley@mines.edu)\
Liam Abell, [labell@mines.edu](mailto:labell@mines.edu)\
Sing Piper, [singpiper@mines.edu](mailto:singpiper@mines.edu)\
Julie Treesh, [jtreesh@mines.edu](mailto:jtreesh@mines.edu)\
Nolan Pratt, [nolanpratt@mines.edu](mailto:nolanpratt@mines.edu)

## Table of Contents

* [File Structure](#file-structure)
* [Usage](#usage)

## File Structure

This respository is broken into folders, which are based on the major project submissions. Each folder has subfolders, which break the code for the submission into languages and use cases. For example, the MiniProject folder has the following breakdown:
```
MiniProject/
├── MATLAB -- All MATLAB code, to generate motor response values
│   ├── motor_control.slx
│   ├── motor_control.slxc
│   ├── posStepData.csv
│   ├── RunPIDSimulink.m
│   └── slprj
│       └── ...
├── mp_arduino -- All final Arduino code for the project
│   ├── encoder.hpp
│   ├── mp_arduino.ino
│   ├── pid_movement.hpp
│   ├── PIDvars.txt
│   └── position.hpp
├── PythonCode -- All final Python code for the project
│   └── QuadrantDetection.py
├── readme.md
├── TestingCode -- All unfinished Python code used for testing and development
│   ├── arucoDetectFAST.py
│   ├── arucoDetect.ino
│   ├── arucoDetectOptimized.py
│   ├── arucoDetect.py
│   ├── arucoDetectthreads.py
│   ├── arucoDetectthreads.py.save
│   ├── lcdtest2.py
│   ├── lcdtest.py
│   └── runSerial.py
└── test.py
```

## Usage

### Raspberry Pi
Clone the entire repository onto the Pi, and run the Python file corresponding to the project that you want to demo for. For example, for the MiniProject folder, run the QuadrantDetection.py file. Generally, the only file you can't run directly is Camera.py, which is a library file we created to manage calculations on the image frames.

### Arduino
Clone the entire repository onto a computer that is connected to the Arduino. Then, use the Arduino IDE to compile the project from the corresponding project folder onto the Arduino. Some library files for the main .ino files are stored as .hpp files. This allows us to manage the order in which the libraries are loaded, which is seemingly not possible with only Arduino IDE tabs.
