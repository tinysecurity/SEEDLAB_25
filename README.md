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

## File Structure

This respository is broken into folders, which are based on the major project submissions. Each folder has subfolders, which break the code for the submission into languages and use cases. For example, the MiniProject folder has the following breakdown:
```
MiniProject/
├── MATLAB # All MATLAB code, to generate motor response values
│   ├── motor_control.slx
│   ├── motor_control.slxc
│   ├── posStepData.csv
│   ├── RunPIDSimulink.m
│   └── slprj
│       └── ...
├── mp_arduino # All final Arduino code for the project
│   ├── encoder.hpp
│   ├── mp_arduino.ino
│   ├── pid_movement.hpp
│   ├── PIDvars.txt
│   └── position.hpp
├── PythonCode # All final Python code for the project
│   └── QuadrantDetection.py
├── readme.md
├── TestingCode # All unfinished Python code used for testing and development
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
