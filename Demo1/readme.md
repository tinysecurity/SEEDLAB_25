Demo 1

MATLAB Folder:

    Cphi.slx, Crho.slx, Gphi.slx, Grho.slx - These files contain the G and C functions for our block diagrams respectively. They are broken up into our rho and phi block diagrams, and they were used to find the constraints for our system.
    
    Phi.slx, Rho.slx - These files are the final implementation of our G and C blocks wrapped together with the Completion of our block diagram. These files were used to gather the P and I values that we needed to then add to our Arduino code. 
    
    RunCphi.m, RunCrho.m, RunGphi.m, RunGrho.m, RunPhi.m, RunRho.m - These file are all used to run their respective systems in simulink. They were used to running the slx files and mapping out the resulting data.
    
    phiDotStepResponse.csv, rhoDotStepResponse.csv - These files contain all of the actual data gathered from our robot. We used these to plot them against the MATLAB simulations to fine tune our system. 
    
PythonCode Folder:
    
    Camera.py - This file undistorts the camera, searches for and returns any aruco markers. It will then take the information gotten from the camera calibration to figure out how far away the closest aruco marker is. It also reports the angle in degrees between the camera axis and the marker.
    
    JULIE4_calibration_matrix.yaml - This file holds the data we collected from our camera calibration. This includes the camera matrix, the distance coefficients, rotation vector, and translation vector. It is the final file we used for camera calibration.
    
    main.py - This code will take information from the camera. It will print out the angle that the marker is at on the LCD screen. It will also print how far away the marker is from the camera.

TestingCode
- cameraCal.py is the script we run to update and create the camera calibration YAML file
- calibrationImg is a directory containing the finished/created YAML files.


arduino_tests_and_experiments:
    
    This folder contains all of the test code that we used to get to our final system:
    
    oldPIcontroller -  This contains our old PID controller from the Mini Project. 
    
    phi_rho_inner_loop_controller - This contains the files for when we were experimenting with the inner loop to our controller.
    
    phi_rho_PI_controller - This contains the files for when we started experimenting with a PI controller instead of a PD controller. 
    
    phi_rho_step_response_arduino - This contains the code used to extract the data from our robot to import into MATLAB for plotting. 
    
    position_angle_with_old_PI - This contains the files to get the position and angle from our old PI controller. This was used for testing our implementations. 
    
demo1_arduino:

    demo1_arduino.ino - This is the final implementation of Arduino code used to move our robot. It uses a PI controller to get feedback on the position of the robot and then make adjustments to correct for any error. We are able to set a distance and angle for the robot to travel to. 
    
    encoder.hpp - This header file contains all of the functions that help us read values from the motor encoders.
    
    pi_rho_phi.hpp - this file contains all of the functions and variables needed for our main file.
