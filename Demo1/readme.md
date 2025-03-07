Demo 1

MATLAB Folder:

PythonCode Folder:
    
    Camera.py - This file undistorts the camera, searches for and returns any aruco markers. It will then take the information gotten from the camera calibration to figure out how far away the closest aruco marker is. It also reports the angle in degrees between the camera axis and the marker.
    
    calibration_matrix.yaml - This file holds the data we collected from our camera calibration. This includes the camera matrix, the distance coefficients, rotation vector, and translation vector.
    
    main.py - This code will take information from the camera. It will print out the angle that the marker is at on the LCD screen. It will also print how far away the marker is from the camera.

arduino_tests_and_experiments:

demo1_arduino:
