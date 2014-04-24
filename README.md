Camera-Robot Calibration
========================

This package offers a rosnode that, given a set of pose measurements 
of the robot end effector w.r.t. a base frame, and the position of 
a point attached to the robot e.e., measured in a other frame 
(e.g. a camera measuring a marker position w.r.t. the camera frame) 
it computes the transformation of from the base frame to the camera frame 
(and the position of the marker w.r.t the end effector, as a collateral effect).

The package contains two python files:
- a rosnode 
- a python module

The python module is self standing and can be tested either with saves data 
(the data.txt brings an example) or with randomly generated data.
usage
```
camera_robot_calibration_module [-h/--help][-i/ --inputfile file_of_input][-p/--plotgraph True/False]
```

```
-h: shows help
-i: name file containing the data (in not provided, random poses for robot and camera-marker will be generated, with noise)
-p: plot the graph of max and average residuals (defaults to True)
```



