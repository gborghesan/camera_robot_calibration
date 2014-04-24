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

##### Usage of the test
```
./camera_robot_calibration_module [-h/--help][-i/--inputfile file_of_input][-p/--plotgraph True/False]
```

Options:

```
-h: shows help
-i: name file containing the data (if not provided, 
    random poses for robot and camera-marker will be generated, with noise)
-p: plot the graph of max and average residuals (defaults to True)
```

## Rosnode description

A video showing the employment of the node (calibration of a kuka and a asus xion using ar_kinect marker tracking) is available here:
http://youtu.be/ihWAxj-8IWM

#####  parameters

- _base_frame_name_ : name of the common frame
- _camera_frame_name_ : Name of frame in which is represented the marker
 	(attached to the 3d measurement system), e.g. _/camera_link_
- _robot_ee_frame_name_ : name of the frame attached to the robot 
	link that supports the marker, e.g.  _/lwr_arm_link_7_
- _target_frame_name_ : name of the frame that is measured by the measurement system, 
	e.g.  _/marker_frame_ 
- _nominal_pose_camera_ : initial guess of the pose between base_frame 
	and camera_frame (defaults to identity)
- _robot_ee_marker_ : initial guess of the position between robot 
e.e. and the marker (defaults to identity, only the position matters, as orientation is never used)

All these frame are expected to be published as tf

##### Operations

- _read_tfs_ : store internally the measurement (the marker w.r.t. camera and e.e. w.r.t. base).
- _reset_frames_ : cancel all measurements
- _compute_frames_ compute the camera pose. can be called more than once.

##### Tf transforms

listens to 
- _base_frame_name_ -> _robot_ee_frame_name_
- _camera_frame_name_ -> _target_frame_name_

sends
- _base_frame_name_ -> _camera_frame_name_
- _robot_ee_frame_name_ -> _target_frame_name_



