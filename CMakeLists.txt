cmake_minimum_required(VERSION 2.8.3)
project(camera_robot_calibration)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf geometry_msgs tf_conversions)

catkin_python_setup()
catkin_package()

install(PROGRAMS scripts/camera_robot_calibration.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})