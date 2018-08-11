# Robot Navigation with ROS

In this repo there is my robot software which allows find its true way to reach goal position. Software works with ROS framework. And used Arduino rotate motors and get sensor values. Jetson TK1 used as a main controller.

## Arduino Files:

	* ROS_Arduino: This folder includes code which works with Arduino 
	* Arduino_Transmitter: This folder includes code which works with tramistter (optional)

## my_robot

	* my_robot_bringup: This folder includes launch file to bringing up the robot
	* my_robot_description: This folder includes urdf files to make robot model and calculate robot state
	* my_robot_navigation: This folder includes navigation files which provides reach goal position using rviz

## This repo needs these packages

	* [hector-slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
	* [rplidar-ros](https://github.com/robopeak/rplidar_ros)

	
