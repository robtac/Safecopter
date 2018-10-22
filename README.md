
# Safecopter: The Quadcopter that Never Crashes

Safecopter uses an array of infrared time-of-flight 3D cameras to detect, map, and avoid objects around it. This project was built utilizing the [ROS (Robot Operating System)](http://www.ros.org) framework. It was programmed with a combination of C++ and Python. 

For more information about this project, check out my website at www.rgtac.com.

## Structure

The repository is a clone of my catkin_ws directory used in the development of the project. It does not contain the requirements/libraries it is built with. 

### Safecopter

The Safecopter folder holds the main project code. Within there, the src folder contains the C++ that does the processing on the data received from the 3D camera point clouds. The scripts folder contains Python files that tells the PixHawk software where to fly towards. 

### pico_flexx_driver

The pico_flexx_driver folder is a modified version of [this ROS pico flexx driver](https://github.com/code-iai/pico_flexx_driver). I used this modified version because there was a bug in the original version that didn't work with multiple cameras attached. My solution utilized a mutex lock to prevent multiple instances of the driver requesting data from the same camera. 
