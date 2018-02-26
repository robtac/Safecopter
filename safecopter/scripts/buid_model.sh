#!/bin/bash
set -x #echo on
cd ../urdf
rosrun xacro xacro --inorder -o iris_base.urdf iris_base.xacro enable_wind:=false enable_mavlink_interface:=true enable_ground_truth:=false enable_logging:=false rotors_description_dir:=/home/robbie/catkin_ws/src/safecopter
rm /home/robbie/catkin_ws/src/safecopter/safecopter.sdf
gz sdf -p  iris_base.urdf >> /home/robbie/catkin_ws/src/safecopter/safecopter.sdf

