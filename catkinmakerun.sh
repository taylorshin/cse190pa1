#!/bin/bash
cd ../../..
catkin_make
cd src/CSE_190_PA1/scripts
roslaunch cse_190_assi_1 solution_python.launch
