#!/bin/bash
rm -rf build/ devel/ logs/
catkin_make
source devel/setup.bash