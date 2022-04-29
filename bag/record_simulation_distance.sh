#!/bin/bash

if [ "$1" = "" ]
then
    echo "Please enter your project"
    return 0
else
    rosbag record -O $1.bag /robot/uwb_data_distance
fi

