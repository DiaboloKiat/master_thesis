#!/bin/bash

if [ "$1" = "" ]
then
    echo "Please enter your project"
    return 0
else
    rosbag record -O $1.bag /robot/odometry /robot/odometry_tf /robot/posestamped /robot/posestamped_tf /robot/localization_data_topic /robot/joy_teleop/cmd_vel
fi

