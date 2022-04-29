#!/bin/bash

if [ "$1" = "" ]
then
    echo "Please enter your project"
    return 0
else
    rostopic echo -b $1.bag -p /robot/uwb_data_distance > distance_$1.csv
fi


