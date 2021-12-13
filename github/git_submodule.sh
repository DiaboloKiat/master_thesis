#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=master_thesis
    HOME=master_thesis
else
    echo "Please enter your project"
    return 0
fi

cd $PROJECT
git submodule init
git submodule update --recursive

echo "------------------update submodule----------------------"