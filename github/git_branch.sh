#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=master_thesis
    REPO=master_thesis
else
    echo "Please enter your project"
    return 0
fi

cd ~/$PROJECT
git checkout master

############################## submodules ####################################

source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_branch.sh master_thesis
cd ~/$REPO