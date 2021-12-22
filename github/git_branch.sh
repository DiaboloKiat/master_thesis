#!/bin/bash

REPO=master_thesis

cd ~/$PROJECT
git checkout master

############################## submodules ####################################

source ~/$REPO/catkin_ws/src/vrx/github/git_branch.sh master_thesis
cd ~/$REPO

source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_branch.sh master_thesis
cd ~/$REPO