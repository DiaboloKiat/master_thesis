#!/bin/bash

REPO=master_thesis

cd ~/$PROJECT
git checkout master

############################## submodules ####################################

# ------------- ARG -------------- #
cd ~/$REPO/catkin_ws/src/ARG/real_to_sim_env
git checkout main

cd ~/$REPO/catkin_ws/src/ARG/subt-gazebo
git checkout devel-kiat


# ------------- vrx -------------- #

source ~/$REPO/catkin_ws/src/vrx/github/git_branch.sh master_thesis
cd ~/$REPO

# ---------- Pozyx_UWB ----------- #

source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_branch.sh master_thesis
cd ~/$REPO

# ---------- Turtlebot3 ---------- #

cd ~/$REPO/catkin_ws/src/Turlebot/turtlebot3
git checkout melodic-devel

cd ~/$REPO/catkin_ws/src/Turlebot/turtlebot3_simulations/
git checkout melodic-devel



cd ~/$REPO
