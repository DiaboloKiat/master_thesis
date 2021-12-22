#!/bin/bash

git config --global user.name "DiaboloKiat"
git config --global user.email "DiaboloKiat@gmail.com"

git status
git checkout master
echo "Enter your message"
read message_base

PROJECT=master_thesis
REPO=master_thesis

BRANCH=master
echo "---------------------------------------------------------------------------------------------------"
source ~/$REPO/github/git_branch.sh
pwd
echo "$REPO"
echo "---------------------------------------------------------------------------------------------------"
source ~/$REPO/github/git_pull.sh base
pwd

PULLSTAT=$?
if [ "$PULLSTAT" -gt 0 ]
then
   echo "There is conflict. Aborting"
   cd ~/$REPO
   return
fi
echo "---------------------------------------------------------------------------------------------------"
echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull success >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "---------------------------------------------------------------------------------------------------"

echo "---------------------------------------------------------------------------------------------------"
echo "******************************************* push pozyx_uwb ****************************************"
echo "---------------------------------------------------------------------------------------------------"
source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_push.sh master_thesis
cd ~/$REPO

echo "---------------------------------------------------------------------------------------------------"
echo "********************************************* push vrx ********************************************"
echo "---------------------------------------------------------------------------------------------------"
source ~/$REPO/catkin_ws/src/vrx/github/git_push.sh master_thesis
cd ~/$REPO

echo "----------------------------------------------------------------------------------------------------"
echo "************************************** push project_seadrone ***************************************"
echo "----------------------------------------------------------------------------------------------------"
cd ~/$REPO
git add -A
git commit -m "${message_base} on project_seadrone"
git push


cd ~/$REPO