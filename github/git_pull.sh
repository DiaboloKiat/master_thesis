#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=master_thesis
    REPO=master_thesis

    BRANCH=master
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull master_thesis >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"
    cd ~/$REPO
    git checkout $BRANCH
    git pull

    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in master_thesis. Aborting"
        return 1
    fi
else
    PROJECT=master_thesis
    REPO=master_thesis

    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull vrx >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    source ~/$REPO/catkin_ws/src/vrx/github/git_pull.sh master_thesis
    cd ~/$REPO

    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull pozyx_uwb >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_pull.sh master_thesis
    cd ~/$REPO

    BRANCH=master
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull master_thesis >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"
    cd ~/$REPO
    git checkout $BRANCH
    git pull

    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in master_thesis. Aborting"
        return 1
    fi
fi





cd ~/$REPO
return 0