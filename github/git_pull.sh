#!/bin/bash

if [ "$1" = "base" ]
then
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
    REPO=master_thesis

    # ------------- vrx -------------- #
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull vrx >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    source ~/$REPO/catkin_ws/src/vrx/github/git_pull.sh master_thesis
    cd ~/$REPO

    # ---------- Pozyx_UWB ----------- #
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull pozyx_uwb >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    source ~/$REPO/catkin_ws/src/pozyx_uwb/github/git_pull.sh master_thesis
    cd ~/$REPO

    # ---------- Turtlebot3 ---------- #
    BRANCH=melodic-devel
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull turtlebot3 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    cd ~/$REPO/catkin_ws/src/Turlebot/turtlebot3
    git checkout $BRANCH
    git pull
    
    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in turtlebot3. Aborting"
        return 1
    fi

    BRANCH=melodic-devel
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull turtlebot3_simulations >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    cd ~/$REPO/catkin_ws/src/Turlebot/turtlebot3_simulations
    git checkout $BRANCH
    git pull
    
    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in turtlebot3_simulations. Aborting"
        return 1
    fi

    # ------------- ARG -------------- #
    BRANCH=main
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull real_to_sim_env >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    cd ~/$REPO/catkin_ws/src/ARG/real_to_sim_env
    git checkout $BRANCH
    git pull
    
    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in real_to_sim_env. Aborting"
        return 1
    fi

    BRANCH=master
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull subt-gazebo >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    cd ~/$REPO/catkin_ws/src/ARG/subt-gazebo
    git checkout $BRANCH
    git pull
    
    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in subt-gazebo. Aborting"
        return 1
    fi

    BRANCH=master
    echo "---------------------------------------------------------------------------------------------------"
    echo "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< pull subt_rl >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "---------------------------------------------------------------------------------------------------"

    cd ~/$REPO/catkin_ws/src/ARG/subt_rl
    git checkout $BRANCH
    git pull
    
    CONFLICTS=$(git ls-files -u | wc -l)
    if [ "$CONFLICTS" -gt 0 ]
    then
        echo "There is conflict in subt_rl. Aborting"
        return 1
    fi


    # ---------- Master_Thesis ---------- #
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