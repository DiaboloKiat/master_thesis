#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=master_thesis
    REPO=master_thesis
else
    echo "Please enter your project"
    return 0
fi

BRANCH=master
echo "---------------------------------------------------------------------------------------------------"
echo "------------------------------------pull master_thesis---------------------------------------------"
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



cd ~/$REPO
return 0