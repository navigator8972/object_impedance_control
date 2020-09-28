#!/bin/bash
if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` DesiredModuleName"
  exit
fi
if [ -f $1 ] || [ -d $1 ]
then
  echo "Error: a file or a folder named" $1 "already exists"
  exit
fi
echo "Creating robot module:" $1
mkdir -p $1
mkdir -p $1/include
mkdir -p $1/src

RTK_pkg_tools_PATH=$(rospack find RTK_pkg_tools)


sed -e "s/EmptyRobotModule/$1/g" < $RTK_pkg_tools_PATH/EmptyRobotModule/include/EmptyRobotModule.h > ./$1/include/$1.h
sed -e "s/EmptyRobotModule/$1/g" < $RTK_pkg_tools_PATH/EmptyRobotModule/src/EmptyRobotModule.cpp   > ./$1/src/$1.cpp
echo "Module can now be found in folder:" $1
echo "- Don't forget to add it to your CMakeList.txt"


