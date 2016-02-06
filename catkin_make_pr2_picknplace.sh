#!/bin/bash
# @Author: alex
# @Date:   2016-02-06
# @Last Modified by:   alex
# @Last Modified time: 2016-02-06

if [ "$#" -eq 1 ]; then
  if [ "$1" == "msgs" ]; then
    packages=($(ls ./pr2_picknplace_msgs/))
  else
    packages=("$1")
  fi
else
  packages=($(ls ./) $(ls ./pr2_picknplace_msgs/))
fi

white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list
