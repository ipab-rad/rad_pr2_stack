# rad_pr2_stack
Everything necessary to get our PR2 in the RAD lab running in tip-top shape


# To install:

```
# Add dependencies
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-pr2-controllers ros-indigo-pr2-controllers-msgs
sudo apt-get install ros-indigo-pr2-gripper-sensor-msgs

# If usaing older v ersion of git, make sure that submodules are pulled
# git submodule update --init --recursive

# Build haf-grasping
roscd haf_grasping/libsvm-3.12
make

# Build rad-pr2-stack
catkin_make
```
