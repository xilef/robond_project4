# Mapping and SLAM

Submission for Project 4 of the Udacity Robotics Software Engineer Nanodegree Program. This repository contains an upgrade to the robot to include an RGB-D camera to be able to use the rtabmap package.

## How to run

### Setup
Make sure you have gazebo and rviz installed and you have setup a catkin workspace

Clone the repo to your catkin workspace's src folder

eg:
`git clone https://github.com/xilef/robond_project4 /home/robond/workspace/catkin_ws/src
`

Go to the root of your catkin workspace and run `catkin_make` to build the whole repo

### Execute

When you open a new terminal make sure to run first `source devel/setup.bash` from the root of your catkin workspace.

Once all the setup is done run:

`roslaunch my_robot world.launch`

to load the world.

Open a new terminal, navigate to your catkin workspace and run the teleop node:

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Finally open a new terminal, nvaigate to your catkin workspace and run the mapping node:

`roslaunch my_robot mapping.launch`

Once everything is setup you can now manually control the robot all around the world to create a map.

## License

The content in this repository is free to use.
