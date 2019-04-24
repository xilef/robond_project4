# Ball chaser

Submission for Project 2 of the Udacity Robotics Software Engineer Nanodegree Program. This repository contains a robot that has the following features:
- Chase a solid colored ball if it is within camera view.
- If it is chasing a ball and it goes out of camera view, seek the ball by turning approximately 360 degrees then give up until a new ball is seen.

## How to run

### Setup
Make sure you have gazebo and rviz installed and you have setup a catkin workspace

Clone the repo to your catkin workspace's src folder

eg:
`git clone https://github.com/xilef/robond_project1 /home/robond/workspace/catkin_ws/src
`

Go to the root of your catkin workspace and run `catkin_make` to build the whole repo

### Execute

When you open a new terminal make sure to run first `source devel/setup.bash` from the root of your catkin workspace.

Once all the setup is done run:

`roslaunch my_robot world.launch`

to load the world.

Open a new terminal, navigate to your catkin workspace and run:

`roslaunch ball_chaser ball_chaser.launch`

to run the ball chasing service

## License

The content in this repository is free to use.
