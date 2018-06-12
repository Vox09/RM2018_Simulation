## How to use
git clone to your catkin\_ws
then catkin\_make

##### Environment setup 
> rosrun rm2018\_simulation setup.sh

make sure the *GAZEBO\_MODEL\_PATH* and *GAZEBO\_PLUGIN\_PATH* variables include the dir *meshses* and *plugins* in the project.

then run the command:
> roslaunch rm\_simulation two.launch gui:=true

to see the simulation with GUI

then run the script:
> rosrun rm\_simulation image\_view0.sh

to see the camera image
> rosrun rm\_simulation teleop0.py

to control the vehicle 
The python package 'pyautogui' is needed

number 0 is the blue vehicle

number 1 is the red one 

## v2.0 Update

Treat whole robot as an entity to realize strafe function.
Now directly set model's linear vel instead of simulating motors.

## TODO
Improve control scripts

