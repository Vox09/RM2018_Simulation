## How to use
git clone to your catkin\_ws
then catkin\_make

just run the command:
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

## TODO
Improve control
Develope strafe function(solve the simulation problem of mecanum wheel)
