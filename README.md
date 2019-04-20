# Drone Vision Landing Project on PX4 SITL Simulation
![image](https://github.com/Oliver-ss/DroneLanding/blob/master/Pictures/simulation.png)

## Our project is to make the drone autonomously land on a launch pad by ROS.

## The task could be mostly split into two parts: Vision and Control. 

The first part is vision system which would detect the pad on the ground and compute the relative distance. This is based on opencv aruco marker algorithm. (/catkin_ws/src/opencvtest_py/src/test.py) The input of the system is the raw image of the camera and the output is the relative distance between drone and the launch pad in camera reference frame.

The second part is the PID control system(catkin_ws/control/src/control.py). When the vision system publishes the relative distance , the control system would subscribe it and use it as the input. A 3 dimension velocity loop PID control is built as we would control the x, y, z linear velocity to approach the mark.

## Have fun about our project!!!


# Quick Start
## Open the PX4 simulation environment
cd ~/src/Firmware

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 drone.launch

## Run the offboard program so that the drone could enter offboard mode and be armed
rosrun offboard offboard_node

## Run the keyboard program so that we could use keyboard to make the drone fly
rosrun keyboard keyboard_ndoe

## Run the vision system
rosrun opencvtest_py test.py

## Run the control system(Only when the mark is within the view of drone camera)
rosrun control control.py

## Then you could use keyboad to move the drone and let it automously landing on the mark by control program!
