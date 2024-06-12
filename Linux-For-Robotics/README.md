# Linux-For-Robotics #
![Alt text]([https://github.com/Fouad-Smaoui/ROS-Projects/blob/main/Linux-For-Robotics/ROS_Linux.png)
This project contains various scripts to control a BB-8 robot using ROS (Robot Operating System). The scripts include functionality for moving the robot in different patterns such as squares, circles, and simple forward and backward motions. Additionally, there's a script for teleoperation using keyboard input.

## Project Structure ##
The project includes the following scripts:
* move_bb8.py: Moves the BB-8 robot in a square pattern.
* process_test.py: A simple test script to move the BB-8 robot forward continuously.
* teleop_twist_keyboard.py: Allows manual control of the BB-8 robot using the keyboard.
* circle.py: Moves the BB-8 robot in a circular pattern.
* forward_and_backward.py: Moves the BB-8 robot forward for a specified time, then backward.
  
## Prerequisites ##
* ROS (Robot Operating System) installed and properly configured.
* A working installation of geometry_msgs package.
* The BB-8 robot or a simulation environment set up to receive and execute /cmd_vel commands.
  
## Installation ##
* Ensure that ROS is installed and sourced correctly.
* Clone this repository to your ROS workspace.
```
cd ~/catkin_ws/src
git clone <repository-url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Make sure the Python scripts are executable:
```
chmod +x src/Linux-For-Robotics/*.py
```

## Usage ##
### Moving BB-8 in a Square ###
To move the BB-8 robot in a square pattern, run the move_bb8.py script:
```
rosrun Linux-For-Robotics move_bb8.py
```
### Process Test ###
To run a simple test moving the BB-8 robot forward, use the process_test.py script:
```
rosrun Linux-For-Robotics process_test.py
```
### Teleoperation ###
To manually control the BB-8 robot using the keyboard, run the teleop_twist_keyboard.py script:
```
rosrun Linux-For-Robotics teleop_twist_keyboard.py
```
### Moving BB-8 in a Circle ###
To move the BB-8 robot in a circular pattern, use the circle.py script:
```
rosrun Linux-For-Robotics circle.py
```
Moving BB-8 Forward and Backward
To move the BB-8 robot forward and backward, use the forward_and_backward.py script:
```
rosrun Linux-For-Robotics forward_and_backward.py
```
## Scripts Overview ##
### move_bb8.py ###
This script moves the BB-8 robot in a square pattern. It publishes velocity commands to the /cmd_vel topic.

### process_test.py ###
A basic script to test continuous forward motion of the BB-8 robot. It also publishes to the /cmd_vel topic.

### teleop_twist_keyboard.py ###
A script to control the BB-8 robot using keyboard input. This script reads keyboard inputs and publishes the corresponding velocity commands to the /cmd_vel topic.

### circle.py ###
This script makes the BB-8 robot move in a circular pattern by setting both linear and angular velocities.

### forward_and_backward.py ###
A script to move the BB-8 robot forward for a specified duration, then backward for a specified duration.

## Contributing ##
1. Fork the repository.
2. Create a new branch (git checkout -b feature-branch).
3. Make your changes.
4. Commit your changes (git commit -am 'Add new feature').
5. Push to the branch (git push origin feature-branch).
6. Create a new Pull Request.

## License ##
This project is licensed under the MIT License.
