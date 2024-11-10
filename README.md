# Autonomous Lawn Mower Diferential Drive Robot 

## Introduction

This is the code for my thesis in university. The proyect consist of an autonomous robot that can:
    
    - Map its working area 
    - Locate itself within that area 
    - Plan the best path to mow the lawn 
    - Calculate the appropiate wheel velocities to achieve this

This is a ROS2 based proyect that is built to work on: 
    
    Linux distro: Ubuntu 22.04 LTS
    ROS2 distro: Humble Hawksbill

The processing of data is distributed between a Raspberry Pi running on the lawn mower and a laptop running on the same network

## How the package is organized

### config
Contains all of the necesary configuration files. This includes rviz views, parameters for the robot controller, slam and navigation, as well as configuration for the tele-operation and multiplexer. 

### description
Contains the robot's description written in xacro. Contains information about the robots dimensions, mass, actuators, sensors, etc. Also contains gazebo plugins for simulation purposes

### launch
Contains the launch files to startup the robot, the robot sensors, slam, navigation and simulation.

### worlds
Contains the world files used in simulation. An empty world, a world full of obstacles and a world where the robot spawns inside four walls.

## Usage

### Simulation
The simulation can be launched with
```bash
ros2 launch mower_bot sim.launch.py 
```
You can decide if the simulation visualizes gazebo using the "headless" launch argument. For example:
```bash
ros2 launch mower_bot sim.launch.py headless:=False 
```
The default is set to True.

### Real World Use
The robot can be initialized with the following command.This will launch the robot state publisher, the joint state broadcaster, the rplidar driver and the controller manager and differential drive controller.
```bash
ros2 launch mower_bot mower.launch.py 
```

## Dependencies
Since the load is distributed between two machines, some packages are only used by one of them so its not necesarry to install everything on both machines. I will make it clear which pacakges are used by what.

### General Dependencies:

#### These packages
```bash
sudo apt install -y                         \
    joystick                                \
    jstest-gtk evtest                       \ 
    python3-argcomplete                     \
    software-properties-common              \
    python3-colcon-common-extensions        \
```
#### These ROS2 packages:
```bash
sudo apt install -y                         \
    ros-humble-xacro                        \
    ros-humble-joint-state-publisher-gui    \
    ros-humble-gazebo-ros-pkgs              \
    ros-humble-rplidar-ros                  \
    ros-humble-ros2-control                 \
    ros-humble-ros2-controllers             \
    ros-humble-gazebo-ros2-control          \
    ros-humble-slam-toolbox                 \
    ros-humble-navigation2                  \
    ros-humble-nav2-bringup                 \
    ros-humble-turtlebot3                   \
    ros-humble-twist-mux                    \
```

### Dependencies only needed on the laptop:
#### Fields2Cover and Opennav_Coverage 
First go to your work space or create if you have not already
```bash
cd ~
mkdir -p <path_to_your_workspace>/src
cd <path_to_your_workspace>/src
```
Then download the following files
```bash
git clone https://github.com/Fields2Cover/Fields2Cover.git -b v1.2.1
git clone https://github.com/open-navigation/opennav_coverage.git -b humble
```
Go to the main project folder of Fields2Cover and build the library:
```bash
cd ~/<path_to_your_workspace>/src/Fields2Cover
mkdir -p build; 
cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; 
make -j$(nproc); 
sudo make install;
```
Finally, get back to the main project folder, build, source & test
```bash
cd ~/<path_to_your_workspace>
colcon build --symlink-install
source install/setup.bash
ros2 launch opennav_coverage_demo coverage_demo_launch.py
```

### Rasberry Pi dependency only: 
#### This Package
```bash
sudo apt install -y                         \
    libraspberrypi-bin                      \
```
#### diffdrive_arduino  
Go into your workspace, clone the files and build 
```bash
cd ~/<path_to_your_workspace>/src
git clone https://github.com/joshnewans/diffdrive_arduino.git -b humble
cd ..
colcon build --symlink-install
```
#### rplidar_ros
Go into your workspace, clone the files and build
```bash
cd ~/<path_to_your_workspace>/src
git clone git@github.com:babakhani/rplidar_ros2.git
cd ..
colcon build --symlink-install
source ./install/setup.bash
```
Make sure that the package exist
```bash
ros2 pkg list | grep rplidar
```

## Install
To use this package please download all of the necesary dependencies first and then follow these steps
```bash
cd ~/<path_to_your_workspace>/src
git clone https://github.com/Alexander-Levy/mower_bot.git 
cd ..
colcon build --symlink-install
```
