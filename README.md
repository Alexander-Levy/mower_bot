# Autonomous Lawn Mower Diferential Drive Robot 

## Introduction

This metapackage contains 3 packages necesary for Lawn Mower Operation. The proyect consist of an autonomous robot that can:

    - Map its working area 
    - Locate itself within that area 
    - Plan the best path to mow the lawn 
    - Calculate the appropiate wheel velocities to achieve this

This is a ROS2 based proyect that is built to work on: 

    Linux distro: Ubuntu 22.04 LTS
    ROS2 distro: Humble Hawksbill

## File Organization
This metapackage consist of 5 main folders:

- articubot_one: Includes robot description, parameters and launch files for our robot.

- coverage_logic: Includes the path planning and navigation trees for coverage operations

- nav_api: Includes autonmous system and interfaces for navigation

