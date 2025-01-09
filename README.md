# Turtlebot3 Waypoint Navigation (sensor_fusion)

> Authors:
> Varad Nerlekar (UID: 120501135)
> Munyaradzi Antony (UID: 120482731)
> Siddhant Deshmukh (UID: 121322463)

## Introduction
This project implements a waypoint navigation system for the Turtlebot3 robot using ROS 2. The system consists of multiple components that work together to detect parts in the environment, publish waypoints, and navigate to those waypoints while ensuring the robot's readiness for the next task.

## Usage
1. Copy the package inside the src folder of your workspace.
2. Build the package and source it:
   colcon build && source install/setup.bash
3. To run the code, simply execute the following command:
   ros2 launch sensor_fusion sensor_fusion.launch.py

   (Note: No need to launch the final_project.launch.py, sensor_fusion.launch.py will launch it automatically)
