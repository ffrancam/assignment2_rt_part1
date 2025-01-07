# Assignment 2.1 - Robot Navigation

## **Overview**

This project consists of two nodes and configurations for controlling a robot's navigation and monitoring its state within a ROS environment. Its main functionalities are:
- Allowing the user to set a navigation target (x, y) or cancel the current target,
- Utilizing feedback/status from the action server to determine when the target is reached,
- Publishing the robot's position and velocity as a custom message (x, y, vel_x, vel_z) based on data from the /odom topic.
- Providing the coordinates of the last target set by the user when requested via a ROS service.



## **Getting Started**

### 1. Install ROS
In order to run the project, ROS and turtlesim package must be installed.

### 2. Clone the repository 
Clone the repository into the src folder of your ROS workspace:

```bash
git clone <URL_of_this_repository>
```

### 3. Clone the action server 

```bash
git clone https://github.com/CarmineD8/assignment_2_2024.git
```
### 4. Launch the simulation
```bash
roslaunch assignment2_rt_part1 assignment2_part1.launch 
```

## How it works

### • ActionClient.cpp
This script implements an action client for controlling a robot's navigation. Its main features are:

#### - User Input for Target
The user is prompted to input target coordinates (x, y) for the robot's destination.
#### - Goal Sending
The input target coordinates are sent as a goal to an action server using `PlanningAction`.
#### - Robot State Publishing
The robot's position and velocity (x, y, vel_x, vel_z) are published to a custom message topic based on odometry data.
#### - Goal Monitoring
The robot's progress is monitored through action feedback, and the user can cancel the goal by pressing a key ('x' or 'X'). The program checks if the goal has been successfully reached or stopped.
#### - Iterative Navigation
The user can set new goals or stop the robot after each navigation attempt.

### • TargetService.cpp
This script implements a ROS service to retrieve the last received target coordinates. Its main features are:

#### - Service to Get Last Target
A service is provided via `get_last_target` that returns the last target coordinates (x, y) received by the system.
#### - Target Data Subscription
The node subscribes to the `target_topic` to receive target coordinates and updates the last received target whenever new data is published.
#### - Thread Safety
A mutex (`std::mutex`) is used to ensure thread safety when accessing and updating the last target coordinates.
#### - Data Handling
The service callback returns the latest target coordinates upon request, and the target callback function updates the stored coordinates whenever a new target is received.








