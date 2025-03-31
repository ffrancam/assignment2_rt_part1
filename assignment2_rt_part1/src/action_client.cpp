/**
* \file action_client.cpp
* \brief ROS action client for sending navigation goals and publishing the robot state
* \author Francesca Magno
* \version 1.0
* \date 08/03/2025
*
* \subsection Published Topics:
*	- \b /robot_state: Publishes the robot's current state, including position (x, y) and velocity (vel_x, vel_<).
*	- \b /target_topic: Publishes the target coordinates set by the user (x, y).
*
* \subsection Subscribed Topics:
*	- \b /odom: Receives odometry data to update the robot's position and velocity in real time.
*
* \subsection Action Clients :
*   - \b /reaching_goal: Sends a goal containing the target coordinates (x, y) to the action server, which moves the robot to the destination.
*
* \subsection Description:
*   This node implements an action client that allows the user to control the robot's navigation by setting target coordinates.  
*   The user can stop the execution at any time by pressing the "x" key.  
*   The robot's state, including position and velocity, is continuously published.
*
**/



#include <ros/ros.h>
#include <assignment2_rt_part1/RobotState.h>
#include <assignment2_rt_part1/Target.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <atomic>
#include <limits>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Publisher for robot state and target coordinates
ros::Publisher robot_state_pub; ///< Publishes the robot's current state to /robot_state.
ros::Publisher target_pub; ///< Publishes the target coordinates to /target_topic.


// Define the action client
typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> Client;

// Robot state message
assignment2_rt_part1::RobotState robot_state_msg; ///< Stores the robot's state.
assignment2_rt_part1::Target target_msg; ///< Stores the target coordinates.

// Atomic flags to control program state
std::atomic<bool> stop_requested(false); ///< Flag to signal a stop request by the user.
std::atomic<bool> goal_reached(false); ///< Flag indicating whether the goal was reached.


/**
* \brief Function to check if a key has been pressed (non-blocking).
* \return 1 if a key is pressed, 0 otherwise.
*
* \details This function check is a key is pressed by the user without blocking the program's execution, 
*   allowing real-time use input during the robot's navigation to stop it.
*/
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}


/**
* \brief Callback function to handle odometry data update.
* \param msg Pointer to the received odometry message.
*
* \details This function is called when new odometry data is recieved. It updates the robot's
* position and velocity and publishes the current robot state.
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Get position and velocity from odometry data
    robot_state_msg.x = msg->pose.pose.position.x;
    robot_state_msg.y = msg->pose.pose.position.y;
    robot_state_msg.vel_x = msg->twist.twist.linear.x;
    robot_state_msg.vel_z = msg->twist.twist.angular.z;

    // Publish custom message
    robot_state_pub.publish(robot_state_msg);
}


/**
* \brief Prompts the user to enter target coordinates.
* \param x Reference to store the target X-coordinate.
* \param y Reference to store the target Y-coordinate.
*
* \details This function continuously prompts the user to input valid target coordinates.
*   It then publishes the coordinates to the target topic.
*/
void getTargetFromUser(float& x, float& y) {
    while (true) {
        std::cout << "Enter the target coordinates of the robot." << std::endl;
        std::cout << "Target X: ";
        if (std::cin >> x) {
            std::cout << "Target Y: ";
            if (std::cin >> y) {
            	target_msg.x = x;
            	target_msg.y = y;
            	target_pub.publish(target_msg);
                std::cout << "Press 'x' or 'X' to stop the robot during execution." << std::endl;
                break;
            }
        }
        std::cerr << "Invalid input. Please enter numerical values." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}


/**
* \brief Sends a navigation goal to the action server.
* \param ac Reference to the action client.
* \param target_x X-coordinate of the target. 
* \param target_y Y-coordinate of the target.
*
* \details This function sends a goial with the target coordinate to the action server, 
*   which will move the robot towards the target.
*/
void sendGoal(Client& ac, float target_x, float target_y) {
    assignment_2_2024::PlanningGoal goal;
    goal.target_pose.pose.position.x = target_x;
    goal.target_pose.pose.position.y = target_y;
    ac.sendGoal(goal);
}


/**
* \brief Main function to run the action client.
* \param argc Number of arguments.
* \param argv Argument vector.
* \return 0 on successful execution.
* 
* \details This function initializes the ROS node, creates the action client and handles the main loop.
*   It allows the user to input the target coordinates, sends the goal to the action server
*   and checks for key presses to stop the robot.
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client");
    ros::NodeHandle nh;
    ros::Rate rate(10);

	// Publisher
	robot_state_pub = nh.advertise<assignment2_rt_part1::RobotState>("robot_state", 10);
	target_pub = nh.advertise<assignment2_rt_part1::Target>("target_topic", 10);

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Initialize the action client and wait for the server
    Client ac("/reaching_goal", true);
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    while (ros::ok()) {
        float target_x, target_y;
        getTargetFromUser(target_x, target_y);

        stop_requested = false;
        goal_reached = false;

        sendGoal(ac, target_x, target_y);

        while (ros::ok()) {
            ros::spinOnce();

            // Check if a key is pressed
            if (kbhit()) {
                char input = getchar();
                if (input == 'x' || input == 'X') {
                    stop_requested = true;
                    std::cout << "Stopping the robot." << std::endl;
                    ac.cancelGoal();
                    break; // Exit the goal processing loop
                }
            }

            // Check the state of the action
            actionlib::SimpleClientGoalState state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                std::cout << "Target reached successfully!" << std::endl;
                goal_reached = true;
                break; // Exit the goal processing loop
            }

            rate.sleep();
        }

        // If the goal was stopped, prompt the user for a new goal
        if (stop_requested) {
            std::cout << "Execution stopped. You can set a new target." << std::endl;
            continue; // Restart the main loop to set a new target
        }

        // If the goal was reached, ask the user if they want to continue
        char choice;
        std::cout << "Do you want to set another goal? (y/n): ";
        std::cin >> choice;

        if (choice == 'n' || choice == 'N') {
            std::cout << "Exiting program." << std::endl;
            break;
        }
    }

    ROS_INFO("Action client terminated.");
    return 0;
}

