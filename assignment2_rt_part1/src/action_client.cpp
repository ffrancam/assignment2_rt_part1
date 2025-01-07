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

// Publisher for robot state
ros::Publisher robot_state_pub;

// Define the action client
typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> Client;

// Robot state message
assignment2_rt_part1::RobotState robot_state_msg;
assignment2_rt_part1::Target target_msg;

// Atomic flags to control program state
std::atomic<bool> stop_requested(false);
std::atomic<bool> goal_reached(false);

// Function to check if a key has been pressed (non-blocking)
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

// Function to receive odometry data and update position and velocity
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Get position and velocity from odometry data
    robot_state_msg.x = msg->pose.pose.position.x;
    robot_state_msg.y = msg->pose.pose.position.y;
    robot_state_msg.vel_x = msg->twist.twist.linear.x;
    robot_state_msg.vel_z = msg->twist.twist.angular.z;

    // Publish custom message
    robot_state_pub.publish(robot_state_msg);
}

// Function to get the target coordinates from the user
void getTargetFromUser(float& x, float& y) {
    while (true) {
        std::cout << "Enter the target coordinates of the robot." << std::endl;
        std::cout << "Target X: ";
        if (std::cin >> x) {
            std::cout << "Target Y: ";
            if (std::cin >> y) {
            	target_msg.x = x;
            	target_msg.y = y;
                std::cout << "Press 'x' or 'X' to stop the robot during execution." << std::endl;
                break;
            }
        }
        std::cerr << "Invalid input. Please enter numerical values." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

// Function to send a goal to the action server
void sendGoal(Client& ac, float target_x, float target_y) {
    assignment_2_2024::PlanningGoal goal;
    goal.target_pose.pose.position.x = target_x;
    goal.target_pose.pose.position.y = target_y;
    ac.sendGoal(goal);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client");
    ros::NodeHandle nh;
    ros::Rate rate(10);

	// Publisher
	robot_state_pub = nh.advertise<assignment2_rt_part1::RobotState>("robot_state", 10);

    // Subscriber
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

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

