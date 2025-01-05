#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment2_rt_part1/RobotState.h>
#include <iostream>
#include <thread>
#include <atomic>

// Define the action client
typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> Client;

// Global variables to store robot state
geometry_msgs::Pose current_pose;
geometry_msgs::Twist current_velocity;

// Atomic flag to stop the robot
std::atomic<bool> stop_requested(false);

// Function to receive odometry data and update position and velocity
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose = msg->pose.pose;
    current_velocity = msg->twist.twist;
}

// Function to get the target coordinates from the user
void getTargetFromUser(float& x, float& y) {
    while (true) {
        std::cout << "Enter the target coordinates of the robot." << std::endl;
        std::cout << "Target X: ";
        if (std::cin >> x) {
            std::cout << "Target Y: ";
            if (std::cin >> y) {
                std::cout << "Press X to stop the robot during execution." << std::endl;
                break;
            }
        }
        // The inserted input is not valid
        std::cerr << "The inserted input is not valid. Please insert numerical values." << std::endl;
        std::cin.clear(); // Reset error flag
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore remaining input
    }
}

// Function to send a goal to the action server
void sendGoal(Client& ac, float target_x, float target_y) {
    assignment_2_2024::PlanningGoal goal;
    goal.target_pose.pose.position.x = target_x;
    goal.target_pose.pose.position.y = target_y;
    ac.sendGoal(goal);
}

// Function to handle user input in a separate thread
void userInputThread() {
    while (!stop_requested) {
        char input;
        std::cin >> input;
        if (input == 'x' || input == 'X') {
            stop_requested = true;
            std::cout << "Stopping the robot." << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "Assignment2_action_client");
    ros::NodeHandle nh;
    ros::Rate rate(10); // Lower frequency is sufficient here

    // Publisher for stopping the robot
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the /odom topic to get the robot's position and velocity
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Create an action client that connects to the "reaching_goal" action server
    Client ac("/reaching_goal", true);

    // Wait until the action server is available
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    // Get target position from the user
    float target_x, target_y;
    getTargetFromUser(target_x, target_y);

    // Start the user input thread
    std::thread input_thread(userInputThread);

    // Send a goal to the action server
    sendGoal(ac, target_x, target_y);

    while (ros::ok() && !stop_requested) {
        ros::spinOnce(); // Process incoming messages

        // Check the state of the action
        actionlib::SimpleClientGoalState state = ac.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            std::cout << "Target reached successfully!" << std::endl;
            break;
        }

        rate.sleep();
    }

    // If stopping, cancel the goal
    if (stop_requested) {
        ac.cancelGoal();
    }

    // Clean up
    if (input_thread.joinable()) {
        input_thread.join();
    }

    ROS_INFO("Exiting action client.");
    return 0;
}

