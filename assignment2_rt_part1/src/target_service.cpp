/**
* \file target_service.cpp
* \brief ROS service to retrieve the last received target coordinates.
* \author Francesca Magno
* \version 1.0
* \date 08/03/2025
*
* \subsection Published Topics:
*	- \b None
*
* \subsection Subscribed Topics:
*	- \b /target_topic: Receives target coordinates to be stored as the last target.
*
* \subsection Services :
*   - \b /get_last_target: Provides the last received target coordinates when requested.
*
* \subsection Description:
*   This node provides a service to return the last received target coordinates.
*   It subscribes to a topic where the target coordinates are published and stores the last received target.
*   When requested via the service, the node returns the last target coordinates.
*
**/



#include <ros/ros.h>
#include <assignment2_rt_part1/GetLastTarget.h>
#include <assignment2_rt_part1/Target.h>     
#include <mutex>

// Global variables to store the last received target
double last_target_x = 0.0; ///< Last X coordinate
double last_target_y = 0.0; ///< Last Y coordinate

// Mutex to ensure thread safety for accessing shared data
std::mutex target_mutex; ///< Mutex to ensure thread safety

/**
* \brief Service callback to provide the last received target coordinates.
* \param req The request object, which is empty.
* \param res The response object, which will be populated with the last target coordinates.
* \return true, it indicates that the service was successfully executed. 
*
* \details This function handles service requests to get the last received target coordinates,
*   using a mutex to ensure thread-safe access to shared data
*/
bool getLastTarget(assignment2_rt_part1::GetLastTarget::Request &req,
                   assignment2_rt_part1::GetLastTarget::Response &res) {
    
    // Lock the mutex to ensure safety
    std::lock_guard<std::mutex> lock(target_mutex);

	// Populate the service response
    res.x = last_target_x;
    res.y = last_target_y;

    ROS_INFO("Returning last target coordinates: x=%.2f, y=%.2f", res.x, res.y);
    return true;
}


/**
* \brief Callback function to handle incoming target messages.
* \param msg The received target message containing new coordinates.
*
* \details This function is called whenever a new target message is received.
*   It updates the last received target coordinates. The mutex is again used to ensure thread safety.
*/
void targetCallback(const assignment2_rt_part1::Target &msg) {

	// Lock the mutex to ensure safety
    std::lock_guard<std::mutex> lock(target_mutex);
    
    // Update the last target coordinates with the received message data
    last_target_x = msg.x;
    last_target_y = msg.y;

    ROS_INFO("Received new target: x=%.2f, y=%.2f", last_target_x, last_target_y);
}


/**
* \brief Main function to run the action client.
* \param argc Number of arguments.
* \param argv Argument vector.
* \return 0 on successful execution.
* 
* \details This function initializes the ROS node,  advertises the service to provide
*   the last received target coordinates, and subscribes to the target topic.
*   It ensures that the service is ready to handle requests and keeps the node spinning.
*/
int main(int argc, char **argv) {
	
	// Initialize ROS node
    ros::init(argc, argv, "target_service");
    ros::NodeHandle nh;

	// Advertize the service
    ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);
    // Subscriber
    ros::Subscriber sub = nh.subscribe("target_topic", 10, targetCallback);

    ROS_INFO("Target service node ready.");
    ros::spin();

    return 0;
}

