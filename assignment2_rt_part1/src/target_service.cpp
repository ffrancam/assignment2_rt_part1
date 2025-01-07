#include <ros/ros.h>
#include <assignment2_rt_part1/GetLastTarget.h>
#include <assignment2_rt_part1/Target.h>     
#include <mutex>

// Global variables to store the last recieved target
double last_target_x = 0.0;
double last_target_y = 0.0;

// Mutex to ensure thread safety for accesing shared data
std::mutex target_mutex;

// Service callback function
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

// Topic callback function
void targetCallback(const assignment2_rt_part1::Target &msg) {

	// Lock the mutex to ensure safety
    std::lock_guard<std::mutex> lock(target_mutex);
    
    // Update the last target coosdinates with the received message data
    last_target_x = msg.x;
    last_target_y = msg.y;

    ROS_INFO("Received new target: x=%.2f, y=%.2f", last_target_x, last_target_y);
}

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

