#!/usr/bin/env python3

import rospy
import actionlib
from assignment2_rt_part1.msg import RobotState, Target
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
import sys
import select

# Publishers
robot_state_pub = None
target_pub = None

def odom_callback(msg):
    """Callback function to receive odometry data and update robot state."""
    robot_state_msg = RobotState()
    robot_state_msg.x = msg.pose.pose.position.x
    robot_state_msg.y = msg.pose.pose.position.y
    robot_state_msg.vel_x = msg.twist.twist.linear.x
    robot_state_msg.vel_z = msg.twist.twist.angular.z
    
    # Publish robot state
    robot_state_pub.publish(robot_state_msg)

def get_target_from_user():
    """Function to get the target coordinates from the user."""
    while True:
        try:
            x = float(input("Enter Target X: "))
            y = float(input("Enter Target Y: "))
            
            target_msg = Target()
            target_msg.x = x
            target_msg.y = y
            target_pub.publish(target_msg)
            
            print("Press 'x' or 'X' to stop the robot during execution.")
            return x, y
        except ValueError:
            print("Invalid input. Please enter numerical values.")

def send_goal(client, target_x, target_y):
    """Function to send a goal to the action server."""
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    client.send_goal(goal)

def key_pressed():
    """Non-blocking function to check if a key is pressed."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():
    global robot_state_pub, target_pub
    
    rospy.init_node("action_client")
    robot_state_pub = rospy.Publisher("robot_state", RobotState, queue_size=10)
    target_pub = rospy.Publisher("target_topic", Target, queue_size=10)
    
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server started.")
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        target_x, target_y = get_target_from_user()
        stop_requested = False
        
        send_goal(client, target_x, target_y)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
            # Check for user input
            if key_pressed():
                input_char = sys.stdin.read(1)
                if input_char.lower() == 'x':
                    rospy.loginfo("Stopping the robot.")
                    client.cancel_goal()
                    stop_requested = True
                    break
            
            # Check goal status
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                print("Target reached successfully!")
                break
            
            rate.sleep()
        
        if stop_requested:
            print("Execution stopped. You can set a new target.")
            continue
        
        choice = input("Do you want to set another goal? (y/n): ")
        if choice.lower() == 'n':
            print("Exiting program.")
            break
    
    rospy.loginfo("Action client terminated.")

if __name__ == "__main__":
    main()

