#!/usr/bin/env python3

import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()

    #intial position
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.0234
    initial_pose.pose.position.y = 2.5702
    initial_pose.pose.position.z = 0.0283
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    print("Localizing the robot at the initial position...")
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # --- (loading_position) ---
    loading_goal = PoseStamped()
    loading_goal.header.frame_id = 'map'
    loading_goal.header.stamp = navigator.get_clock().now().to_msg()
    loading_goal.pose.position.x = 5.48
    loading_goal.pose.position.y = 0.0281
    loading_goal.pose.position.z = 0.00154
    loading_goal.pose.orientation.z = 0.0
    loading_goal.pose.orientation.w = 1.0

    print("Sending navigation goal to loading_position...")
    navigator.goToPose(loading_goal)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        
        time.sleep(0.2)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Success! Robot arrived safely at the loading_position.")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled.")
    elif result == TaskResult.FAILED:
        print("Navigation failed.")
    else:
        print("Goal returned an invalid state status.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()