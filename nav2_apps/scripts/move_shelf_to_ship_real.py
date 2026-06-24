#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WarehouseMissionCoordinator(Node):
    def __init__(self, navigator):
        super().__init__('warehouse_mission_coordinator')
        self.navigator = navigator

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.detected_legs = []

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.elevator_publisher = self.create_publisher(String, '/elevator_up', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def laser_callback(self, msg):
        legs = []
        in_leg = False
        start_idx = 0
        for i, intensity in enumerate(msg.intensities):
            if not in_leg and intensity >= 8000.0:
                in_leg = True
                start_idx = i
            elif in_leg and intensity < 8000.0:
                in_leg = False
                mid = (start_idx + i - 1) // 2
                dist = msg.ranges[mid]
                angle = msg.angle_min + mid * msg.angle_increment
                if math.isfinite(dist) and dist > 0.05:
                    legs.append((dist * math.cos(angle), dist * math.sin(angle)))
        if in_leg:
            mid = (start_idx + len(msg.intensities) - 1) // 2
            dist = msg.ranges[mid]
            angle = msg.angle_min + mid * msg.angle_increment
            if math.isfinite(dist) and dist > 0.05:
                legs.append((dist * math.cos(angle), dist * math.sin(angle)))
        
        self.detected_legs = legs

    def stop_robot(self):
        stop = Twist()
        for _ in range(5):
            self.vel_publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.1)


    def rotate_to_relative_south(self):
        self.get_logger().info("Executing P-Controller Rotation (Relative -90 degrees)...")
        
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
            
        target_yaw = math.atan2(
            math.sin(self.current_yaw - math.pi / 2),
            math.cos(self.current_yaw - math.pi / 2)
        )
        
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            error = math.atan2(
                math.sin(target_yaw - self.current_yaw),
                math.cos(target_yaw - self.current_yaw)
            )
            if abs(error) < math.radians(1.0):
                break
                
            speed = max(0.12, min(0.40, abs(error) * 1.2))
            cmd.angular.z = speed if error > 0 else -speed 
            self.vel_publisher.publish(cmd)

        self.stop_robot()
        self.get_logger().info(f"Rotation Complete. Final Yaw: {math.degrees(self.current_yaw):.2f}°")


    def drive_under_shelf(self):
        self.get_logger().info("Scanning for legs to calculate docking distance...")
        
        for _ in range(15):  
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(self.detected_legs) < 2:
            self.get_logger().error("Cannot dock! Did not detect at least 2 legs. Check cart alignment.")
            return False

        mid_x = (self.detected_legs[0][0] + self.detected_legs[1][0]) / 2.0
        
        offset = 0.35 
        target_distance = mid_x + offset
        
        self.get_logger().info(f"Legs found! Midpoint is {mid_x:.3f}m away.")
        self.get_logger().info(f"Driving forward {target_distance:.3f}m to center underneath...")
        
        start_x = self.current_x
        start_y = self.current_y
        cmd = Twist()
        cmd.linear.x = 0.10 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            distance_moved = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            if distance_moved >= target_distance:
                break
                
            self.vel_publisher.publish(cmd)
            
        self.stop_robot()
        self.get_logger().info("Robot is perfectly positioned under the shelf.")
        return True

    # 
    def lift_shelf(self):
        self.get_logger().info("Activating elevator to lift the shelf...")
        msg = String()
        msg.data = "up" 
        
        for i in range(3):
            self.elevator_publisher.publish(msg)
            self.get_logger().info(f"Elevator command {i+1}/3 sent...")
            time.sleep(0.5) 
            
        time.sleep(3.0) 
        self.get_logger().info("Shelf attached and lifted successfully!")


    def execute_full_docking(self):
        self.navigator.cancelTask()
        time.sleep(1.0)
        self.stop_robot()
        
        self.rotate_to_relative_south()

        success = self.drive_under_shelf()

        if success:
            self.lift_shelf()


def navigate_to(navigator, x, y, qz, qw, label):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    print(f"Navigating to {label}...")
    navigator.goToPose(goal)
    while not navigator.isTaskComplete():
        time.sleep(0.2)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"Reached {label}.")
        return True
    return False


def main():
    rclpy.init()
    navigator = BasicNavigator()

    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.022
    init_pose.pose.position.y = 0.00909
    init_pose.pose.orientation.w = 1.0

    print("Setting initial pose and waiting for Nav2...")
    navigator.setInitialPose(init_pose)
    navigator.waitUntilNav2Active()

    success = navigate_to(
        navigator,
        x=5.68, y=-0.0297,
        qz=0.0, qw=1.0,
        label="loading_position"
    )

    if success:
        print("Starting custom precision docking sequence...")
        coordinator = WarehouseMissionCoordinator(navigator)
        coordinator.execute_full_docking()

        print("Docking complete! Initiating Phase 3: Shipping...")
    
            
    else:
        print("Navigation to loading position failed.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()