#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

class WarehouseMissionCoordinator(Node):
    def __init__(self, navigator):
        super().__init__('warehouse_mission_coordinator')
        self.navigator = navigator

        # Odometry for rotation tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.num_legs = 0
        self.total_readings = 0
        self.leg_data = []
        
        # --- TF buffer, listener, and broadcaster ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
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
        legs = 0
        self.total_readings = len(msg.ranges)
        local_leg_data = []
        leg_flag = False
        leg_start = 0

        for i, intensity in enumerate(msg.intensities):
            if intensity >= 8000.0:
                if not leg_flag:
                    leg_flag = True
                    leg_start = i
            else:
                if leg_flag:
                    leg_flag = False
                    leg_center = (leg_start + i) // 2
                    legs += 1
                    local_leg_data.append((leg_center, msg.ranges[leg_center]))

        self.num_legs = legs
        self.leg_data = local_leg_data

    def stop_robot(self):
        stop = Twist()
        for _ in range(5):
            self.vel_publisher.publish(stop)
            time.sleep(0.05)

    def rotate_to_target(self):
        self.get_logger().info("Rotating into alignment position...")
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.02)
        target_yaw = math.atan2(math.sin(self.current_yaw - math.pi / 2), math.cos(self.current_yaw - math.pi / 2))
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            error = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
            if abs(error) < math.radians(2.0):
                break
            cmd.angular.z = 0.25 if error > 0 else -0.25 
            self.vel_publisher.publish(cmd)
        self.stop_robot()
        self.get_logger().info("Rotation complete.")
        
    def drive_under_shelf(self):

        time.sleep(1.0)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"Legs detected: {self.num_legs}")

        if self.num_legs != 2:
            self.get_logger().error("Legs not found. Aborting.")
            return False

        # --- TRIANGULATION MATH ---
        a = self.leg_data[0][1]  # Left leg range
        b = self.leg_data[1][1]  # Right leg range
        
        idx_diff = abs(self.leg_data[0][0] - self.leg_data[1][0])
        theta = 2.0 * math.pi * idx_diff / self.total_readings
        
        c = math.sqrt(a * a + b * b - 2.0 * a * b * math.cos(theta))
        d = 0.5 * math.sqrt(2.0 * (a * a + b * b) - c * c)
        
        alpha = math.asin(max(-1.0, min(1.0, a * math.sin(theta) / c)))
        beta = math.asin(max(-1.0, min(1.0, (c / 2.0) * math.sin(alpha) / d)))
        
        x = d * math.sin(alpha + beta)
        y = d * math.cos(alpha + beta)

        # --- BROADCAST THE POSITION AS A STATIC TF FRAME ---
        try:
            # Query the location of the laser link relative to the global odom frame
            trans = self.tf_buffer.lookup_transform('odom', 'robot_front_laser_base_link', rclpy.time.Time())
            
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            laser_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            odom_cart_x = trans.transform.translation.x + x * math.cos(laser_yaw) - y * math.sin(laser_yaw)
            odom_cart_y = trans.transform.translation.y + x * math.sin(laser_yaw) + y * math.cos(laser_yaw)

            ts = TransformStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = 'odom'
            ts.child_frame_id = 'cart_frame'
            ts.transform.translation.x = odom_cart_x
            ts.transform.translation.y = odom_cart_y
            ts.transform.rotation.w = 1.0
            
            self.tf_static_broadcaster.sendTransform(ts)
            self.get_logger().info("Successfully broadcasted cart_frame to TF Tree.")
        except Exception as e:
            self.get_logger().error(f"Failed to calculate and broadcast TF frame: {str(e)}")
            return False

        time.sleep(0.5)

        # --- TF-DRIVEN APPROACH CONTROL LOOP ---
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            try:
                # Dynamically query frame error relative to the moving robot base link
                t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time())
                
                rel_x = t.transform.translation.x + 0.30
                rel_y = t.transform.translation.y

                if rel_x <= 0.04:
                    self.get_logger().info("Breaking loop")
                    break

                error_distance = math.sqrt(rel_x**2 + rel_y**2)
                error_yaw = math.atan2(rel_y, rel_x)

                cmd.angular.z = 0.3 * error_yaw
                cmd.linear.x = min(1.0 * error_distance, 0.15)  # Capped speed slightly for better accuracy
                self.vel_publisher.publish(cmd)
                time.sleep(0.1)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        self.stop_robot()
        return True

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

    if navigator.getResult() == TaskResult.SUCCEEDED:
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

    navigator.setInitialPose(init_pose)
    navigator.waitUntilNav2Active()

    # 1. MOVE TO LOADING STATION
    if navigate_to(navigator, x=5.72, y=-0.0297, qz=0.0, qw=1.0, label="loading_position"):
        coordinator = WarehouseMissionCoordinator(navigator)
        navigator.cancelTask()
        time.sleep(0.5)
        
        # 2. ALIGN AND DOCK UNDER SHELF ONLY
        coordinator.rotate_to_target()
        coordinator.drive_under_shelf()
        
        print("Robot under Shelf. Exiting script.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()