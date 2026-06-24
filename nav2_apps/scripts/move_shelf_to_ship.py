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

# NEW IMPORTS FOR DYNAMIC FOOTPRINT UPDATES
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class WarehouseMissionCoordinator(Node):
    def __init__(self, navigator):
        super().__init__('warehouse_mission_coordinator')
        self.navigator = navigator

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.detected_legs = []

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Both Up and Down elevator publishers
        self.elevator_up_pub = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_down_pub = self.create_publisher(String, '/elevator_down', 10)
        
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

    # FOOTPRINT UPDATER
    def change_nav2_footprint(self, footprint_string):
        self.get_logger().info(f"Updating Nav2 footprint to: {footprint_string}")
        
        costmaps = ['/global_costmap/global_costmap', '/local_costmap/local_costmap']

        for costmap_name in costmaps:
            client = self.create_client(SetParameters, f'{costmap_name}/set_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"Service {costmap_name}/set_parameters not available!")
                continue

            req = SetParameters.Request()
            param_value = ParameterValue(string_value=footprint_string, type=ParameterType.PARAMETER_STRING)
            req.parameters.append(Parameter(name='footprint', value=param_value))

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Footprint updated for {costmap_name}")

    # ROTATION & DRIVING LOGIC
    def rotate_to_target(self):
        self.get_logger().info("Executing P-Controller Rotation to -90 degrees...")
        
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
        
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(self.detected_legs) < 2:
            self.get_logger().error("Cannot dock! Did not detect at least 2 legs.")
            return False

        mid_x = (self.detected_legs[0][0] + self.detected_legs[1][0]) / 2.0
        
        offset = 0.45 
        target_distance = mid_x + offset
        
        self.get_logger().info(f"Legs found! Midpoint is {mid_x:.3f}m away.")
        self.get_logger().info(f"Adding {offset}m offset. Driving forward {target_distance:.3f}m...")
        
        start_x = self.current_x
        start_y = self.current_y
        cmd = Twist()
        cmd.linear.x = 0.15 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            distance_moved = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            if distance_moved >= target_distance:
                break
                
            self.vel_publisher.publish(cmd)
            
        self.stop_robot()
        self.get_logger().info("Robot is perfectly positioned under the shelf.")
        return True

    def drive_backwards(self, distance):
        self.get_logger().info(f"Reversing {distance}m to clear the shelf legs...")
        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = -0.15 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            distance_moved = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            if distance_moved >= distance: break
            self.vel_publisher.publish(cmd)
            
        self.stop_robot()

    def set_elevator(self, direction="up"):
        self.get_logger().info(f"Activating elevator: {direction.upper()}...")
        msg = String()
        msg.data = direction 
        publisher = self.elevator_up_pub if direction == "up" else self.elevator_down_pub
        
        publisher.publish(msg)
        time.sleep(2.0)

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

    # --- NAVIGATE TO LOADING POSITION ---
    success = navigate_to(
        navigator,
        x=5.72, y=-0.0297,  
        qz=0.0, qw=1.0,
        label="loading_position"
    )

    if success:
        # --- DOCKING ---
        print("Starting custom precision docking sequence...")
        coordinator = WarehouseMissionCoordinator(navigator)
        
        navigator.cancelTask()
        time.sleep(1.0)
        coordinator.stop_robot()
        
        coordinator.rotate_to_target()
        
        if coordinator.drive_under_shelf():
            coordinator.set_elevator("up")
            
            # EXPAND FOOTPRINT TO AVOID WALLS
            coordinator.change_nav2_footprint('[[-0.4, -0.4], [-0.4, 0.4], [0.4, 0.4], [0.4, -0.4]]')

            # --- SHIPPING POSITION ---
            print("Docking complete! Initiating Phase 3: Delivery...")
            success_ship = navigate_to(
                navigator,
                x=2.22, y=1.40,  
                qz=-0.00175, qw=0.0,  
                label="shipping_position"
            )
            
            if success_ship:
                print("Arrived at shipping position. Unloading...")
                
                # --- UNLOAD ---
                navigator.cancelTask()
                time.sleep(1.0)
                coordinator.stop_robot()
                
                coordinator.set_elevator("down")
                coordinator.drive_backwards(0.6)
                
                # SHRINK FOOTPRINT BACK TO NORMAL
                coordinator.change_nav2_footprint('[[-0.2, -0.2], [-0.2, 0.2], [0.2, 0.2], [0.2, -0.2]]')

                # --- RETURN HOME ---
                print("Shelf delivered! Returning to Init Position...")
                navigate_to(navigator, x=0.022, y=0.00909, qz=0.0, qw=1.0, label="init_position")
                print("Mission Complete!")
                
    else:
        print("Navigation to loading position failed.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()