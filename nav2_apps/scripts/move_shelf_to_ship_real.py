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
from std_msgs.msg import String

# Imports required for dynamic parameter updates
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
import tf2_ros

class WarehouseMissionCoordinator(Node):
    def __init__(self, navigator):
        super().__init__('warehouse_mission_coordinator')
        self.navigator = navigator

        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.num_legs = 0
        self.total_readings = 0
        self.leg_data = []
        
        # TF system setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Actuation & Sensor interfaces
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.elevator_publisher = self.create_publisher(String, '/elevator_up', 10)
        
        # Service clients to dynamically change Nav2 parameters
        self.global_costmap_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters'
        )
        self.local_costmap_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters'
        )
        
        # CHANGED: Subscribed to the physical robot's odometry topic
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
        """EXACT translation of the Checkpoint 9 C++ laser_callback loop"""
        legs = 0
        self.total_readings = len(msg.ranges)
        local_leg_data = []
        leg_flag = False
        leg_start = 0

        for i, intensity in enumerate(msg.intensities):
            if intensity >= 3500.0:  # CHANGED: Tuned for real-world metal leg reflectivity
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

        approach_yaw = self.current_yaw
        
        # Triangulation Math
        a = self.leg_data[0][1]  
        b = self.leg_data[1][1]  
        idx_diff = abs(self.leg_data[0][0] - self.leg_data[1][0])
        theta = 2.0 * math.pi * idx_diff / self.total_readings
        c = math.sqrt(a * a + b * b - 2.0 * a * b * math.cos(theta))
        d = 0.5 * math.sqrt(2.0 * (a * a + b * b) - c * c)
        alpha = math.asin(max(-1.0, min(1.0, a * math.sin(theta) / c)))
        beta = math.asin(max(-1.0, min(1.0, (c / 2.0) * math.sin(alpha) / d)))
        x = d * math.sin(alpha + beta)
        y = d * math.cos(alpha + beta)

        # Broadcast static frame
        try:
            # CHANGED: 'odom' -> 'robot_odom' for live physical robot hardware transformations
            trans = self.tf_buffer.lookup_transform('robot_odom', 'robot_front_laser_base_link', rclpy.time.Time())
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            laser_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            odom_cart_x = trans.transform.translation.x + x * math.cos(laser_yaw) - y * math.sin(laser_yaw)
            odom_cart_y = trans.transform.translation.y + x * math.sin(laser_yaw) + y * math.cos(laser_yaw)

            ts = TransformStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = 'robot_odom'  # CHANGED: 'odom' -> 'robot_odom'
            ts.child_frame_id = 'cart_frame'
            ts.transform.translation.x = odom_cart_x
            ts.transform.translation.y = odom_cart_y
            ts.transform.rotation.w = 1.0
            
            self.tf_static_broadcaster.sendTransform(ts)
            self.get_logger().info("Successfully broadcasted cart_frame to TF Tree.")
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast TF frame: {str(e)}")
            return False

        time.sleep(0.5)

        # --- PHASE A: align heading to cart_frame while completely stationary ---
        self.get_logger().info("Phase A: aligning heading to cart_frame (stationary)...")
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            align_rel_x = t.transform.translation.x
            align_rel_y = t.transform.translation.y
            error_yaw = math.atan2(align_rel_y, align_rel_x)

            if abs(error_yaw) < math.radians(1.0):
                break

            cmd.linear.x = 0.0
            cmd.angular.z = max(-0.3, min(0.3, 0.5 * error_yaw))
            self.vel_publisher.publish(cmd)

        self.stop_robot()
        self.get_logger().info("Phase A complete - heading aligned to cart_frame.")

        # --- PHASE B: drive straight only ---
        self.get_logger().info("Phase B: driving straight to cart_frame (no heading correction)...")
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            try:
                t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            # CHANGED: Shifted target clearance to 0.30m to account for the real robot's structure
            rel_x = t.transform.translation.x + 0.25
            rel_y = t.transform.translation.y  

            if rel_x <= 0.04:
                break

            if abs(rel_y) > 0.05:
                self.get_logger().warn(
                    f"Straight-line drive: lateral drift is {rel_y:.3f}m - "
                    f"not correcting (by design). If this keeps growing, "
                    f"Phase A's alignment tolerance may need tightening."
                )

            error_distance = math.sqrt(rel_x**2 + rel_y**2)
            cmd.linear.x = min(1.0 * error_distance, 0.15)
            cmd.angular.z = 0.0
            self.vel_publisher.publish(cmd)
            time.sleep(0.1)

        self.stop_robot()
        return True

    def set_elevator(self, action="up"):
        # CHANGED: Added wireless reliability burst loop for real warehouse deployment
        msg = String()
        msg.data = action
        self.get_logger().info(f"Publishing elevator command burst: [{action.upper()}]")
        
        for _ in range(3):
            self.elevator_publisher.publish(msg)
            time.sleep(0.1)
            
        time.sleep(4.0)  

    def drive_backwards(self, distance):
        self.get_logger().info(f"Reversing {distance:.2f}m to clear the shelf legs...")
        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = -0.12
        cmd.angular.z = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            distance_moved = math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2)
            if distance_moved >= distance:
                break
            self.vel_publisher.publish(cmd)

        self.stop_robot()
        self.get_logger().info("Clear of the shelf.")

    def update_nav2_footprint(self, dimension_x, dimension_y):
        """Generates a rectangular bounding box string and pushes it to Nav2 Costmaps"""
        half_x = dimension_x / 2.0
        half_y = dimension_y / 2.0
        footprint_string = f"[ [{half_x}, {half_y}], [{half_x}, {-half_y}], [{-half_x}, {-half_y}], [{-half_x}, {half_y}] ]"
        
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'footprint'
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = footprint_string
        req.parameters.append(param)
        
        self.get_logger().info(f"Sending dynamic footprint update request: {footprint_string}")
        if self.global_costmap_client.wait_for_service(timeout_sec=2.0):
            self.global_costmap_client.call_async(req)
        if self.local_costmap_client.wait_for_service(timeout_sec=2.0):
            self.local_costmap_client.call_async(req)
        time.sleep(1.0)
        self.get_logger().info("Nav2 global and local footprints successfully updated.")

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

    # PHASE 1: MOVE TO LOADING STATION
    # CHANGED: Coordinates configured for the real robot loading zone frame layout
    if navigate_to(navigator, x=4.28, y=-0.0632, qz=0.0, qw=1.0, label="loading_position"):
        coordinator = WarehouseMissionCoordinator(navigator)
        navigator.cancelTask()
        time.sleep(0.5)
        
        # PHASE 2: DOCK UNDER SHELF
        coordinator.rotate_to_target()
        
        if coordinator.drive_under_shelf():
            # PHASE 3: LIFT CART
            coordinator.set_elevator(action="up")
            
            # PHASE 3.5: DYNAMICALLY EXPAND ROBOT BOUNDING BOX FOR NAVIGATION
            coordinator.update_nav2_footprint(dimension_x=0.70, dimension_y=0.85)
            
            # PHASE 4: NAVIGATE TO SHIPPING POSE WITH EXPANDED FOOTPRINT
            shipping_success = navigate_to(
                navigator, 
                x=2.08,
                y=1.1,
                qz=0.0,
                qw=1.0,
                label="shipping_position"
            )
            
            if shipping_success:
                print("Cart successfully delivered to warehouse shipping bay!")

                # PHASE 5: UNLOAD
                coordinator.set_elevator(action="down")
                coordinator.drive_backwards(0.6)

                # Reset footprint back down to the robot's normal (no-shelf) size
                coordinator.update_nav2_footprint(dimension_x=0.40, dimension_y=0.40)

                # PHASE 6: RETURN TO INIT POSITION
                print("Shelf delivered! Returning to init_position...")
                return_success = navigate_to(
                    navigator, x=0.022, y=0.00909, qz=0.0, qw=1.0, label="init_position"
                )
                if return_success:
                    print("Mission complete! Robot back at init_position.")
                else:
                    print("Failed to return to init_position.")
            else:
                print("Failed to navigate to physical shipping position.")
        else:
            print("Docking processing failure.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()