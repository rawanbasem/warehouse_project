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
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # TF system setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.elevator_publisher = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_down_publisher = self.create_publisher(String, '/elevator_down', 10)

        # Service clients to change Nav2 parameters
        self.global_costmap_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters'
        )
        self.local_costmap_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters'
        )

        # Subscriptions
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
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        local_leg_data = []
        leg_flag = False
        leg_start = 0

        # Loop through the laser scan intensities
        for i, intensity in enumerate(msg.intensities):
            if intensity >= 3500.0:  # Kept at 3500.0 as requested
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

        # Empirical correction for the observed leftward heading drift -
        # the robot consistently ends up rotated left of where it should
        # be, so this shifts the target further clockwise to compensate.
        # If it's still off in the same direction after testing, flip the
        yaw_offset_deg = 15.0
        yaw_offset = math.radians(yaw_offset_deg)

        target_yaw = math.atan2(
            math.sin(self.current_yaw - math.pi / 2 - yaw_offset),
            math.cos(self.current_yaw - math.pi / 2 - yaw_offset)
        )
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

        map_trans = None
        map_laser_yaw = 0.0
        try:
            map_trans = self.tf_buffer.lookup_transform(
                'map', 'robot_front_laser_base_link', rclpy.time.Time()
            )
            mq = map_trans.transform.rotation
            map_siny_cosp = 2.0 * (mq.w * mq.z + mq.x * mq.y)
            map_cosy_cosp = 1.0 - 2.0 * (mq.y * mq.y + mq.z * mq.z)
            map_laser_yaw = math.atan2(map_siny_cosp, map_cosy_cosp)
        except Exception as e:
            self.get_logger().warn(f"Could not look up map -> laser transform for logging: {str(e)}")

        def laser_point_to_map(local_x, local_y):
            if map_trans is None:
                return None
            map_x = map_trans.transform.translation.x + local_x * math.cos(map_laser_yaw) - local_y * math.sin(map_laser_yaw)
            map_y = map_trans.transform.translation.y + local_x * math.sin(map_laser_yaw) + local_y * math.cos(map_laser_yaw)
            return map_x, map_y

        for leg_num, (leg_index, leg_range) in enumerate(self.leg_data):
            leg_angle = self.angle_min + leg_index * self.angle_increment
            leg_x = leg_range * math.cos(leg_angle)
            leg_y = leg_range * math.sin(leg_angle)
            map_pt = laser_point_to_map(leg_x, leg_y)
            map_str = f", map_x={map_pt[0]:.3f}, map_y={map_pt[1]:.3f}" if map_pt else ", map=unavailable"
            self.get_logger().info(
                f"Leg {leg_num}: distance={leg_range:.3f}m, x={leg_x:.3f}, y={leg_y:.3f}{map_str}"
            )

        if self.num_legs != 2:
            self.get_logger().error("Not enough legs detected. Aborting server task.")
            return False

        # ---  Triangulation Math ---
        a = self.leg_data[0][1]  # Distance to left leg
        b = self.leg_data[1][1]  # Distance to right leg
        idx_diff = abs(self.leg_data[0][0] - self.leg_data[1][0])
        
        # Theta: Angle between the legs based on full-circle  division
        theta = 2.0 * math.pi * idx_diff / self.total_readings
        # C: Distance between the two legs
        c = math.sqrt(a * a + b * b - 2.0 * a * b * math.cos(theta))
        # D: Distance from robot to the midpoint of C
        d = 0.5 * math.sqrt(2.0 * (a * a + b * b) - c * c)
        # Alpha & Beta angle 
        alpha = math.asin(max(-1.0, min(1.0, a * math.sin(theta) / c)))
        beta = math.asin(max(-1.0, min(1.0, (c / 2.0) * math.sin(alpha) / d)))
        
        x = d * math.sin(alpha + beta)
        y = d * math.cos(alpha + beta)

        midpoint_map = laser_point_to_map(x, y)
        midpoint_map_str = (
            f", map_x={midpoint_map[0]:.3f}, map_y={midpoint_map[1]:.3f}"
            if midpoint_map else ", map=unavailable"
        )
        self.get_logger().info(
            f"Triangulation calculated Local Laser coordinates: X={x:.3f}, Y={y:.3f}{midpoint_map_str}"
        )

        # --- Transform coordinates from laser frame to robot_odom frame ---
        try:
            trans = self.tf_buffer.lookup_transform(
                'robot_odom', 'robot_front_laser_base_link', rclpy.time.Time()
            )
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            laser_yaw = math.atan2(siny_cosp, cosy_cosp)

            # Compute absolute odom map coordinates based on local laser offsets
            odom_cart_x = trans.transform.translation.x + x * math.cos(laser_yaw) - y * math.sin(laser_yaw)
            odom_cart_y = trans.transform.translation.y + x * math.sin(laser_yaw) + y * math.cos(laser_yaw)

            # Create transform message
            ts = TransformStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = 'robot_odom'  
            ts.child_frame_id = 'cart_frame'
            ts.transform.translation.x = odom_cart_x
            ts.transform.translation.y = odom_cart_y
            ts.transform.rotation.w = 1.0

            self.tf_static_broadcaster.sendTransform(ts)
            self.get_logger().info(f"Broadcasted cart_frame to robot_odom tree at X: {odom_cart_x:.3f}, Y: {odom_cart_y:.3f}")

            cart_map = laser_point_to_map(x, y)
            if cart_map:
                self.get_logger().info(
                    f"cart_frame in map coordinates (approx): X: {cart_map[0]:.3f}, Y: {cart_map[1]:.3f}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast TF frame: {str(e)}")
            return False

        time.sleep(0.5)

        # --- PHASE A: Stationary Alignment Loop ---
        self.get_logger().info("Stationary aligning heading to cart_frame...")
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
        self.get_logger().info("Heading aligned.")

        # --- PHASE B: Straight-Line Driving Loop ---
        self.get_logger().info("Driving straight to cart_frame...")
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            try:
                t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            rel_x = t.transform.translation.x #+ 0.25
            rel_y = t.transform.translation.y

            if rel_x <= 0.04:
                break

            error_distance = math.sqrt(rel_x**2 + rel_y**2)
            cmd.linear.x = min(1.0 * error_distance, 0.15)
            cmd.angular.z = 0.0
            self.vel_publisher.publish(cmd)
            time.sleep(0.1)

        self.stop_robot()
        return True
        
    def set_elevator(self, action="up"):
        msg = String()
        msg.data = action
        publisher = self.elevator_publisher if action == "up" else self.elevator_down_publisher
        self.get_logger().info(f"Publishing elevator command burst: [{action.upper()}]")

        for _ in range(3):
            publisher.publish(msg)
            time.sleep(0.1)

        time.sleep(4.0)

    def update_nav2_footprint(self, dimension_x, dimension_y):
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

def navigate_to(navigator, x, y, qz, qw, label, z=0.0, qx=0.0, qy=0.0):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
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
    if navigate_to(navigator, x=4.65, y=-0.05560, qz=0.0, qw=1.0, label="loading_position"):
        coordinator = WarehouseMissionCoordinator(navigator)
        navigator.cancelTask()
        time.sleep(0.5)

        # PHASE 2: DOCK UNDER SHELF
        coordinator.rotate_to_target()

        if coordinator.drive_under_shelf():
            # PHASE 3: LIFT CART
            coordinator.set_elevator(action="up")

            # EXPAND ROBOT BOX FOR NAVIGATION
            coordinator.update_nav2_footprint(dimension_x=0.70, dimension_y=0.85)

            # PHASE 4: NAVIGATE TO SHIPPING POSE WITH EXPANDED FOOTPRINT
            shipping_success = navigate_to(
                navigator, 
                x=2.12,      
                y=1.24,       
                qz=0.0,
                qw=1.0,
                label="shipping_position"
            )

            if shipping_success:
                print("Cart successfully delivered to warehouse shipping bay!")

                # PHASE 5: UNLOAD
                coordinator.set_elevator(action="down")
                print("Waiting for elevator plate mechanism to lower completely...")
                time.sleep(4.0)

                # --- LOOP FOR ROBOT REVERSING TO CLEAR OUT THE SHELF ---
                print("Reversing to clear shelf framework safely...")

                rclpy.spin_once(coordinator, timeout_sec=0.1)
                start_x = coordinator.current_x
                start_y = coordinator.current_y

                reverse_cmd = Twist()
                reverse_cmd.linear.x = -0.15  
                reverse_cmd.angular.z = 0.0

                target_distance = 1.20
                while rclpy.ok():
                    rclpy.spin_once(coordinator, timeout_sec=0.01)

                    distance_moved = math.sqrt(
                        (coordinator.current_x - start_x) ** 2 + 
                        (coordinator.current_y - start_y) ** 2
                    )

                    if distance_moved >= target_distance:
                        print(f"Target distance reached: {distance_moved:.2f}m. Stopping.")
                        break

                    coordinator.vel_publisher.publish(reverse_cmd)
                    time.sleep(0.05)

                coordinator.stop_robot()
                time.sleep(1.0)
                print("Clear of the shelf structure.")

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
                print("Failed to navigate to shipping position.")

        else:
            print("Docking processing failure.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()