#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
import os
import matplotlib.pyplot as plt

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")

        # Get target position from user input
        self.desired_x, self.desired_y = self.get_input()

        # Publisher & Subscriber
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.control_pid, 10)

        self.get_logger().info("Turtle PID Controller has been started")

        # PID gains for linear velocity (distance)
        self.kp_dist = 1.0
        self.ki_dist = 0.0
        self.kd_dist = 0.1

        # PID gains for angular velocity (heading)
        self.kp_ang = 6.0
        self.ki_ang = 0.0
        self.kd_ang = 0.3

        # PID state variables
        self.prev_dist_error = 0.0
        self.integral_dist_error = 0.0

        self.prev_ang_error = 0.0
        self.integral_ang_error = 0.0

        self.prev_time = None

        # Data logging
        self.x_data = []
        self.y_data = []
        self.v_data = []
        self.w_data = []
        self.time_data = []

        # Control flags
        self.reached_target = False

    def get_input(self):
        # Ask user for target within turtlesim bounds
        while True:
            val_x = float(input("Enter your value X (0 to 11): "))
            if 0 <= val_x <= 11:
                break
            print("X must be between 0 and 11.")
        while True:
            val_y = float(input("Enter your value Y (0 to 11): "))
            if 0 <= val_y <= 11:
                break
            print("Y must be between 0 and 11.")
        return val_x, val_y

    def control_pid(self, pose: Pose):
        if self.reached_target:
            return  # Skip processing once reached

        msg = Twist()

        current_time = time.time()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
        self.prev_time = current_time

        # Distance and heading errors
        dist_error = math.sqrt((self.desired_x - pose.x) ** 2 + (self.desired_y - pose.y) ** 2)
        desired_theta = math.atan2(self.desired_y - pose.y, self.desired_x - pose.x)
        ang_error = desired_theta - pose.theta
        # Normalize angle to [-pi, pi]
        ang_error = math.atan2(math.sin(ang_error), math.cos(ang_error))

        # PID for linear velocity
        self.integral_dist_error += dist_error * dt
        derivative_dist_error = (dist_error - self.prev_dist_error) / dt if dt > 0 else 0.0
        v = (self.kp_dist * dist_error +
             self.ki_dist * self.integral_dist_error +
             self.kd_dist * derivative_dist_error)
        self.prev_dist_error = dist_error

        # PID for angular velocity
        self.integral_ang_error += ang_error * dt
        derivative_ang_error = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
        w = (self.kp_ang * ang_error +
             self.ki_ang * self.integral_ang_error +
             self.kd_ang * derivative_ang_error)
        self.prev_ang_error = ang_error

        # Velocity limits
        max_v = 2.0
        max_w = 4.0
        v = max(min(v, max_v), -max_v)
        w = max(min(w, max_w), -max_w)

        # Stop condition near target
        distance_threshold = 0.2
        if dist_error < distance_threshold:
            v = 0.0
            w = 0.0
            self.reached_target = True
            self.get_logger().info("Target reached. Stopping and preparing to save plots.")
            # Publish zero velocity one last time
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(msg)
            # After short delay, save plot and shutdown
            self.create_timer(1.0, self.shutdown_and_save)
            return

        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub_.publish(msg)

        # Log data for plotting
        self.x_data.append(pose.x)
        self.y_data.append(pose.y)
        self.v_data.append(v)
        self.w_data.append(w)
        self.time_data.append(current_time)

    def shutdown_and_save(self):
        self.get_logger().info("Saving plots and shutting down.")
        self.save_plots()
        os._exit(0)  

    def save_plots(self):
        if not self.time_data:
            self.get_logger().warn("No data to save!")
            return

        t0 = self.time_data[0]
        times = [t - t0 for t in self.time_data]

        plt.figure(figsize=(12, 5))

        # Position plot
        plt.subplot(1, 2, 1)
        plt.plot(times, self.x_data, label='x')
        plt.plot(times, self.y_data, label='y')
        plt.xlabel('Time [s]')
        plt.ylabel('Position')
        plt.title('Turtle Position over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig("position_plot.png")

        # Control inputs plot
        plt.subplot(1, 2, 2)
        plt.plot(times, self.v_data, label='Linear velocity v')
        plt.plot(times, self.w_data, label='Angular velocity ω')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity')
        plt.title('Control Inputs over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig("control_inputs_plot.png")

        plt.close()  # Close plots so they don't display

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
