#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.desired_x = 0
        self.desired_y = 0

        # Prompt the user to input desired target position (x, y)
        self.get_input()

        # Create Publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # Create Subscriber to get the turtle's current pose
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.control_pid, 10)
        self.get_logger().info("Turtle Controller has been started")

    def get_input(self):
        # User will enter the desired target position here (x, y)
        val_x = input("Enter your value X: ")
        self.desired_x = float(val_x)
        val_y = input("Enter your value Y: ")
        self.desired_y = float(val_y)

    def control_pid(self, pose: Pose):
        msg = Twist()

        # Get current time
        current_time = time.time()
        time.sleep(0.1)  # Optional: this is just to control the frequency of updates
        
        # Here you would calculate the time difference `dt` between the last callback and the current time
        # You should store the `current_time` from this iteration and calculate `dt` in the next iteration
        
        # TODO: Calculate dt (time difference between current and last callback) and store the current time
        
        # Placeholder: You need to implement the PID calculation for linear motion and angular motion

        # For linear velocity (to reach the target x, y)
        # TODO: Calculate error_x, integral_x, and derivative_x, then use the PID formula to calculate msg.linear.x

        # For angular velocity (to adjust the angle towards the target)
        # TODO: Calculate the target_theta, error_theta, integral_theta, and derivative_theta,
        # and then use the PID formula to calculate msg.angular.z

        # Add a condition to stop the robot once it's close enough to the target (use a threshold)
        distance_threshold = 0.2  # Threshold for stopping when close to the target
        
        # TODO: If the turtle is close enough to the target (within the threshold), stop the turtle
        if math.sqrt((self.desired_x - pose.x) ** 2 + (self.desired_y - pose.y) ** 2) < distance_threshold:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish the velocity message to control the turtle
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

