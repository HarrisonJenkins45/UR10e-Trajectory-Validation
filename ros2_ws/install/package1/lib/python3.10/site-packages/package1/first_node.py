#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#create a class
class MyNode(Node):
	def __init__(self): # Create a constructor 
		super().__init__("first_node") #Call the constructor from the node class
		self.get_logger().info("Hello from ros2!")  


def main(args=None):
	rclpy.init(args=args)

	# Code for the node goes here
	node= MyNode()
	rclpy.spin(node)  # Keeps the node alive until we decide to kill it (with ctr+c)
	rclpy.shutdown

if __name__ == "__main__":
	main()

