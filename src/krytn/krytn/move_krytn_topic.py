#!/bin/python3

import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
 
rclpy.init()

# Create node and setup
node = Node("krytn_mover")
pub = node.create_publisher(PoseStamped, '/goal_pose', qos_profile=1)
clock = node.get_clock()
logger = node.get_logger()

# Wait 1 second
time.sleep(1.0)    

# Construct a new message goal pose. 
point = PoseStamped()
    
point.pose.position.x = -3.07
point.pose.position.y = -4.0
point.header.frame_id ='map'
point.header.stamp = clock.now().to_msg()


# Publish the message and also make a note on the info log stream. 
pub.publish(point)
logger.info("published goal")

# Wait 1 second then shutdown 
time.sleep(1.0)
rclpy.shutdown()
  