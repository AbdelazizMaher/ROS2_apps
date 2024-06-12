#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        
        self.target_x = 4
        self.target_y = 8
        
        self.pose_ = None
        
        self.new_pos_publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.turtle_pos_subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.callbackNewPos)
        self.control_loop_timer_ = self.create_timer(0.01,self.callbackControlPos)
    
    def callbackNewPos(self,msg):
        self.pose_ = msg    
     
    def callbackControlPos(self):
        if self.pose_ == None:
            return
        
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        
        msg = Twist()
        
        if distance > 0.5:
            msg._linear.x = 2*distance
            
            theta = math.atan2(dist_y,dist_x)
            diff = theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
                
            msg.angular.z = 6*diff    
        else:
            msg.linear.x = 0.0
            msg.angular.z =0.0
            
        
        self.new_pos_publisher_.publish(msg)
           
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()