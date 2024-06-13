#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
from functools import partial

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from my_custom_interfaces.msg import Turtle
from my_custom_interfaces.msg import TurtleArray
from my_custom_interfaces.srv import KillTurtle

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value 
                   
        self.turtle_to_catch_ = None
        self.pose_ = None
        
        self.new_pos_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.turtle_pos_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callbackNewPos)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callbackALiveTurtles)
        self.control_loop_timer_ = self.create_timer(0.01,self.callbackControlPos)
    
    
    def callbackALiveTurtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
                    
                    if closest_turtle == None or closest_turtle_distance == None:
                        closest_turtle = turtle
                        closest_turtle_distance = distance 
                self.turtle_to_catch_ = closest_turtle                   
            else:
                self.turtle_to_catch_ = msg.turtles[0]
        
        
    def callbackNewPos(self,msg):
        self.pose_ = msg    
     
    def callbackControlPos(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
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
            self.callback_call_KillTurtle(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
                    
        self.new_pos_publisher_.publish(msg)
        
    def callKillTurtle_server(self, turtle_name):
        client = self.create_client(KillTurtle, "kill_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for Server...") 
        
        request = KillTurtle.Request()        
        request.name =  turtle_name
        
        future = client.call_async(request)        
        future.add_done_callback(
            partial(self.callback_call_KillTurtle, turtle_name=turtle_name))  
        
    def callback_call_KillTurtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be killed")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))         
           
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()