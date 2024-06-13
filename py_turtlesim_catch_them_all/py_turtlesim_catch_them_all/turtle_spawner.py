#!/usr/bin/env python3

import math
import random
from functools import partial

import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

class TurtleSpawner(Node):
    
    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")
        
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
            
        self.turtle_counter_ = 0
        
        self.spawn_turtle_timer_ = self.create_timer(1.0/self.spawn_frequency, self.spawn_new_turtle) 
        
    def spawn_new_turtle(self):
        self.turtle_counter += 1
        turtle_name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        
        theta = random.uniform(0.0, 2*math.pi) 
               
        self.callTurtleSpawner_server(x, y, theta, turtle_name)    
        
    def callTurtleSpawner_server(self, x, y, theta, turtle_name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for Server...") 
        
        request = Spawn.Request()
        
        request.x = x
        request.y = y
        request.theta = theta
        request.name =  turtle_name
        
        future = client.call_async(request)
        
        future.add_done_callback(
            partial(self.callback_call_spawn, x=x, y=y, theta=theta, turtle_name=turtle_name))  
        
    def callback_call_spawn(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " is now alive")
            
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
                
         

           
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()