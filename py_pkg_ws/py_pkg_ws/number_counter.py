#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberPublisherNode(Node):
    number_ = 1
    
    def __init__(self):
        super().__init__("number_counter")
        
        self.counter_publisher_ = self.create_publisher(Int64,"number_count", 10)
        self.number_subscriber_ = self.create_subscription(Int64,"number",self.callbackNumber,10)
        self.reset_counter_service_ = self.create_service(SetBool,"reset_counter",self.callbackResetCounter)
        
        self.get_logger().info("Number Counter has been started.")
        
    def callbackNumber(self,msg):
        self.counter_ += msg.data
        newMsg = Int64()
        newMsg.data = self.counter_ 
        
        self.counter_publisher_.publish(newMsg) 
        
    def callbackResetCounter(self,request,response):
        if request.data:
            self.counter = 0     
            response.success = True
            response.message = "Counter has been reset"
        else:
            response.success = False
            response.message = "Counter has not been reset"
        return response
    
           
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()