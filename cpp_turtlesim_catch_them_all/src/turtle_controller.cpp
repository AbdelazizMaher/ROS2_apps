/* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

/* Define a class that inherits from rclcpp::Node */
class TurtleControllerNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "turtle_controller" */
    TurtleControllerNode() : Node("turtle_controller")
    {
        new_pos_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        turtle_pos_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackNewPos, this,std::placeholders::_1));
    }

private:
    void callbackNewPos(const turtlesim::msg::Pose::SharedPtr msg)
    {
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr new_pos_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pos_subscriber_;
};

int main(int argc, char **argv)
{
    /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of TurtleControllerNode */
    auto node = std::make_shared<TurtleControllerNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}