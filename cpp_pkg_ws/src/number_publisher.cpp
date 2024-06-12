 /* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

/* Include the message type header for Int64 messages */
#include "example_interfaces/msg/int64.hpp"

/* Define a class that inherits from rclcpp::Node */
class NumberPublisherNode : public rclcpp::Node
{
public:
     /* Constructor, initializing the Node with the name "number_publisher" */
    NumberPublisherNode() : Node("number_publisher") 
    {
        /* Declare a parameter named "number_to_publish" */
        this->declare_parameter("number_to_publish");
        /* Declare a parameter named "publish_frequency" */
        this->declare_parameter("publish_frequency");

         /* Get the value of "number_to_publish" parameter as an integer and store it in number_ */
        number_ = this->get_parameter("number_to_publish").as_int();
        /* Get the value of "publish_frequency" parameter as a double */
        double publish_frequency = this->get_parameter("publish_frequency").as_double();

        /* Create a publisher for Int64 messages on the "number" topic with a queue size of 10 */
        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        /* Create a timer that calls the callback function at a regular interval */
        number_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0/publish_frequency)),
                                                std::bind(&NumberPublisherNode::callbackPublishNumber, this));
         /* Log an informational message */
        RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
    }

private:
    /* Define the callback function for the timer */
    void callbackPublishNumber()
    {   
        /* Create a new local Int64 message object */
        auto msg = example_interfaces::msg::Int64();
        /* Set the message data to the value of number_ */
        msg.data = number_;
        /* Publish the message */
        number_publisher_->publish(msg);
    }

    /* Member variable to store the number to publish */
    int number_;
    /* Shared pointer to the publisher object */
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    /* Shared pointer to the timer object */
    rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char **argv)
{
     /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of NumberPublisherNode */
    auto node = std::make_shared<NumberPublisherNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}