 /* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

/* Include the message type header for Int64 messages */
#include "example_interfaces/msg/int64.hpp"
/* Include the service type header for SetBool services */
#include "example_interfaces/srv/set_bool.hpp"

/* Using placeholders for binding callbacks */
using std::placeholders::_1;
using std::placeholders::_2;

/* Define a class that inherits from rclcpp::Node */
class NumberCounterNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "number_counter" and initializing counter_ to 0 */
    NumberCounterNode() : Node("number_counter"), counter_{0}
    {
        /* Create a publisher for Int64 messages on the "number_count" topic with a queue size of 10 */
        counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        /* Create a subscription to the "number" topic and Bind the callback function for received messages */
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        /* Create a service for resetting the counter and Bind the callback function for for the service*/
        reset_counter_service_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));

        /* Log an informational message */
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    /* Define the callback function for the number subscription */
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        /* Increment the counter by the received message data */
        counter_ += msg->data;
        /* Create a new local Int64 message object */
        auto newMsg = example_interfaces::msg::Int64();
        /* Set the message data to the updated counter value */
        newMsg.data = counter_;
        /* Publish the updated counter value */
        counter_publisher_->publish(newMsg);
    }

    /* Define the callback function for the reset counter service */
    void callbackResetCounter(example_interfaces::srv::SetBool::Request::SharedPtr request,
                              example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data) /* If client wants to reset counter */
        {
             /* Reset the counter */
            counter_ = 0;
            /* Set the response success to true */
            response->success = true;
            /* Set the response message */
            response->message = "Counter has been reset";
        }
        else /* If client doesn't want to reset counter */
        {
            /* Set the response success to false */
            response->success = false; 
             /* Set the response message */
            response->message = "Counter has not been reset";
        }
    }

    /* Member variable to store the counter value */
    int counter_;
    /* Shared pointer to the publisher object */
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
    /* Shared pointer to the subscription object */
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
     /* Shared pointer to the service object */
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_service_;
};

int main(int argc, char **argv)
{
    /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of NumberCounterNode */
    auto node = std::make_shared<NumberCounterNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}