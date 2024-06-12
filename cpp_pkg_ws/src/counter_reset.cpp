 /* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

/* Include the message type header for Int64 messages */
#include "example_interfaces/msg/int64.hpp"
/* Include the service type header for SetBool services */
#include "example_interfaces/srv/set_bool.hpp"

/* Define a class that inherits from rclcpp::Node */
class CounterResetNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "counter_reset" */
    CounterResetNode() : Node("counter_reset")
    {
        /* Create and start a new thread that calls the callCounterReset function with the argument true */
        thread1_ = std::thread(std::bind(&CounterResetNode::callCounterReset, this, true));

        //threads_.push_back(std::thread(std::bind(&CounterResetNode::callCounterReset, this, true)));

    }

    /* Define the function to call the reset_counter service */
    void callCounterReset(bool reset)
    {
        /* Create a client for the SetBool service named "reset_counter" */
        auto client = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

         /* Create a shared pointer to a new SetBool request */
        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
         /* Set the request data to the value of the reset argument */
        request->data = reset;

        /* Send the request asynchronously and get a future to track the response */
        auto future = client->async_send_request(request);

        /* Catch any exceptions that occur during the service call */
        try
        {
            /* Wait for the response (blocking) and get the result */
            auto response = future.get();
            /* Log the response success and message */
            RCLCPP_INFO(this->get_logger(), "Response success: %d, message: %s", response->success, response->message);
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

private:
    /* Member variable to store the first thread */
    std::thread thread1_;
    /* Member variable(simple thread pool) to store additional threads (if needed) */
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of CounterResetNode */
    auto node = std::make_shared<CounterResetNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}