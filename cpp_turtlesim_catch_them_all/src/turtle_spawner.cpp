/* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

/* Include for mathematical functions */
#include <cmath>

/* Include the service type header for Spawn services */
#include "turtlesim/srv/spawn.hpp"
/* Include the service type header for Kill services */
#include "turtlesim/srv/kill.hpp"

/* Include the custom message type header for Turtle messages */
#include "my_custom_interfaces/msg/turtle.hpp"
/* Include the custom message type header for TurtleArray messages */
#include "my_custom_interfaces/msg/turtle_array.hpp"
/* Include the custom service type header for KillTurtle services */
#include "my_custom_interfaces/srv/kill_turtle.hpp"

/* Using placeholders for binding callbacks */
using std::placeholders::_1;
using std::placeholders::_2;

/* Define a class that inherits from rclcpp::Node */
class TurtleSpawnerNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "turtle_spawner" */
    TurtleSpawnerNode() : Node("turtle_spawner"), turtle_counter_{0}
    {
        /* Declare parameters for the turtle name prefix and spawn frequency */
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 1.0);

        /* Get the parameters' values */
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        /* Create a publisher for the alive turtles */
        alive_turtles_publisher_ = this->create_publisher<my_custom_interfaces::msg::TurtleArray>("alive_turtles", 10);

        /* Create a timer to periodically spawn new turtles */
        spawn_turtle_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / spawn_frequency_)),
                                                      std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));

        /* Create a service to kill turtles */
        kill_turtle_service_ = this->create_service<my_custom_interfaces::srv::KillTurtle>(
            "kill_turtle", std::bind(&TurtleSpawnerNode::callbackKillTurtle, this, _1, _2));
    }

private:
    /* Callback function to handle kill turtle requests */
    void callbackKillTurtle(const my_custom_interfaces::srv::KillTurtle::Request::SharedPtr request,
                            const my_custom_interfaces::srv::KillTurtle::Response::SharedPtr response)
    {
        /* Create a new thread to call the kill service */
        kill_turtle_threads_.push_back(
            std::thread(std::bind(&TurtleSpawnerNode::callKillServer, this, request->name)));

        /* Set the response success to true */
        response->success = true;
    }

    /* Function to call the kill service */
    void callKillServer(std::string turtle_name)
    {
        /* Create a client for the SetBool service named "kill" */
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

        /* Create a shared pointer to a new Spawn request */
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();

        /* Set the request data to the value of the turtle_name argument */
        request->name = turtle_name;

        /* Send the request asynchronously and get a future to track the response */
        auto future = client->async_send_request(request);

        /* Catch any exceptions that occur during the service call */
        try
        {
            /* Wait for the response (blocking) and get the result */
            future.get();
            /* Remove the turtle from the list of alive turtles */
            alive_turtles_.erase(
                std::remove_if(alive_turtles_.begin(), alive_turtles_.end(),
                               [&](const my_custom_interfaces::msg::Turtle &turtle)
                               { return turtle.name == turtle_name; }),
                alive_turtles_.end());

            /* Publish the updated list of alive turtles */
            publishAliveTurtles();
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    /* Function to publish the list of alive turtles */
    void publishAliveTurtles()
    {
        /* Create a TurtleArray message */
        auto msg = my_custom_interfaces::msg::TurtleArray();
        /* Set the turtles field to the list of alive turtles */
        msg.turtles = alive_turtles_;

        /* Publish the message */
        alive_turtles_publisher_->publish(msg);
    }

    /* Function to spawn a new turtle */
    void spawnNewTurtle()
    {
        /* Increment the turtle counter */
        turtle_counter_ += 1;

        /* Generate a new turtle name */
        auto turtle_name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        /* Generate random x, y, and theta values */
        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        double theta = randomDouble() * 2 * M_PI;

        /* Create a new thread to call the spawn service */
        spawn_turtle_threads_.push_back(
            std::thread(std::bind(&TurtleSpawnerNode::callTurtleSpawner_server, this, x, y, theta, turtle_name)));
    }

    /* Function to generate a random double value */
    double randomDouble()
    {
        return static_cast<double>(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    /* Function to call the spawn service */
    void callTurtleSpawner_server(double x, double y, double theta, std::string turtle_name)
    {
        /* Create a client for the SetBool service named "spawn" */
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

        /* Create a shared pointer to a new Spawn request */
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        /* Set the request x, y, and theta values*/
        request->x = x;
        request->y = y;
        request->theta = theta;
        /* Set the request data to the value of the turtle_name argument */
        request->name = turtle_name;

        /* Send the request asynchronously and get a future to track the response */
        auto future = client->async_send_request(request);

        /* Catch any exceptions that occur during the service call */
        try
        {
            /* Wait for the response (blocking) and get the result */
            auto response = future.get();

            /* Check if the turtle was successfully spawned */
            if (response->name != "")
            {
                RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", turtle_name.c_str());

                /* Add the new turtle to the list of alive turtles */
                auto new_alive_turtle = my_custom_interfaces::msg::Turtle();
                new_alive_turtle.x = x;
                new_alive_turtle.y = y;
                new_alive_turtle.theta = theta;
                new_alive_turtle.name = turtle_name;
                alive_turtles_.push_back(new_alive_turtle);

                /* Publish the updated list of alive turtles */
                publishAliveTurtles();
            }
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    /* Counter for the number of turtles spawned */
    int turtle_counter_;
    /* Prefix for the names of spawned turtles */
    std::string turtle_name_prefix_;
    /* Frequency at which turtles are spawned */
    double spawn_frequency_;
    /* List of alive turtles */
    std::vector<my_custom_interfaces::msg::Turtle> alive_turtles_;
    /* Publisher for the alive turtles */
    rclcpp::Publisher<my_custom_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    /* Service for killing turtles */
    rclcpp::Service<my_custom_interfaces::srv::KillTurtle>::SharedPtr kill_turtle_service_;
    /* Timer for spawning turtles */
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
    /* Vector of threads for spawning turtles */
    std::vector<std::thread> spawn_turtle_threads_;
    /* Vector of threads for killing turtles */
    std::vector<std::thread> kill_turtle_threads_;
};

int main(int argc, char **argv)
{
    /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of TurtleSpawnerNode */
    auto node = std::make_shared<TurtleSpawnerNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}