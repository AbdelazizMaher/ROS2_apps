/* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

#include <cmath>

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include "my_custom_interfaces/msg/turtle.hpp"
#include "my_custom_interfaces/msg/turtle_array.hpp"
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
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 0.33);

        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        alive_turtles_publisher_ = this->create_publisher<my_custom_interfaces::msg::TurtleArray>("alive_turtles", 10);

        spawn_turtle_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / spawn_frequency_)),
                                                      std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));

        kill_turtle_service_ = this->create_service<my_custom_interfaces::srv::KillTurtle>(
            "kill_turtle", std::bind(&TurtleSpawnerNode::callbackKillTurtle, this, _1, _2));
    }

private:
    void callbackKillTurtle(const my_custom_interfaces::srv::KillTurtle::Request::SharedPtr request,
                            const my_custom_interfaces::srv::KillTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(
            std::thread(std::bind(&TurtleSpawnerNode::callKillServer, this, request->name)));

        response->success = true;
    }

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

            alive_turtles_.erase(
                std::remove_if(alive_turtles_.begin(), alive_turtles_.end(),
                               [&](const my_custom_interfaces::msg::Turtle &turtle)
                               { return turtle.name == turtle_name; }),
                alive_turtles_.end());

            publishAliveTurtles();    
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    void publishAliveTurtles()
    {
        auto msg = my_custom_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles_;

        alive_turtles_publisher_->publish(msg);
    }

    void spawnNewTurtle()
    {
        turtle_counter_ += 1;

        auto turtle_name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        double theta = randomDouble() * 2 * M_PI;

        spawn_turtle_threads_.push_back(
            std::thread(std::bind(&TurtleSpawnerNode::callTurtleSpawner_server, this, x, y, theta, turtle_name)));
    }

    double randomDouble()
    {
        return static_cast<double>(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    void callTurtleSpawner_server(double x, double y, double theta, std::string turtle_name)
    {
        /* Create a client for the SetBool service named "spawn" */
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

        /* Create a shared pointer to a new Spawn request */
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

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

            if (response->name != "")
            {
                RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", turtle_name.c_str());

                auto new_alive_turtle = my_custom_interfaces::msg::Turtle();
                new_alive_turtle.x = x;
                new_alive_turtle.y = y;
                new_alive_turtle.theta = theta;
                new_alive_turtle.name = turtle_name;
                alive_turtles_.push_back(new_alive_turtle);

                publishAliveTurtles();
            }
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    int turtle_counter_;

    std::string turtle_name_prefix_;

    double spawn_frequency_;

    std::vector<my_custom_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::Publisher<my_custom_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;

    rclcpp::Service<my_custom_interfaces::srv::KillTurtle>::SharedPtr kill_turtle_service_;

    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;

    std::vector<std::thread> spawn_turtle_threads_;
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