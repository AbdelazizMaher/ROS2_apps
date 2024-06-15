/* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

/* Include for mathematical functions */
#include <cmath>

/* Include the Twist message type header */
#include "geometry_msgs/msg/twist.hpp"
/* Include the Pose message type header */
#include "turtlesim/msg/pose.hpp"

/* Include the custom message type header for Turtle messages */
#include "my_custom_interfaces/msg/turtle.hpp"
/* Include the custom message type header for TurtleArray messages */
#include "my_custom_interfaces/msg/turtle_array.hpp"
/* Include the custom service type header for KillTurtle services */
#include "my_custom_interfaces/srv/kill_turtle.hpp"

/* Define a class that inherits from rclcpp::Node */
class TurtleControllerNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "turtle_controller" */
    TurtleControllerNode() : Node("turtle_controller")
    {
        /* Declare parameter for catching the closest turtle first */
        this->declare_parameter("catch_closest_turtle_first", true);
        /* Get the parameter value */
        catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();

        /* Create a publisher for controlling turtle movement */
        new_pos_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        /* Create a subscriber to receive turtle pose updates */
        turtle_pos_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackNewPos, this, std::placeholders::_1));

        /* Create a subscriber to receive updates on alive turtles */
        alive_turtles_subscriber_ = this->create_subscription<my_custom_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::callbackALiveTurtles, this, std::placeholders::_1));

        /* Create a timer to control the main loop of the controller */
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::callbackControlPos, this));
    }

private:
    /* Callback function to handle updates on alive turtles */
    void callbackALiveTurtles(const my_custom_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
        {
            /* Determine which turtle to catch based on configuration */
            if (catch_closest_turtle_first_)
            {
                auto closest_turtle = msg->turtles.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);
                /* Find the closest turtle */
                for (auto turtle : msg->turtles)
                {
                    double distance = getDistanceFromCurrentPose(turtle);

                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = turtle;
                        closest_turtle_distance = distance;
                    }
                }
                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                /* Always catch the first turtle in the list */
                turtle_to_catch_ = msg->turtles.at(0);
            }
        }
    }

    /* function to calculate the distance from the current pose to a given turtle */
    double getDistanceFromCurrentPose(my_custom_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    /* Callback function to handle updates on the turtle's pose */
    void callbackNewPos(const turtlesim::msg::Pose::SharedPtr msg)
    {
        /* Get pose message from the raw pointer from the shared pointer */
        pose_ = *msg.get();
    }

    void callbackControlPos()
    {
        /* Check if there is a turtle to catch */
        if (turtle_to_catch_.name == "")
        {
            return;
        }

        /* Calculate distance to the turtle to catch */
        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        /* Create a Twist message to control turtle movement */
        auto msg = geometry_msgs::msg::Twist();

        /* If the turtle is far away, move towards it */
        if (distance > 0.5)
        {
            /* Set linear velocity( directly proportional with distance) */
            msg.linear.x = 2 * distance;

            /* Calculate steering angle and adjust orientation */
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            /* Adjust orientation */
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }

            /* Set angular velocity( directly proportional with theta) */
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            /* If close enough, stop moving and attempt to catch the turtle */
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            /* Create a thread to call the service to catch the turtle */
            catch_turtle_threads_.push_back(
                std::thread(std::bind(&TurtleControllerNode::callKillTurtle_server, this, turtle_to_catch_.name)));

            /* Clear the name of the turtle being caught */
            turtle_to_catch_.name = "";
        }
        /* Publish the twist message to control turtle movement */
        new_pos_publisher_->publish(msg);
    }

    void callKillTurtle_server(std::string turtle_name)
    {
        /* Create a client for the SetBool service named "kill_turtle" */
        auto client = this->create_client<my_custom_interfaces::srv::KillTurtle>("kill_turtle");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

        /* Create a shared pointer to a new KillTurtle request */
        auto request = std::make_shared<my_custom_interfaces::srv::KillTurtle::Request>();
        /* Set the request data to the value of the turtle_name argument */
        request->name = turtle_name;

        /* Send the request asynchronously and get a future to track the response */
        auto future = client->async_send_request(request);

        /* Catch any exceptions that occur during the service call */
        try
        {
            /* Wait for the response (blocking) and get the result */
            auto response = future.get();
            /* Check if the turtle was successfully killed */
            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "turtle %s could not be killed", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    /* Flag to determine if the closest turtle should be caught first */
    bool catch_closest_turtle_first_;
    /* Data structure to hold information about the turtle to catch */
    my_custom_interfaces::msg::Turtle turtle_to_catch_;
    /* Current pose of the turtle controlled by this node */
    turtlesim::msg::Pose pose_;
    /* Publisher to control the movement of the turtle */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr new_pos_publisher_;
    /* Subscriber to receive updates on alive turtles */
    rclcpp::Subscription<my_custom_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    /* Subscriber to receive updates on the turtle's pose */
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pos_subscriber_;
    /* Timer for the main control loop of the controller */
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    /* Vector of threads to handle catching turtles */
    std::vector<std::thread> catch_turtle_threads_;
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